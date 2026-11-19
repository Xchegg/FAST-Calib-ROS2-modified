/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mono_qr_pattern");

  // 读取参数
  Params params = loadParameters(node);

  // 初始化 QR 检测和 LiDAR 检测
  QRDetectPtr qrDetectPtr;
  qrDetectPtr.reset(new QRDetect(node, params));

  LidarDetectPtr lidarDetectPtr;
  lidarDetectPtr.reset(new LidarDetect(node, params));

  DataPreprocessPtr dataPreprocessPtr;
  dataPreprocessPtr.reset(new DataPreprocess(params));

  // 读取图像和点云
  cv::Mat img_input = dataPreprocessPtr->img_input_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;

  // 检测 QR 码（相机侧，只检测一次）
  pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  qr_center_cloud->reserve(4);
  qrDetectPtr->detect_qr(img_input, qr_center_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers(new pcl::PointCloud<pcl::PointXYZ>);
  sortPatternCenters(qr_center_cloud, qr_centers, "camera");

  if (qr_centers->size() != 4)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "QR 检测到 %ld 个圆心（需要 4 个），请检查图像与 marker 参数。",
                 qr_centers->size());
  }

  // 发布原始点云（未滤波），方便在 RViz 中查看标定板位置并拖动包围盒。
  // 用 transient_local（latched）发布一次，即使 RViz 后启动也能收到完整点云。
  auto raw_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud", rclcpp::QoS(1).transient_local());
  {
    sensor_msgs::msg::PointCloud2 raw_msg;
    pcl::toROSMsg(*cloud_input, raw_msg);
    raw_msg.header.stamp = node->now();
    raw_msg.header.frame_id = "map";
    raw_cloud_pub->publish(raw_msg);
  }

  auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
  auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

  // ------------------------------------------------------------------
  // 交互式滤波包围盒：在 RViz 中拖动盒子、右键菜单调整大小
  // ------------------------------------------------------------------
  const std::string BOX_NAME = "calib_filter_box";

  // 盒子状态（中心 + 尺寸），由 yaml 的初始滤波范围推导
  double box_cx = (params.x_min + params.x_max) / 2.0;
  double box_cy = (params.y_min + params.y_max) / 2.0;
  double box_cz = (params.z_min + params.z_max) / 2.0;
  double box_sx = params.x_max - params.x_min;
  double box_sy = params.y_max - params.y_min;
  double box_sz = params.z_max - params.z_min;

  // 保存最近一次标定结果，供菜单“保存结果”和定时重发使用
  Eigen::Matrix4f latest_transform = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // 把当前结果发布到 RViz（也会被定时器调用，保证 RViz 后启动仍能看到）
  auto publish_results = [&]()
  {
    if (!DEBUG) return;
    auto stamp = node->now();
    auto pub = [&](auto &publisher, const auto &cloud)
    {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*cloud, msg);
      msg.header.stamp = stamp;
      msg.header.frame_id = "map";
      publisher->publish(msg);
    };
    pub(raw_cloud_pub, cloud_input);
    pub(qrDetectPtr->qr_pub_, qr_centers);
    pub(lidarDetectPtr->center_pub_, latest_lidar_centers);
    pub(lidarDetectPtr->filtered_pub_, lidarDetectPtr->getFilteredCloud());
    pub(lidarDetectPtr->plane_pub_, lidarDetectPtr->getPlaneCloud());
    pub(lidarDetectPtr->aligned_pub_, lidarDetectPtr->getAlignedCloud());
    pub(lidarDetectPtr->edge_pub_, lidarDetectPtr->getEdgeCloud());
    pub(lidarDetectPtr->center_z0_pub_, lidarDetectPtr->getCenterZ0Cloud());
    pub(aligned_lidar_centers_pub, latest_aligned_lidar_centers);
    pub(colored_cloud_pub, latest_colored_cloud);
  };

  // 前向声明，避免 lambda 循环依赖
  std::function<void(bool)> run_calibration;

  auto apply_limits_and_run = [&]()
  {
    lidarDetectPtr->setFilterLimits(
        box_cx - box_sx / 2.0, box_cx + box_sx / 2.0,
        box_cy - box_sy / 2.0, box_cy + box_sy / 2.0,
        box_cz - box_sz / 2.0, box_cz + box_sz / 2.0);
    run_calibration(false);
  };

  // 拖动盒子的回调
  auto on_pose_update = [&](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb)
  {
    if (fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
    {
      box_cx = fb->pose.position.x;
      box_cy = fb->pose.position.y;
      box_cz = fb->pose.position.z;
      apply_limits_and_run();
    }
  };

  auto marker_server = std::make_shared<interactive_markers::InteractiveMarkerServer>("calib_filter_box", node);
  interactive_markers::MenuHandler menu;

  std::function<void()> rebuild_box = [&]()
  {
    visualization_msgs::msg::InteractiveMarker m;
    m.header.frame_id = "map";
    m.header.stamp = node->now();
    m.name = BOX_NAME;
    m.description = "标定板滤波范围：拖动移动，右键菜单调整大小";
    m.pose.position.x = box_cx;
    m.pose.position.y = box_cy;
    m.pose.position.z = box_cz;
    m.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker cube;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.scale.x = box_sx;
    cube.scale.y = box_sy;
    cube.scale.z = box_sz;
    cube.color.r = 0.0f;
    cube.color.g = 1.0f;
    cube.color.b = 0.0f;
    cube.color.a = 0.25f;

    visualization_msgs::msg::InteractiveMarkerControl ctrl;
    ctrl.name = "move_box";
    ctrl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    ctrl.always_visible = true;
    ctrl.markers.push_back(cube);
    m.controls.push_back(ctrl);

    marker_server->insert(m, on_pose_update);
    menu.apply(*marker_server, BOX_NAME);
    marker_server->applyChanges();
  };

  // 右键菜单：调整尺寸（0.1 / 0.5 / 1 / 5 m 步进）/ 保存结果
  auto make_resize = [&](int axis, double delta)
  {
    return [&, axis, delta](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &)
    {
      double *s = (axis == 0) ? &box_sx : ((axis == 1) ? &box_sy : &box_sz);
      *s += delta;
      if (*s < 0.05) *s = 0.05;
      apply_limits_and_run();
      rebuild_box();
    };
  };

  struct Step
  {
    const char *label;
    double value;
  };
  const Step steps[] = {{"0.1", 0.1}, {"0.5", 0.5}, {"1", 1.0}, {"5", 5.0}};
  const char *axis_names[] = {"X", "Y", "Z"};
  for (int a = 0; a < 3; ++a)
  {
    auto axis_handle = menu.insert(std::string("调整 ") + axis_names[a] + " 轴 (Resize " + axis_names[a] + ")");
    for (const auto &st : steps)
    {
      menu.insert(axis_handle, std::string("+") + st.label + " m", make_resize(a, st.value));
      menu.insert(axis_handle, std::string("-") + st.label + " m", make_resize(a, -st.value));
    }
  }

  menu.insert("保存标定结果 (Save result)", [&](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &)
  {
    if (latest_colored_cloud && !latest_colored_cloud->empty())
    {
      saveCalibrationResults(params, latest_transform, latest_colored_cloud, qrDetectPtr->imageCopy_);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "还没有可用的标定结果，请先拖动包围盒完成一次标定。");
    }
  });

  menu.insert("保存调试点云 (Debug save)", [&](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &)
  {
    RCLCPP_INFO(node->get_logger(), "以当前包围盒重跑并保存中间点云 filtered/plane/aligned/edge ...");
    run_calibration(true);
  });

  // 核心标定流程（save=true 时写结果文件与中间点云）
  run_calibration = [&](bool save)
  {
    lidarDetectPtr->save_clouds_ = save;

    // 1. LiDAR 检测（使用当前滤波范围）
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_center_cloud->reserve(4);
    lidarDetectPtr->detect_lidar(cloud_input, lidar_center_cloud);

    // 2. 排序
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");
    latest_lidar_centers = lidar_centers;

    if (lidar_centers->size() != 4)
    {
      RCLCPP_WARN(node->get_logger(),
                  "LiDAR 检测到 %ld 个圆心（需要 4 个），请调整包围盒把标定板框进去。",
                  lidar_centers->size());
      publish_results();
      return;
    }

    // 3. 计算外参
    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);
    latest_transform = transformation;

    // 4. RMSE
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);
    latest_aligned_lidar_centers = aligned_lidar_centers;
    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0)
      RCLCPP_INFO(node->get_logger(), "[Result] RMSE: %.4f m", rmse);

    RCLCPP_INFO(node->get_logger(), "[Result] Extrinsic parameters T_cam_lidar:");
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    // 5. 彩色点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_,
                             qrDetectPtr->distCoeffs_, img_input, colored_cloud);
    latest_colored_cloud = colored_cloud;

    // 每次得到有效标定都保存外参（轻量文本）；彩色点云较大，仅在初始/菜单“保存标定结果”时写盘
    saveExtrinsics(params, transformation, qrDetectPtr->imageCopy_);
    if (save)
      saveColoredCloud(params, colored_cloud);

    // 6. 发布结果
    publish_results();
  };

  // 显示盒子并做一次初始标定（写结果文件，行为与原来一致）
  rebuild_box();
  run_calibration(true);

  RCLCPP_INFO(node->get_logger(),
              "在 RViz 中拖动绿色包围盒（或右键调整大小）即可实时重新标定。");

  // 1 Hz 定时重发结果，避免 RViz 启动时机晚于首次发布而看不到点云
  auto publish_timer = node->create_wall_timer(std::chrono::seconds(1), publish_results);

  // 阻塞处理交互标记反馈与定时器
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
