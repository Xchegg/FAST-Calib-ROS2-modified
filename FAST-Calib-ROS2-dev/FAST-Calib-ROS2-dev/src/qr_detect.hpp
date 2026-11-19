/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef QR_DETECT_HPP
#define QR_DETECT_HPP
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp> // 显式引入以支持新版 drawFrameAxes
#include "common_lib.h"

// 自动适配新版 OpenCV 命名空间和对象的宏
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
  #include <opencv2/objdetect/aruco_detector.hpp>
  #define NEW_OPENCV_ARUCO
#endif

class QRDetect
{
private:
  double marker_size_, delta_width_qr_center_, delta_height_qr_center_;
  double delta_width_circles_, delta_height_circles_;
  int min_detected_markers_;
#ifdef NEW_OPENCV_ARUCO
  cv::aruco::Dictionary dictionary_; // 新版中 Dictionary 为实体对象而非 Ptr
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
#endif
  rclcpp::Node::SharedPtr node_;

public:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr qr_pub_;
  cv::Mat imageCopy_;
  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;

  QRDetect(rclcpp::Node::SharedPtr node, Params &params)
  {
    node_ = node;
    marker_size_ = params.marker_size;
    delta_width_qr_center_ = params.delta_width_qr_center;
    delta_height_qr_center_ = params.delta_height_qr_center;
    delta_width_circles_ = params.delta_width_circles;
    delta_height_circles_ = params.delta_height_circles;
    min_detected_markers_ = params.min_detected_markers;

    // Initialize camera matrix
    cameraMatrix_ = (cv::Mat_<float>(3, 3) << params.fx, 0, params.cx,
                     0, params.fy, params.cy,
                     0, 0, 1);

    // Initialize distortion coefficients
    distCoeffs_ = (cv::Mat_<float>(1, 5) << params.k1, params.k2, params.p1, params.p2, 0);

    // Initialize QR dictionary
#ifdef NEW_OPENCV_ARUCO
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
#else
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
#endif

    qr_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("qr_cloud", 1);
  }

  Point2f projectPointDist(cv::Point3f pt_cv, const Mat intrinsics, const Mat distCoeffs)
  {
    // Project a 3D point taking into account distortion
    vector<Point3f> input{pt_cv};
    vector<Point2f> projectedPoints;
    projectedPoints.resize(1); // TODO: Do it batched? (cv::circle is not batched anyway)
    projectPoints(input, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1),
                  intrinsics, distCoeffs, projectedPoints);
    return projectedPoints[0];
  }

  void comb(int N, int K, std::vector<std::vector<int>> &groups)
  {
    int upper_factorial = 1;
    int lower_factorial = 1;

    for (int i = 0; i < K; i++)
    {
      upper_factorial *= (N - i);
      lower_factorial *= (K - i);
    }
    int n_permutations = upper_factorial / lower_factorial;

    if (DEBUG)
      cout << N << " centers found. Iterating over " << n_permutations
           << " possible sets of candidates" << endl;

    std::string bitmask(K, 1); // K leading 1's
    bitmask.resize(N, 0);      // N-K trailing 0's

    // print integers and permute bitmask
    do
    {
      std::vector<int> group;
      for (int i = 0; i < N; ++i) // [0..N-1] integers
      {
        if (bitmask[i])
        {
          group.push_back(i);
        }
      }
      groups.push_back(group);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

    assert(groups.size() == n_permutations);
  }

  void detect_qr(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud)
  {
    image.copyTo(imageCopy_);

    std::vector<std::vector<cv::Point3f>> boardCorners;
    std::vector<cv::Point3f> boardCircleCenters;
    float width = delta_width_qr_center_;
    float height = delta_height_qr_center_;
    float circle_width = delta_width_circles_ / 2.;
    float circle_height = delta_height_circles_ / 2.;
    boardCorners.resize(4);
    for (int i = 0; i < 4; ++i)
    {
      int x_qr_center = (i % 3) == 0 ? -1 : 1; 
      int y_qr_center = (i < 2) ? 1 : -1; 
      float x_center = x_qr_center * width;
      float y_center = y_qr_center * height;

      cv::Point3f circleCenter3d(x_qr_center * circle_width,
                                 y_qr_center * circle_height, 0);
      boardCircleCenters.push_back(circleCenter3d);
      for (int j = 0; j < 4; ++j)
      {
        int x_qr = (j % 3) == 0 ? -1 : 1; 
        int y_qr = (j < 2) ? 1 : -1;      
        cv::Point3f pt3d(x_center + x_qr * marker_size_ / 2.,
                         y_center + y_qr * marker_size_ / 2., 0);
        boardCorners[i].push_back(pt3d);
      }
    }

    std::vector<int> boardIds{1, 2, 4, 3}; 

    // 修复：针对新旧 OpenCV 处理 Board 对象的差异
#ifdef NEW_OPENCV_ARUCO
    cv::Ptr<cv::aruco::Board> board = cv::makePtr<cv::aruco::Board>(boardCorners, dictionary_, boardIds);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#else
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(boardCorners, dictionary_, boardIds);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  #if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    parameters->doCornerRefinement = true;
  #else
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  #endif
#endif

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
#ifdef NEW_OPENCV_ARUCO
    cv::aruco::ArucoDetector detector(dictionary_, *parameters);
    detector.detectMarkers(image, corners, ids);
#else
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters);
#endif

    // Draw detections if at least one marker detected
    if (ids.size() > 0) {
#ifdef NEW_OPENCV_ARUCO
      cv::aruco::drawDetectedMarkers(imageCopy_, corners, ids);
#else
      cv::aruco::drawDetectedMarkers(imageCopy_, corners, ids);
#endif
    }

    cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0); 

    if (ids.size() >= (size_t)min_detected_markers_ && ids.size() <= TARGET_NUM_CIRCLES)
    {
      // Estimate 3D position of the markers
      vector<Vec3d> rvecs, tvecs;
      Vec3f rvec_sin, rvec_cos;
      
      // 注意：新版 OpenCV 的 ArucoDetector 不再自带 estimatePoseSingleMarkers，
      // 但由于它本质是基于单点 PnP，源码通过旧函数或通用 OpenCV 逻辑来确保向前兼容。
      // 为保证直接可用，此处采用适配宏处理：
#ifdef NEW_OPENCV_ARUCO
      // 新版直接使用循环对单 marker 进行 solvePnP 估计位姿
      rvecs.resize(corners.size());
      tvecs.resize(corners.size());
      std::vector<cv::Point3f> singleMarkerObjPoints;
      singleMarkerObjPoints.push_back(cv::Point3f(-marker_size_/2.f,  marker_size_/2.f, 0));
      singleMarkerObjPoints.push_back(cv::Point3f( marker_size_/2.f,  marker_size_/2.f, 0));
      singleMarkerObjPoints.push_back(cv::Point3f( marker_size_/2.f, -marker_size_/2.f, 0));
      singleMarkerObjPoints.push_back(cv::Point3f(-marker_size_/2.f, -marker_size_/2.f, 0));
      for (size_t i = 0; i < corners.size(); i++) {
          cv::solvePnP(singleMarkerObjPoints, corners[i], cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i]);
      }
#else
      cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, cameraMatrix_,
                                           distCoeffs_, rvecs, tvecs);
#endif

      // Draw markers' axis and centers in color image (Debug purposes)
      for (size_t i = 0; i < ids.size(); i++)
      {
#ifdef NEW_OPENCV_ARUCO
        cv::drawFrameAxes(imageCopy_, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.1);
#else
        cv::aruco::drawAxis(imageCopy_, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.1);
#endif

        // Accumulate pose for initial guess
        tvec[0] += tvecs[i][0];
        tvec[1] += tvecs[i][1];
        tvec[2] += tvecs[i][2];
        rvec_sin[0] += sin(rvecs[i][0]);
        rvec_sin[1] += sin(rvecs[i][1]);
        rvec_sin[2] += sin(rvecs[i][2]);
        rvec_cos[0] += cos(rvecs[i][0]);
        rvec_cos[1] += cos(rvecs[i][1]);
        rvec_cos[2] += cos(rvecs[i][2]);
      }

      // Compute average pose.
      tvec = tvec / int(ids.size());
      rvec_sin = rvec_sin / int(ids.size()); 
      rvec_cos = rvec_cos / int(ids.size()); 
      rvec[0] = atan2(rvec_sin[0], rvec_cos[0]);
      rvec[1] = atan2(rvec_sin[1], rvec_cos[1]);
      rvec[2] = atan2(rvec_sin[2], rvec_cos[2]);

      pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// Estimate 3D position of the board using detected markers
#ifdef NEW_OPENCV_ARUCO
      // 新版中 Board 位姿估计被集成至 match 或独立的 PnP
      std::vector<cv::Point3f> objPoints;
      std::vector<cv::Point2f> imgPoints;
      // 提取被检测到的 marker 对应的 3D 标定物坐标点和 2D 像素坐标点
      for (size_t i = 0; i < ids.size(); i++) {
          int idx = -1;
          for (size_t j = 0; j < boardIds.size(); j++) {
              if (boardIds[j] == ids[i]) { idx = j; break; }
          }
          if (idx != -1) {
              for (int k = 0; k < 4; k++) {
                  objPoints.push_back(boardCorners[idx][k]);
                  imgPoints.push_back(corners[i][k]);
              }
          }
      }
      bool valid = false;
      if (!objPoints.empty()) {
          valid = cv::solvePnP(objPoints, imgPoints, cameraMatrix_, distCoeffs_, rvec, tvec, true);
      }
#else
  #if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
      int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_,
                                               distCoeffs_, rvec, tvec);
  #else
      int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_,
                                               distCoeffs_, rvec, tvec, true);
  #endif
#endif

#ifdef NEW_OPENCV_ARUCO
      cv::drawFrameAxes(imageCopy_, cameraMatrix_, distCoeffs_, rvec, tvec, 0.2);
#else
      cv::aruco::drawAxis(imageCopy_, cameraMatrix_, distCoeffs_, rvec, tvec, 0.2);
#endif

      // Build transformation matrix to calibration target axis
      cv::Mat R(3, 3, cv::DataType<float>::type);
      cv::Rodrigues(rvec, R);

      cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);
      t.at<float>(0) = tvec[0];
      t.at<float>(1) = tvec[1];
      t.at<float>(2) = tvec[2];

      cv::Mat board_transform = cv::Mat::eye(3, 4, CV_32F);
      R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
      t.copyTo(board_transform.rowRange(0, 3).col(3));

      // Compute coordintates of circle centers
      for (size_t i = 0; i < boardCircleCenters.size(); ++i)
      {
        cv::Mat mat = cv::Mat::zeros(4, 1, CV_32F);
        mat.at<float>(0, 0) = boardCircleCenters[i].x;
        mat.at<float>(1, 0) = boardCircleCenters[i].y;
        mat.at<float>(2, 0) = boardCircleCenters[i].z;
        mat.at<float>(3, 0) = 1.0;

        // Transform center to target coords
        cv::Mat mat_qr = board_transform * mat;
        cv::Point3f center3d;
        center3d.x = mat_qr.at<float>(0, 0);
        center3d.y = mat_qr.at<float>(1, 0);
        center3d.z = mat_qr.at<float>(2, 0);

        // Draw center (DEBUG)
        cv::Point2f uv;
        uv = projectPointDist(center3d, cameraMatrix_, distCoeffs_);
        circle(imageCopy_, uv, 5, Scalar(0, 255, 0), -1);

        // Add center to list
        pcl::PointXYZ qr_center;
        qr_center.x = center3d.x;
        qr_center.y = center3d.y;
        qr_center.z = center3d.z;
        candidates_cloud->push_back(qr_center);
      }

      std::vector<std::vector<int>> groups;
      comb(candidates_cloud->size(), TARGET_NUM_CIRCLES, groups);
      std::vector<double> groups_scores(groups.size(), 0.0);

      for (size_t i = 0; i < groups.size(); ++i)
      {
        std::vector<pcl::PointXYZ> candidates;
        for (size_t j = 0; j < groups[i].size(); ++j)
        {
          pcl::PointXYZ center;
          center.x = candidates_cloud->at(groups[i][j]).x;
          center.y = candidates_cloud->at(groups[i][j]).y;
          center.z = candidates_cloud->at(groups[i][j]).z;
          candidates.push_back(center);
        }

        Square square_candidate(candidates, delta_width_circles_,
                                delta_height_circles_);
        groups_scores[i] = square_candidate.is_valid() ? 1.0 : -1.0; 
      }

      int best_candidate_idx = -1;
      double best_candidate_score = -1;
      for (size_t i = 0; i < groups.size(); ++i)
      {
        if (best_candidate_score == 1 && groups_scores[i] == 1)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "[Mono] More than one set of candidates fit target's geometry. "
                       "Please, make sure your parameters are well set. Exiting callback");
          return;
        }
        if (groups_scores[i] > best_candidate_score)
        {
          best_candidate_score = groups_scores[i];
          best_candidate_idx = (int)i;
        }
      }

      if (best_candidate_idx == -1)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[Mono] Unable to find a candidate set that matches target's geometry");
        return;
      }

      for (size_t j = 0; j < groups[best_candidate_idx].size(); ++j)
      {
        centers_cloud->push_back(candidates_cloud->at(groups[best_candidate_idx][j]));
      }

      if (DEBUG)
      { 
        for (size_t i = 0; i < centers_cloud->size(); i++)
        {
          cv::Point3f pt_circle1(centers_cloud->at(i).x, centers_cloud->at(i).y, centers_cloud->at(i).z);
          cv::Point2f uv_circle1;
          uv_circle1 = projectPointDist(pt_circle1, cameraMatrix_, distCoeffs_);
          circle(imageCopy_, uv_circle1, 2, Scalar(255, 0, 255), -1);
        }
      }
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "%lu marker(s) found, %d expected. Skipping frame...", ids.size(),
                  TARGET_NUM_CIRCLES);
    }
  }
};
typedef std::shared_ptr<QRDetect> QRDetectPtr;

#endif