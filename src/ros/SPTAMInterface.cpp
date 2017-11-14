/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#include "SPTAMInterface.hpp"
#include "../sptam/FeatureExtractorThread.hpp"
#include "../sptam/utils/macros.hpp"
#include "../sptam/utils/tf_utils.hpp"
#include "../sptam/utils/projective_math.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include "parameters_opencv2.hpp"
#elif CV_MAJOR_VERSION == 3
  #include "parameters_opencv3.hpp"
#endif

#ifdef USE_LOOPCLOSURE
#include "../sptam/loopclosing/LoopClosing.hpp"
#include "../sptam/loopclosing/LCDetector.hpp"
#include "../sptam/loopclosing/detectors/DLDBriefLoopDetector.hpp"
#endif

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Profiler.hpp"
  #include "../sptam/utils/log/Logger.hpp"
#endif // SHOW_PROFILING

#define INITIAL_COVARIANCE ( Eigen::Matrix6d::Identity() * 1e-9 )

namespace std
{

// TODO part of the c++14 standard
template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args )
{
  return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

}

// Convert from CameraPose to tf2::Pose (position and orientation)
inline void CameraPose2TFPose(const CameraPose& cameraPose, tf2::Transform& pose)
{
  const Eigen::Quaterniond& orientation = cameraPose.GetOrientationQuaternion();
  const Eigen::Vector3d&  position = cameraPose.GetPosition();
  pose.setOrigin(tf2::Vector3(position[0], position[1], position[2]));
  pose.setRotation(tf2::Quaternion( orientation.x(), orientation.y(), orientation.z(), orientation.w() )); // q = (x,y,z,w)
}

// Convert from tf2::Pose to CameraPose (position and orientation)
inline void TFPose2CameraPose(const tf2::Transform& pose, CameraPose& cameraPose)
{
  // convert to position opencv vector
  tf2::Vector3 position_tf = pose.getOrigin();
  Eigen::Vector3d position(position_tf.getX(), position_tf.getY(), position_tf.getZ());

  // Convert to orientation opencv quaternion
  tf2::Quaternion orientation_tf = pose.getRotation();
  Eigen::Quaterniond orientation(orientation_tf.getW(), orientation_tf.getX(), orientation_tf.getY(), orientation_tf.getZ());

  // TODO is it OK to use the Id covariance here?
  cameraPose = CameraPose(position, orientation, Eigen::Matrix6d::Identity());
}

// ================================================================== //

sptam::SPTAMInterface::SPTAMInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp)
  : sptam_( nullptr )
  , motionModel_( new MotionModel(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), INITIAL_COVARIANCE) )
  , lastImageSeq_( 0 )
  , odom_to_map_( tf2::Transform::getIdentity() )
  , transform_thread_( nullptr )
  , transform_listener_( tfBuffer_ )
{
  #if CV_MAJOR_VERSION == 2
    // Load Nonfree OpenCv modules (SURF,SIFT)
    cv::initModule_nonfree();
  #endif // CV_MAJOR_VERSION

  // Get node parameters
  nhp.param<std::string>("odom_frame", odom_frame_, "odom");
  nhp.param<std::string>("base_link_frame", base_frame_, "base_link");
  nhp.param<std::string>("camera_frame", camera_frame_, "camera");
  nhp.param<std::string>("map_frame", map_frame_, "map");
  nhp.param<bool>("use_odometry", use_odometry_, false);
  nhp.param<double>("transform_publish_freq", transform_publish_freq_, 30.0);
  nhp.param<double>("tf_delay", tf_delay_, 1.0/transform_publish_freq_);

  bool use_approx_sync;
  nhp.param<bool>("approximate_sync", use_approx_sync, false);

  // Load salient point detector implementation from configuration.
  std::string detector_name;
  {
    nhp.param<std::string>("FeatureDetector/Name", detector_name, "GFTT");

    featureDetector_ = loadFeatureDetector( nhp, detector_name, "FeatureDetector" );
  }

  // Load descriptor extractor implementation from configuration.
  std::string descriptor_name;
  {
    nhp.param<std::string>("DescriptorExtractor/Name", descriptor_name, "BRIEF");

    descriptorExtractor_ = loadDescriptorExtractor( nhp, descriptor_name, "DescriptorExtractor" );
  }

  // load descriptor matcher
  std::string matcherName;
  {
    nhp.param<std::string>("DescriptorMatcher/Name", matcherName, "BruteForce-Hamming");

    sptam_params_.descriptorMatcher = loadDescriptorMatcher(nhp, matcherName, "DescriptorMatcher" );
  }

  // Load Parameters
  // nhp.param is not overloaded for unsigned int
  int matchingCellSizeParam, matchingNeighborhoodParam;
  nhp.param<int>("MatchingCellSize", matchingCellSizeParam, 30);
  sptam_params_.matchingCellSize = matchingCellSizeParam;
  nhp.param<double>("MatchingDistance", sptam_params_.matchingDistanceThreshold, 25.0);
  nhp.param<int>("MatchingNeighborhood", matchingNeighborhoodParam, 1);
  sptam_params_.matchingNeighborhoodThreshold = matchingNeighborhoodParam;
  nhp.param<double>("EpipolarDistance", sptam_params_.epipolarDistanceThreshold, 0.0);

  // Camera Calibration Parameters
  nhp.param<double>("FrustumNearPlaneDist", frustum_near_plane_distance_, 0.1);
  nhp.param<double>("FrustumFarPlaneDist", frustum_far_plane_distance_, 1000.0);

  // BA parameters
  int nKeyFramesToAdjustByLocal, maxIterationsLocal;
  nhp.param<int>("BundleAdjustmentActiveKeyframes", nKeyFramesToAdjustByLocal, 10);
  sptam_params_.nKeyFramesToAdjustByLocal = nKeyFramesToAdjustByLocal;
  nhp.param<int>("maxIterationsLocal", maxIterationsLocal, 20);
  sptam_params_.maxIterationsLocal = maxIterationsLocal;
  
  #ifdef USE_LOOPCLOSURE
  loop_detector_ = nullptr;
  // Loop detector vocabulary parameters
  if (nhp.hasParam("LoopDetectorVocabulary"))
    if(descriptor_name.compare("BRIEF") == 0){
      std::string lcd_vocabulary;
      nhp.param<std::string>("LoopDetectorVocabulary", lcd_vocabulary, "");
      DLDBriefLoopDetector::Parameters lcd_param;
      ROS_INFO_STREAM("Loop Detector initializing, loading vocabulary");
      loop_detector_.reset(new DLDBriefLoopDetector(lcd_vocabulary, lcd_param));
    }
  #endif

  // write configuration to log file
  #ifdef SHOW_PROFILING
    Logger::Write( "#   matchingCellSize: " + std::to_string(sptam_params_.matchingCellSize) + "\n" );
    Logger::Write( "#   matchingNeighborhoodThreshold: " + std::to_string(sptam_params_.matchingNeighborhoodThreshold) + "\n" );
    Logger::Write( "#   matchingDistanceThreshold: " + std::to_string(sptam_params_.matchingDistanceThreshold) + "\n" );
    Logger::Write( "#   epipolarDistanceThreshold: " + std::to_string(sptam_params_.epipolarDistanceThreshold) + "\n" );
    Logger::Write( "#   FrustumNearPlaneDist: " + std::to_string( frustum_near_plane_distance_ ) + "\n" );
    Logger::Write( "#   FrustumFarPlaneDist: " + std::to_string( frustum_far_plane_distance_ ) + "\n" );
    Logger::Write( "#   BundleAdjustmentActiveKeyframes: " + std::to_string( sptam_params_.nKeyFramesToAdjustByLocal ) + "\n" );
  #endif

  // Create RowMatcher instance
  rowMatcher_ = std::make_unique<RowMatcher>( sptam_params_.matchingDistanceThreshold, sptam_params_.descriptorMatcher, sptam_params_.epipolarDistanceThreshold );

  // Subscribe to images messages

  sub_img_l_.subscribe(nhp, "/stereo/left/image_rect", 1);
  sub_info_l_.subscribe(nhp, "/stereo/left/camera_info", 1);
  sub_img_r_.subscribe(nhp, "/stereo/right/image_rect", 1);
  sub_info_r_.subscribe(nhp, "/stereo/right/camera_info", 1);

  if ( use_approx_sync )
  {
    approximate_sync_.reset( new ApproximateSync( ApproximatePolicy(10),
      sub_img_l_, sub_info_l_,
      sub_img_r_, sub_info_r_
    ) );

    approximate_sync_->registerCallback( boost::bind(
      &SPTAMInterface::onImages, this, _1, _2, _3, _4
    ) );
  }
  else
  {
    exact_sync_.reset( new ExactSync( ExactPolicy(1),
      sub_img_l_, sub_info_l_,
      sub_img_r_, sub_info_r_
    ) );

    exact_sync_->registerCallback( boost::bind(
      &SPTAMInterface::onImages, this, _1, _2, _3, _4
    ) );
  }

  mapPub_ = nhp.advertise<sensor_msgs::PointCloud2>("point_cloud", 100);
  posePub_ = nhp.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot/pose", 100);
  keyframesPub_ = nhp.advertise<nav_msgs::Path>("keyframes", 100);

  stereoFrame_Pub_ = nhp.advertise<sensor_msgs::Image>("frame_before_refine", 100);
  stereoFrameAfter_Pub_ = nhp.advertise<sensor_msgs::Image>("frame_after_refine", 100);

  // The node will publish the transformation map_frame_ -> published_frame_
  // to tf. If using odometry this will be the odom_frame_ frame and if not, then base_frame_.
  published_frame_ = use_odometry_ ? odom_frame_ : base_frame_;

  // start transform publisher thread
  transform_thread_ = std::make_unique<std::thread>( boost::bind(&sptam::SPTAMInterface::publishTransformLoop, this) );

  ROS_INFO("S-PTAM node initialized.");
}

sptam::SPTAMInterface::~SPTAMInterface()
{
  // we use couts because ROS is dying
  std::cout << "starting sptam node cleanup..." << std::endl;

  std::cout << "stopping sptam threads..." << std::endl;
  sptam_->stop();

  #ifdef SHOW_PROFILING
    for ( const auto& mapPoint : sptam_->GetMap().getMapPoints() ) {
      WriteToLog( " tk MeasurementCount: ", mapPoint->measurements().size() );
    }
  #endif

  #ifdef SHOW_PROFILING
    for ( const auto& keyFrame : sptam_->GetMap().getKeyframes() ) {
      CameraPose keyFramePose = keyFrame->GetCameraPose();
      WriteToLog("BASE_LINK_KF:", keyFrame->GetId(), keyFramePose.GetPosition(), keyFramePose.GetOrientationMatrix(), keyFramePose.covariance());
    }
  #endif

  // create map file
  {
    std::ofstream out("map cloud.dat");
    for ( const auto& point : sptam_->GetMap().getMapPoints() )
      out << point->GetPosition().x() << " " << point->GetPosition().y() <<  " " << point->GetPosition().z() << std::endl;
    out.close();
  }

  // transform_thread_ is null if odometry is disabled
  if ( transform_thread_ )
  {
    std::cout << "wait for transform publisher thread to join..." << std::endl;
    transform_thread_->join();
  }

  std::cout << "done!" << std::endl;

  // sleep for one second
  ros::Duration( 1.0 ).sleep();
}

bool sptam::SPTAMInterface::getBaseLinkPose(const CameraPose& cameraPose, const ros::Time& t, tf2::Transform& base_to_map)
{
  tf2::Transform base_to_camera;
  if (not lookupTransformSafe(tfBuffer_, camera_frame_, base_frame_, t, base_to_camera))
    return false;

  tf2::Transform camera_to_map;
  CameraPose2TFPose( cameraPose, camera_to_map );

  base_to_map = camera_to_map * base_to_camera;

  return true;
}

void sptam::SPTAMInterface::fixOdomFrame(const CameraPose& cameraPose, const tf2::Transform& camera_to_odom, const ros::Time& t)
{
  tf2::Transform camera_to_map;
  CameraPose2TFPose( cameraPose, camera_to_map );

  // compute the new difference between map and odom
  tf2::Transform odom_to_map = camera_to_map * camera_to_odom.inverse();

  std::lock_guard<std::mutex> lock( odom_to_map_mutex_ );
  odom_to_map_ = odom_to_map;
}

void sptam::SPTAMInterface::onImages(
  const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
  const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info
)
{
  size_t currentSeq = img_msg_left->header.seq;
  ros::Time currentTime = img_msg_left->header.stamp;
  //ros::Time currentTime = ros::Time::now(); // Gaston: level7_full has a bug on headers stamps (they jump into the future)
  
  // Check if a image message was missed
  ROS_DEBUG_STREAM_COND((currentSeq - lastImageSeq_) > 1, "STETEREO FRAME WAS MISSED! current: " << currentSeq << " last: " << lastImageSeq_);
  lastImageSeq_ = currentSeq;

  // If using odometry, try to get a new estimate for the current pose.
  // if not using odometry, camera_to_odom is left blank.
  tf2::Transform camera_to_odom;
  if ( use_odometry_ )
  {
    if ( not lookupTransformSafe(tfBuffer_, odom_frame_, camera_frame_, currentTime, camera_to_odom) )
    {
      ROS_WARN("Ignoring this pair of images.");
      return;
    }

    tf2::Transform camera_to_map = odom_to_map_ * camera_to_odom;

    TFPose2CameraPose(camera_to_map, cameraPose_);
  }
  else
  {
    Eigen::Vector3d currentCameraPosition;
    Eigen::Quaterniond currentCameraOrientation;
    Eigen::Matrix6d covariance;
    motionModel_->predictPose(currentTime, currentCameraPosition, currentCameraOrientation, covariance);
    cameraPose_ = CameraPose( currentCameraPosition, currentCameraOrientation, covariance );
  }

  // If SPTAM has not been initialized yet, do it
  if ( not sptam_ )
  {
    ROS_INFO_STREAM("init calib");

    // Extract camera parameters from cameraInfo messages.
    loadCameraCalibration(left_info, right_info);

    // Create SPTAM instance. The reason we can't initialize it
    // in the constructor is because camera parameters arrive
    // with the first cameraInfo messages.
    sptam_ = std::make_unique<SPTAM>(
      *cameraParametersLeft_, *cameraParametersRight_, stereo_baseline_,
      *rowMatcher_, sptam_params_
      #ifdef USE_LOOPCLOSURE
      ,motionModel_, loop_detector_
      #endif
    );
  }

  // convert image to OpenCv cv::Mat format
  cv_bridge::CvImageConstPtr bridgeLeft_ptr = cv_bridge::toCvShare(img_msg_left, "rgb8");
  cv_bridge::CvImageConstPtr bridgeRight_ptr = cv_bridge::toCvShare(img_msg_right, "rgb8");

  // save images
  cv::Mat imageLeft(bridgeLeft_ptr->image, left_roi_);
  cv::Mat imageRight(bridgeRight_ptr->image, right_roi_);

  #ifdef SHOW_PROFILING
      double start, end;
      start = GetSeg();
  #endif // SHOW_PROFILING

  #ifdef SHOW_PROFILING
    double startStep, endStep;
    startStep = GetSeg();
  #endif

  // Extract features

  FeatureExtractorThread featureExtractorThreadLeft(imageLeft, *featureDetector_, *descriptorExtractor_);
  FeatureExtractorThread featureExtractorThreadRight(imageRight, *featureDetector_, *descriptorExtractor_);

  featureExtractorThreadLeft.WaitUntilFinished();
  const std::vector<cv::KeyPoint>& keyPointsLeft = featureExtractorThreadLeft.GetKeyPoints();
  const cv::Mat& descriptorsLeft = featureExtractorThreadLeft.GetDescriptors();

  featureExtractorThreadRight.WaitUntilFinished();
  const std::vector<cv::KeyPoint>& keyPointsRight = featureExtractorThreadRight.GetKeyPoints();
  const cv::Mat& descriptorsRight = featureExtractorThreadRight.GetDescriptors();

  ImageFeatures imageFeaturesLeft(cv::Size(left_info->width, left_info->height), keyPointsLeft, descriptorsLeft, sptam_params_.matchingCellSize);
  ImageFeatures imageFeaturesRight(cv::Size(right_info->width, right_info->height), keyPointsRight, descriptorsRight, sptam_params_.matchingCellSize);

  //  std::vector<cv::DMatch> cvMatches;
  //  rowMatcher_->match(keyPointsLeft, descriptorsLeft, keyPointsRight, descriptorsRight, cvMatches);

  //  cv::Mat imageMatches;
  //  cv::drawMatches(imageLeft,keyPointsLeft,imageRight,keyPointsRight,cvMatches, imageMatches);

  //  cv::imshow("Initial Matches",imageMatches);
  //  cv::waitKey(0);

  #ifdef SHOW_PROFILING
    endStep = GetSeg();
    WriteToLog(" tk Extract: ", startStep, endStep);
  #endif

  // if the map was not initialized, try to build it
  if( not sptam_->isInitialized() )
  {
    ROS_INFO("Trying to intialize map...");

    if ( not sptam_->initFromStereo(cameraPose_, imageFeaturesLeft, imageFeaturesRight) )
      return;

    std::cout << "Points initialized from stereo: " << sptam_->GetMap().getMapPoints().size() << std::endl;
  }

  // if the map is already initialized, do tracking
  else
  {
    #ifdef SHOW_PROFILING
    WriteToLog("ESTIMATED_CAMERA_POSE:", currentSeq, cameraPose_.GetPosition(), cameraPose_.GetOrientationMatrix(), cameraPose_.covariance());
    #endif

    TrackingReport tracking_report = sptam_->track(
      currentSeq, cameraPose_, imageFeaturesLeft, imageFeaturesRight
      #ifdef SHOW_TRACKED_FRAMES
      , imageLeft, imageRight
      #endif
    );

    cameraPose_ = tracking_report.refinedCameraPose;

    #ifdef SHOW_PROFILING
      WriteToLog("REFINED_CAMERA_POSE:", currentSeq, cameraPose_.GetPosition(), cameraPose_.GetOrientationMatrix(), cameraPose_.covariance() );
    #endif

    #ifdef SHOW_PROFILING
      end = GetSeg();
      WriteToLog(" tk trackingtotal: ", start, end);
    #endif

    // fix the error between map and odom to correct the current pose.
    if ( use_odometry_ )
      fixOdomFrame( cameraPose_, camera_to_odom, currentTime );
    else
    // if odometry is disabled, odom_to_map_ actually contains the map->cam transform because I'm too lazy
    {
      motionModel_->updatePose(currentTime, cameraPose_.GetPosition(), cameraPose_.GetOrientationQuaternion(), cameraPose_.covariance());

      tf2::Transform cam_to_map;
      CameraPose2TFPose(cameraPose_, cam_to_map);

      // Get the transformation between the base_frame and the camera_frame
      tf2::Transform base_to_camera;
      if ( lookupTransformSafe(tfBuffer_, camera_frame_, base_frame_, currentTime, base_to_camera) )
      {
        std::lock_guard<std::mutex> lock( odom_to_map_mutex_ );
        odom_to_map_ = cam_to_map * base_to_camera;
      }
      else
      {
        ROS_WARN("Skipping tf state update.");
      }
    }

    // Publish processed camera frames
    drawStereoFrames(tracking_report);
  }

  // Publish Map To be drawn by rviz visualizer
  publishMap();
  publishKFs(currentSeq, currentTime);

  // Publish the camera Pose
  publishPose( currentSeq, currentTime, cameraPose_ );
}

void sptam::SPTAMInterface::loadCameraCalibration(
  const sensor_msgs::CameraInfoConstPtr& left_info,
  const sensor_msgs::CameraInfoConstPtr& right_info
)
{
  // Check if a valid calibration exists
  if (left_info->K[0] == 0.0) {
    ROS_ERROR("The camera is not calibrated");
    return;
  }

  // Ponemos que el frame id de las camara info sea el mismo
  sensor_msgs::CameraInfoPtr left_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*left_info);
  sensor_msgs::CameraInfoPtr right_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*right_info);
  left_info_copy->header.frame_id = "stereo";
  right_info_copy->header.frame_id = "stereo";

  // Get Stereo Camera Model from Camera Info message
  image_geometry::StereoCameraModel stereoCameraModel;
  stereoCameraModel.fromCameraInfo(left_info_copy, right_info_copy);

  // Get PinHole Camera Model from the Stereo Camera Model
  const image_geometry::PinholeCameraModel& cameraLeft = stereoCameraModel.left();
  const image_geometry::PinholeCameraModel& cameraRight = stereoCameraModel.right();

  // Get rectify intrinsic Matrix (is the same for both cameras because they are rectify)
  cv::Mat projectionLeft = cv::Mat( cameraLeft.projectionMatrix() );
  cv::Matx33d intrinsicLeft = projectionLeft( cv::Rect(0,0,3,3) );
  cv::Mat projectionRight = cv::Mat( cameraRight.projectionMatrix() );
  cv::Matx33d intrinsicRight = projectionRight( cv::Rect(0,0,3,3) );

  assert(intrinsicLeft == intrinsicRight);

  const cv::Matx33d& intrinsic = intrinsicLeft;

  // Save the baseline
  stereo_baseline_ = stereoCameraModel.baseline();
  ROS_INFO_STREAM("baseline: " << stereo_baseline_);
  assert( stereo_baseline_ > 0 );

  // get the Region Of Interes (If the images are already rectified but invalid pixels appear)
  left_roi_ = cameraLeft.rawRoi();
  right_roi_ = cameraRight.rawRoi();

  cameraParametersLeft_ = std::make_unique<CameraParameters>(intrinsic, left_roi_.width, left_roi_.height, frustum_near_plane_distance_, frustum_far_plane_distance_, stereo_baseline_);
  cameraParametersRight_ = std::make_unique<CameraParameters>(intrinsic, right_roi_.width, right_roi_.height, frustum_near_plane_distance_, frustum_far_plane_distance_, stereo_baseline_);
}

void sptam::SPTAMInterface::publishMap()
{
  if ( mapPub_.getNumSubscribers() < 1 )
    return;
    
  boost::shared_lock<boost::shared_mutex> lock(sptam_->GetMap().map_mutex_);

  // Create PointCloud message for visualization
  pcl::PointCloud<pcl::PointXYZRGB> msg;
  msg.header.frame_id = map_frame_;
  msg.height = 1;
  msg.width = sptam_->GetMap().getMapPoints().size();
  for(const auto& mapPoint : sptam_->GetMap().getMapPoints() )
  {
    // Get Point from Map
    const Eigen::Vector3d point3d = mapPoint->GetPosition();
    pcl::PointXYZRGB point_pcl;

    point_pcl.x = point3d.x();
    point_pcl.y = point3d.y();
    point_pcl.z = point3d.z();

    cv::Vec3b color = mapPoint->getColor();

    point_pcl.r = color(0);
    point_pcl.g = color(1);
    point_pcl.b = color(2);

    msg.points.push_back ( point_pcl );
  }

  // Publish the PointCloud
  mapPub_.publish( msg );
}

void sptam::SPTAMInterface::publishPose(const uint32_t seq, const ros::Time& time, const CameraPose& currentCameraPose)
{
  tf2::Transform base_to_map;
  if ( not getBaseLinkPose( currentCameraPose, time, base_to_map ) )
  {
    ROS_WARN("Failed to retrieve camera pose in base_link frame. Ignoring pose publication.");
    return;
  }

  // TODO this is the covariance of the camera frame, shouldn't
  // it be translated to the base link frame?
  const Eigen::Matrix6d& covariance = currentCameraPose.covariance();

  #ifdef SHOW_PROFILING
    const tf2::Vector3& position = base_to_map.getOrigin();
    const tf2::Matrix3x3& orientation = base_to_map.getBasis();

    std::stringstream message;

    message << "BASE_LINK_POSE:" << " " << seq << " "
      << printFullPrecision( orientation[0][0] ) << " " << printFullPrecision( orientation[0][1] ) << " " << printFullPrecision( orientation[0][2] ) << " " << printFullPrecision( position.x() ) << " "
      << printFullPrecision( orientation[1][0] ) << " " << printFullPrecision( orientation[1][1] ) << " " << printFullPrecision( orientation[1][2] ) << " " << printFullPrecision( position.y() ) << " "
      << printFullPrecision( orientation[2][0] ) << " " << printFullPrecision( orientation[2][1] ) << " " << printFullPrecision( orientation[2][2] ) << " " << printFullPrecision( position.z() ) << " "
      << std::endl;

    message << "BASE_LINK_COVARIANCE:" << " " << seq << " " << printFullPrecision( time.toSec() ) << " ";
    forn(i, 6) forn(j, 6)
      message << " " << printFullPrecision( covariance(i, j) );
    message << std::endl;

    Logger::Write( message.str() );
  #endif

  //~ geometry_msgs::PoseStamped msg;
  geometry_msgs::PoseWithCovarianceStamped msg;

  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = map_frame_;

  msg.pose.pose.orientation.x = base_to_map.getRotation().x();
  msg.pose.pose.orientation.y = base_to_map.getRotation().y();
  msg.pose.pose.orientation.z = base_to_map.getRotation().z();
  msg.pose.pose.orientation.w = base_to_map.getRotation().w();

  msg.pose.pose.position.x = base_to_map.getOrigin().x();
  msg.pose.pose.position.y = base_to_map.getOrigin().y();
  msg.pose.pose.position.z = base_to_map.getOrigin().z();

  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  forn(i, 6) forn(j, 6)
  {
    msg.pose.covariance[6*i + j] = covariance(i, j);
    //~ if( covariance(i, j) != covariance(j, i) )
      //~ ROS_WARN_STREAM("MATRIX IS NOT SYMMETRIC for (" << i << "," << j << ") and (" << j << "," << i << "): " << covariance(i, j)-covariance(j, i));
  }

  //~ bool ret = isDefinitePositive( covariance );
  //~ ROS_INFO_STREAM("pose matrix defPos: " << ret);

  //~ Eigen::Matrix3d pos = covariance.block(0, 0, 3, 3);
  //~ ret = isDefinitePositive( pos );
  //~ ROS_INFO_STREAM("position matrix defPos: " << ret);

  //~ Eigen::Matrix3d ori = covariance.block(3, 3, 3, 3);
  //~ ret = isDefinitePositive( ori );
  //~ ROS_INFO_STREAM("orientation matrix defPos: " << ret);

  //~ ROS_INFO_STREAM("covariance: " << std::endl << covariance);

  // Publish the camera pose
  posePub_.publish( msg );
}

void sptam::SPTAMInterface::publishKFs(const uint32_t seq, const ros::Time& time)
{
  if ( keyframesPub_.getNumSubscribers() < 1 )
    return;
  
  boost::shared_lock<boost::shared_mutex> lock(sptam_->GetMap().map_mutex_);

  nav_msgs::Path keyframes_msg;
  keyframes_msg.header.seq = seq;
  keyframes_msg.header.stamp = time;
  keyframes_msg.header.frame_id = map_frame_;

  for(const auto& keyframe : sptam_->GetMap().getKeyframes()){
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.seq = seq;
    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = map_frame_;
    
    tf2::Transform kf_pose;
    CameraPose2TFPose( keyframe->GetCameraPose(), kf_pose );
    
    pose_msg.pose.orientation.x = kf_pose.getRotation().x();
    pose_msg.pose.orientation.y = kf_pose.getRotation().y();
    pose_msg.pose.orientation.z = kf_pose.getRotation().z();
    pose_msg.pose.orientation.w = kf_pose.getRotation().w();

    pose_msg.pose.position.x = kf_pose.getOrigin().x();
    pose_msg.pose.position.y = kf_pose.getOrigin().y();
    pose_msg.pose.position.z = kf_pose.getOrigin().z();

    keyframes_msg.poses.push_back(pose_msg);
  }

  // Publish the camera pose
  keyframesPub_.publish( keyframes_msg );
}

void sptam::SPTAMInterface::publishTransform()
{
  std::lock_guard<std::mutex> lock( odom_to_map_mutex_ );

  // TODO: remove the delay and parameters from here if we are sure that they are not needed
  ros::Time tf_expiration = ros::Time::now()/* + ros::Duration( tf_delay_ )*/;

  // if odometry is disabled, odom_to_map_ actually contains the map->base transform because I'm too lazy

  geometry_msgs::TransformStamped odom_to_map_msg;
  odom_to_map_msg.header.frame_id = map_frame_;
  odom_to_map_msg.header.stamp = tf_expiration;
  odom_to_map_msg.child_frame_id = published_frame_;

  odom_to_map_msg.transform = tf2::toMsg( odom_to_map_ );

  transform_broadcaster_.sendTransform(odom_to_map_msg);
}

void sptam::SPTAMInterface::publishTransformLoop()
{
  if ( transform_publish_freq_ == 0 )
    return;

  ros::Rate r( transform_publish_freq_ );

  while ( ros::ok() ) {
    publishTransform();
    r.sleep();
  }
}

void sptam::SPTAMInterface::drawStereoFrames(const TrackingReport &report)
{
  cv_bridge::CvImage cv_img;
  cv_img.encoding = "bgr8";

  if (stereoFrame_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = report.stereoFrameBeforeRefine;
    stereoFrame_Pub_.publish(cv_img.toImageMsg());
  }

  if (stereoFrameAfter_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = report.stereoFrameAfterRefine;
    stereoFrameAfter_Pub_.publish(cv_img.toImageMsg());
  }
}

