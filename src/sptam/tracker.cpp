#include "tracker.hpp"

#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>

/////====================================================================================
/////=======================      start SPTAMInterface.cpp        =======================
/////====================================================================================


#define INITIAL_COVARIANCE ( Eigen::Matrix6d::Identity() * 1e-9 )
#define KEYFRAME_TRACKER_WINDOW_NAME "Tracker KeyFrame"

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



Tracker::Tracker(ros::NodeHandle& nh, ros::NodeHandle& nhp) 
  : motionModel_( new MotionModel(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), INITIAL_COVARIANCE) )
  , odom_to_map_( tf2::Transform::getIdentity() )
  , transform_thread_( nullptr )
  , transform_listener_( tfBuffer_ )
//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM -  
  // , sptam_( nullptr )                no hay instancia sptam_
  // , lastImageSeq_( 0 )               inicializado en .hpp
//++++++++++++++++++++++++++++++++++++++++++++++++
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
  // std::string detector_name;
  {
    nhp.param<std::string>("FeatureDetector/Name", detector_name, "GFTT");

    featureDetector_ = loadFeatureDetector( nhp, detector_name, "FeatureDetector" );
  }

  // Load descriptor extractor implementation from configuration.
  // std::string descriptor_name;
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

//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - inicializado en el constructor de MapMaker (uso new para que sea persistente)
   local_keyframe_window_ = new KeyFrameCache( nKeyFramesToAdjustByLocal );
//++++++++++++++++++++++++++++++++++++++++++++++++



  // Subscribe to images messages

  sub_img_l_.subscribe(nhp, "/stereo/left/image_rect", 1);
  sub_info_l_.subscribe(nhp, "/stereo/left/camera_info", 1);
  sub_img_r_.subscribe(nhp, "/stereo/right/image_rect", 1);
  sub_info_r_.subscribe(nhp, "/stereo/right/camera_info", 1);

//Sincronizo las img para obtener stereo
  if ( use_approx_sync )
  {
    approximate_sync_.reset( new ApproximateSync( ApproximatePolicy(10),
      sub_img_l_, sub_info_l_,
      sub_img_r_, sub_info_r_
    ) );

    approximate_sync_->registerCallback( boost::bind(
      &Tracker::callBack, this, _1, _2, _3, _4
    ) );
  }
  else
  {
    exact_sync_.reset( new ExactSync( ExactPolicy(1),
      sub_img_l_, sub_info_l_,
      sub_img_r_, sub_info_r_
    ) );

    exact_sync_->registerCallback( boost::bind(
      &Tracker::callBack, this, _1, _2, _3, _4
    ) );
  }

  mapPub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 100);
  posePub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot/pose", 100);
//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - no lo uso
  // keyframesPub_ = nhp.advertise<nav_msgs::Path>("keyframes", 100);
//++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - publisher para mapDiff  
    pub_msg_mapDiff_ = nhp.advertise<sptam::msg_mapDiff>("msg_mapDiff",1000);
//++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Set Subscriber 
    sub_mapRefined_ = nh.subscribe("mapNode/msg_mapRefined", 100, &Tracker::callback_mapRefined, this);
//++++++++++++++++++++++++++++++++++++++++++++++++




  stereoFrame_Pub_ = nhp.advertise<sensor_msgs::Image>("frame_before_refine", 100);
  stereoFrameAfter_Pub_ = nhp.advertise<sensor_msgs::Image>("frame_after_refine", 100);

  // The node will publish the transformation map_frame_ -> published_frame_
  // to tf. If using odometry this will be the odom_frame_ frame and if not, then base_frame_.
  published_frame_ = use_odometry_ ? odom_frame_ : base_frame_;

  // start transform publisher thread
  transform_thread_ = std::make_unique<std::thread>( boost::bind(&Tracker::publishTransformLoop, this) );

//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego idef 
  #ifdef SHOW_PRINTS
    ROS_INFO("[Tracker: Tracker] - Node initialized.");   //ORIGINAL: ROS_INFO("S-PTAM node initialized.");
  #endif

//++++++++++++++++++++++++++++++++++++++++++++++++
}




//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Sólo tomo esta funcionalidad del destructor 
Tracker::~Tracker(){

  // create map file
  {
    std::ofstream out("/tmp/map_cloud.dat");
    for ( const auto& point : map_.getMapPoints() )
      out << point->GetPosition().x() << " " << point->GetPosition().y() <<  " " << point->GetPosition().z() << std::endl;
    out.close();
  }
}

//++++++++++++++++++++++++++++++++++++++++++++++++







bool Tracker::getBaseLinkPose(const CameraPose& cameraPose, const ros::Time& t, tf2::Transform& base_to_map)
{
  tf2::Transform base_to_camera;
   if (not lookupTransformSafe(tfBuffer_, camera_frame_, base_frame_, t, base_to_camera))
     return false;


  tf2::Transform camera_to_map;
  CameraPose2TFPose( cameraPose, camera_to_map );

  base_to_map = camera_to_map * base_to_camera;

  return true;
}





void Tracker::fixOdomFrame(const CameraPose& cameraPose, const tf2::Transform& camera_to_odom, const ros::Time& t)
{
  tf2::Transform camera_to_map;
  CameraPose2TFPose( cameraPose, camera_to_map );

  // compute the new difference between map and odom
  tf2::Transform odom_to_map = camera_to_map * camera_to_odom.inverse();

  std::lock_guard<std::mutex> lock( odom_to_map_mutex_ );
  odom_to_map_ = odom_to_map;
}












void Tracker::callBack(
  const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
  const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info
)
{
//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego idef
  #ifdef SHOW_PROFILING
    Logger::Write("[Tracker: callBack] - Recibiendo img.. \n");
  #endif  
//++++++++++++++++++++++++++++++++++++++++++++++++


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

//++++++++++++++++++++++++++++++++++++++++++++++
//## DSPTAM - No hay instancia sptam_
/*
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
*/
//++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++
  //## DSPTAM - verifico si la camara fue calibrada anteriormente
   if( !cameraCalibration_was_set_ )
  {
    ROS_INFO_STREAM("init calib");
    // Extract camera parameters from cameraInfo messages.
    loadCameraCalibration(left_info, right_info);
  
    cameraCalibration_was_set_ = true; 
  }
//++++++++++++++++++++++++++++++++++++++++++++++

  
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
 
// Al crearse, ejecutan la fc Extract() que procesa las img
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

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - No hay instancia sptam_, chequeo con mapa interno
  // if the map was not initialized, try to build it

  if( map_.getMapPoints().size() == 0 ) // ORIGINAL:  if( not sptam_->isInitialized() )
  {
    ROS_INFO("Trying to intialize map...");

/* ORIGINAL:    
  if ( not sptam_->initFromStereo(cameraPose_, imageFeaturesLeft, imageFeaturesRight) )
      return;  */  
    initFromStereo(cameraPose_, imageFeaturesLeft, imageFeaturesRight);       
    
    std::cout << "Points initialized from stereo: " << map_.getMapPoints().size() << std::endl; //ORIGINAL:        std::cout << "Points initialized from stereo: " << sptam_->GetMap().getMapPoints().size() << std::endl;

    #ifdef SHOW_PRINTS
      // Prints initial keyFrame
      dsptam::print_keyFrame(*(map_.getKeyframes().front()), 1);
    #endif

//++++++++++++++++++++++++++++++++++++++++++++++
  }

  // if the map is already initialized, do tracking
  else
  {
    #ifdef SHOW_PROFILING
    WriteToLog("ESTIMATED_CAMERA_POSE:", currentSeq, cameraPose_.GetPosition(), cameraPose_.GetOrientationMatrix(), cameraPose_.covariance());
    #endif

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - No hay instancia sptam_

    TrackingReport tracking_report =  Tracker::track(   // ORIGINAL:    TrackingReport tracking_report = sptam_->track(
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

  // Creo msg a partir de listToMakeMSG_mapDiff   
  if (listToMakeMSG_mapDiff.size() > 0)       
    pub_msg_mapDiff_.publish(dsptam::createMsgMapDiff(listToMakeMSG_mapDiff)); 
  
 
 
  #ifdef SHOW_PRINTS
    std::cout << "[Tracker] - map_.getKeyframes->size(): " << map_.getKeyframes().size() << " - map_.getMapPoints->size(): " << map_.getMapPoints().size() << std::endl;
  #endif

  // Publish Map To be drawn by rviz visualizer
  publishMap();

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - No se usa esto 
  // publishKFs(currentSeq, currentTime);
//++++++++++++++++++++++++++++++++++++++++++++++

  // Publish the camera Pose
  publishPose( currentSeq, currentTime, cameraPose_ );
}





void Tracker::loadCameraCalibration(
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

//++++++++++++++++++++++++++++++++++++++++++++++
//## DSPTAM - Seteo los parametros
 
  //## Set public ParamServer 
  if ( !params_were_set_)
  {

    // ROS_INFO_STREAM("[Tracker: loadCameraCalibration] - Seteando Parámetros: ");
    ros::param::set("/cameraParameters/baseline", stereo_baseline_);

    // REVISAR: utilizar sharedpointers - Verificar si es necesario setear esto en el rosparam
    ros::param::set("/cameraParameters/left_horizontalFOV", (*cameraParametersLeft_).horizontalFov());
    ros::param::set("/cameraParameters/left_verticalFOV", (*cameraParametersLeft_).verticalFov());
    ros::param::set("/cameraParameters/right_horizontalFOV", (*cameraParametersRight_).horizontalFov());
    ros::param::set("/cameraParameters/right_verticalFOV", (*cameraParametersRight_).verticalFov());

    //image width and height
    ros::param::set("/cameraParameters/left_width", left_roi_.width);       
    ros::param::set("/cameraParameters/left_height", left_roi_.height);
    ros::param::set("/cameraParameters/right_width", right_roi_.width);       
    ros::param::set("/cameraParameters/right_height", right_roi_.height);     

    ros::param::set("FrustumNearPlaneDist", frustum_near_plane_distance_);
    ros::param::set("FrustumFarPlaneDist", frustum_far_plane_distance_);

    std::vector<double> intrinsic_vector;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        intrinsic_vector.push_back(intrinsic(i,j));
      }    
     
    }

    ros::param::set("/cameraParameters/intrinsic_vector", intrinsic_vector);

    #ifdef SHOW_PRINTS
      std::cout << "Tracker - /cameraParameters/baseline: " << stereo_baseline_ << std::endl; 
      std::cout << "Tracker - image_size: " << cv::Size(left_roi_.width, left_roi_.height) << std::endl;
      std::cout << "Tracker - frustumNearPlaneDist: " << frustum_near_plane_distance_ << std::endl; 
      std::cout << "Tracker - frustumFarPlaneDist: " << frustum_far_plane_distance_ << std::endl;     
      ROS_INFO_STREAM("[Tracker: loadCameraCalibration] - Camera Calibration was set...");
    #endif

    params_were_set_ = true;
  }
//++++++++++++++++++++++++++++++++++++++++++++++
}





void Tracker::publishMap()
{
  if ( mapPub_.getNumSubscribers() < 1 )
    return;
//++++++++++++++++++++++++++++++++++++++++++++++
//## DSPTAM - no hay instancia sptam_    
  // boost::shared_lock<boost::shared_mutex> lock(sptam_->GetMap().map_mutex_);

  // Create PointCloud message for visualization
  pcl::PointCloud<pcl::PointXYZRGB> msg;
  msg.header.frame_id = map_frame_;
  msg.height = 1;
  // ORIGINAL: msg.width = sptam_->GetMap().getMapPoints().size();
  msg.width = map_.getMapPoints().size();
  // ORIGINAL: for(const auto& mapPoint : sptam_->GetMap().getMapPoints() )
  for(const auto& mapPoint : map_.getMapPoints() )
//++++++++++++++++++++++++++++++++++++++++++++++
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












void Tracker::publishPose(const uint32_t seq, const ros::Time& time, const CameraPose& currentCameraPose)
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



void Tracker::publishTransform()
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

void Tracker::publishTransformLoop()
{
  if ( transform_publish_freq_ == 0 )
    return;

  ros::Rate r( transform_publish_freq_ );

  while ( ros::ok() ) {
    publishTransform();
    r.sleep();
  }
}


void Tracker::drawStereoFrames(const TrackingReport &report)
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




/////====================================================================================
/////============================  end SPTAMInterface.cpp   =======================
/////====================================================================================













/////====================================================================================
/////============================  start sptam.cpp   =======================
/////====================================================================================





bool Tracker::initFromStereo(const CameraPose& estimatedCameraPose, const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight)
{
//++++++++++++++++++++++++++++++++++++++++++++++
//## DSPTAM - desreferencio los punteros
  // Create Initial keyFrame
  StereoFrame frame(
    estimatedCameraPose,
    *cameraParametersLeft_,   //ORIGINAL: cameraParametersLeft_,
    stereo_baseline_,
    *cameraParametersRight_,  //ORIGINAL: cameraParametersRight_
    imageFeaturesLeft, imageFeaturesRight, true
  );

  frame.SetId( 0 ); // Set Keyframe ID

  std::vector<cv::Point3d> points;
  std::vector<cv::Point2d> featuresLeft, featuresRight;
  std::vector<cv::Mat> descriptorsLeft, descriptorsRight;

  frame.TriangulatePoints(
    *rowMatcher_, points,     //ORIGINAL: rowMatcher_, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );
//++++++++++++++++++++++++++++++++++++++++++++++

  // check that there are at least a minimum number of correct matches when there is no map
  if ( points.size() < 10 )
    return false;

  // Add Keyframe to the map
  sptam::Map::SharedKeyFrame keyFrame = map_.AddKeyFrame( frame );

//++++++++++++++++++++++++++++++++++++++++++++++
//## DSPTAM - no hay instancia mapMaker_
  addStereoPoints(            //ORIGINAL: mapMaker_.addStereoPoints(
    keyFrame, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );
//++++++++++++++++++++++++++++++++++++++++++++++

  lastKeyFrame_ = keyFrame;

  initialized_ = true;

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - prints
   #ifdef SHOW_PRINTS
    std::cout << " [Tracker::initFromStereo] - Cantidad de puntos creados por KeyFrame inicial:" << points.size() << std::endl;
    std::cout << " [Tracker::initFromStereo] - KeyFrame position:" << keyFrame->GetCameraPose() << std::endl;
   #endif

  // Guardo translation para keyframe
    id2keyframe.set(keyFrame->GetId(), keyFrame);
//++++++++++++++++++++++++++++++++++++++++++++++

  return true;
}











TrackingReport Tracker::track(
  const size_t frame_id, CameraPose estimatedCameraPose,
  const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight
#ifdef SHOW_TRACKED_FRAMES
  , const cv::Mat& imageLeft, const cv::Mat& imageRight
#endif
)
{
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - No se usa esto 
  // while(isPaused())
  //   std::this_thread::yield();

  // setTracking(true);
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef USE_LOOPCLOSURE
  {
    std::lock_guard<std::mutex> lock(pause_mutex_);
    if(not lc_T_.matrix().isIdentity()){ // The lc thread its updating a recent part of the map, we need to wait until finish
      #ifdef SHOW_PROFILING
      WriteToLog(" tk reloc pre: ", 0, 0, estimatedCameraPose.GetPosition(), estimatedCameraPose.GetOrientationMatrix());
      #endif

      Eigen::Isometry3d estimatedPose; // CameraPose to Isometry pose matrix
      estimatedPose.linear() = estimatedCameraPose.GetOrientationMatrix();
      estimatedPose.translation() = estimatedCameraPose.GetPosition();

      estimatedPose = estimatedPose * lc_T_; // applying loop correction

      /* re-setting estimated camera pose */
      Eigen::Quaterniond orientation(estimatedPose.linear());
      estimatedCameraPose = CameraPose(estimatedPose.translation(), orientation, Eigen::Matrix6d::Identity());

      lc_T_.setIdentity();

      if(posePredictor_ != nullptr)
        posePredictor_->resetPose(estimatedCameraPose.GetPosition(), estimatedCameraPose.GetOrientationQuaternion(), Eigen::Matrix6d::Identity());

      #ifdef SHOW_PROFILING
      WriteToLog(" tk reloc post: ", 0, 0, estimatedCameraPose.GetPosition(), estimatedCameraPose.GetOrientationMatrix());
      #endif
    }
  }
  #endif

//++++++++++++++++++++++++++++++++++++++++++++++
//## DSPTAM - desreferencio los punteros
  StereoFrame frame(
    estimatedCameraPose, *cameraParametersLeft_,      //ORIGINAL: estimatedCameraPose, cameraParametersLeft_,      
    stereo_baseline_, *cameraParametersRight_,      //ORIGINAL: stereo_baseline_, cameraParametersRight_,
    imageFeaturesLeft, imageFeaturesRight
  );
//++++++++++++++++++++++++++++++++++++++++++++++
  #ifdef SHOW_PROFILING
    double t_start, t_end;
    t_start = GetSeg();
  #endif

  sptam::Map::SharedMapPointList filtered_points;

  {
    /* Lock for reading a small segment of the Map, after getting the required points locking is no longer
       required due to internal locking of MapPoints */
    boost::shared_lock<boost::shared_mutex> lock(map_.map_mutex_);

    #ifdef SHOW_PROFILING
      t_end = GetSeg();
      WriteToLog(" tk lock_frustum: ", t_start, t_end);
    #endif

    filtered_points = filterPoints( frame );
  }

  // Try to match the features found in the new frame to the
  // 3D points saved in the map.
  #ifdef SHOW_PROFILING
    t_start = GetSeg();
  #endif

  // Here we will save the measured features in this frame,
  // to be passed to the tracker for pose refinement.
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - cambio params_ por sptam_params_
  std::list<Match> measurements = matchToPoints(
    frame, ListIterable<sptam::Map::SharedPoint>::from( filtered_points ),
    sptam_params_.descriptorMatcher, sptam_params_.matchingNeighborhoodThreshold,   // ORIGINAL:  params_.descriptorMatcher, params_.matchingNeighborhoodThreshold,
    sptam_params_.matchingDistanceThreshold, Measurement::SRC_TRACKER               // ORIGINAL: params_.matchingDistanceThreshold, Measurement::SRC_TRACKER
  );
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    t_end = GetSeg();
    WriteToLog(" tk find_matches: ", t_start, t_end);
    WriteToLog( " tk matched_feat_total: ", measurements.size() );
  #endif

  #ifdef SHOW_PROFILING
    t_start = GetSeg();
  #endif

  // Update the descriptor of the MapPoint with the new one.
  // It is not needed to ask a lock to the map to modify the descriptor of the mappoint (they have internal locking)
  for ( auto& match : measurements ){
      match.mapPoint->updateDescriptor( match.measurement.GetDescriptor() );
      match.mapPoint->IncreaseMeasurementCount(); // increase measurement counter of mapPoint
  }

  #ifdef SHOW_PROFILING
    t_end = GetSeg();
    WriteToLog(" tk update_descriptors: ", t_start, t_end);
  #endif

  TrackingReport report;

  #ifdef SHOW_TRACKED_FRAMES
  report.drawStereoFrame(frame, imageLeft, imageRight, filtered_points, measurements, sptam_params_, true);
  #endif // SHOW_TRACKED_FRAMES

  #ifdef SHOW_PROFILING
    t_start = GetSeg();
  #endif

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - en el código original, se inicializa tracker_ en el constructor del objeto SPTAM 
  // tracker_g2o tracker_;

tracker_g2o tracker_( cameraParametersLeft_->focalLengths(), cameraParametersLeft_->principalPoint(), stereo_baseline_ );
//++++++++++++++++++++++++++++++++++++++++++++++

  // The tracker will try to refine the new camera pose
  // from the computed measurements
  bool refine_ok = tracker_.RefineCameraPose(
                     estimatedCameraPose, report.refinedCameraPose, measurements);

  if (!refine_ok)
    report.state = TrackingReport::State::NOT_ENOUGH_POINTS;
  else
    report.state = TrackingReport::State::OK;

  #ifdef SHOW_PROFILING
    t_end = GetSeg();
    WriteToLog(" tk tracking_refine: ", t_start, t_end);
  #endif

  // Update frame pose
  frame.UpdateCameraPose( report.refinedCameraPose );

  frames_since_last_kf_++;

  #ifdef SHOW_TRACKED_FRAMES
    report.drawStereoFrame(frame, imageLeft, imageRight, filtered_points, measurements, sptam_params_, false);
  #endif // SHOW_TRACKED_FRAMES

  if( shouldBeKeyframe( frame, measurements ) )
  {


    frame.SetId( frame_id );

    #ifdef SHOW_PROFILING
      t_start = GetSeg();
    #endif

    //~ std::cout << " Adding key-frame with id " << frame.GetId() << std::endl;

    // The mapMaker takes ownership of the UniquePtr,
    // so after this call the variable should not be used.
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - no hay instancia mapMaker_ 
    sptam::Map::SharedKeyFrame keyFrame = AddKeyFrameWithMeasurements( frame, measurements );     // ORIGINAL: sptam::Map::SharedKeyFrame keyFrame = mapMaker_.AddKeyFrame( frame, measurements );
//++++++++++++++++++++++++++++++++++++++++++++++

    #ifdef SHOW_PROFILING
      t_end = GetSeg();
      WriteToLog(" tk add_keyframe: ", t_start, t_end);
    #endif

    lastKeyFrame_ = keyFrame;
    frames_since_last_kf_ = 0;

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - se agrega print
  #ifdef SHOW_PRINTS
    std::cout << "[Tracker::Track] - Agregando keyframe id = " << std::to_string(frame.GetId()) << std::endl;     // std::cout << "Adding key-frame." << std::endl;
  #endif    
//++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Guardo translation para keyframe
    id2keyframe.set(keyFrame->GetId(), keyFrame);
//---------------------------------------------- 
  }

  // hack to make profiling data more easy to process
  /*#ifdef SHOW_PROFILING
  else
  {
//    std::cout << "frames since last kf: " << frames_since_last_kf_ << std::endl;
    // Esto está para que haya tantos mensajes como frames
    WriteToLog(" tk triangulate_points: ", 0, 0);
    WriteToLog(" tk add_points: ", 0, 0);
    WriteToLog(" tk add_meas: ", 0, 0);
    WriteToLog(" tk lock_add_points: ", 0, 0);
    WriteToLog(" tk lock_add_meas: ", 0, 0);
    WriteToLog(" tk add_keyframe: ", 0, 0);
  }
  #endif*/

  /* Quick tracking state implementation
   * this way LoopClosing knows when its safe to correct trayectory */
  setTracking(false);
  return report;
}









sptam::Map::SharedMapPointList Tracker::filterPoints(const StereoFrame& frame)
{
  sptam::Map::SharedMapPointList filtered_points;

  #ifdef SHOW_PROFILING
    double t_start, t_end;
    t_start = GetSeg();
  #endif

  sptam::Map::SharedMapPointSet localMap = map_.getLocalMap( *lastKeyFrame_ );

//  std::list<sptam::Map::Point>& localMap = map_.GetMapPoints();

//  std::cout << "localMap size: " << localMap.size() << std::endl;


  for ( const sptam::Map::SharedPoint& mapPoint : localMap )
    if ( not mapPoint->IsBad() and frame.canView( *mapPoint ) )
    {
      mapPoint->IncreaseProjectionCount();
      filtered_points.push_back( mapPoint );
    }

  #ifdef SHOW_PROFILING
    t_end = GetSeg();
    WriteToLog(" tk frustum: ", t_start, t_end);
    WriteToLog(" tk visiblePoints: ", filtered_points.size());
  #endif

  return filtered_points;
}








bool Tracker::shouldBeKeyframe(const StereoFrame& frame, const std::list<Match> &measurements)
{
  /* dont queue keyframes if the mapper its paused (or about to be) */

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - no hay instancia mapMaker_
  // if(mapMaker_.isPaused() or mapMaker_.isPauseRequested()){
  //   #ifdef SHOW_PROFILING
  //   WriteToLog(" tk Mapper is paused: ", 1);
  //   #endif
  //   return false;
  // }
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    double t_start, t_end;
    t_start = GetSeg();
  #endif

  // las Heuristicas de agregado de keyframes es un trade-off entre velocidad y precision
  // Mientras mas esporadicamente se agregan los keyframes,
  // se tendra mas tiempo para ajustar el mapa, pero menos preciso sera el tracking

  // Based free coverage image Policy
//  return AddKeyFrameImageCoverPolicy(frame, 1241, 0.2);

  // Based Matched Features percentage Policy
//  return AddKeyFrameFeaturesPolicy(frame, 0.1);


  // Based Tracking features percentage wrt to reference keyframe Policy
  size_t numStereoMeas = measurements.size();
  bool shouldBeKeyframe = (AddKeyFrameTrackingFeaturesPolicy(frame, measurements, *lastKeyFrame_, 0.9)
                          or (numStereoMeas < 20)); // number of matches less than a threshold

  #ifdef SHOW_PROFILING
    t_end = GetSeg();
    WriteToLog(" tk keyframe_selection_strategy: ", t_start, t_end);
  #endif

  return shouldBeKeyframe;
}










void Tracker::setTracking(bool set){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isTracking_ = set;
}




/////====================================================================================
/////============================  end sptam.cpp   =======================
/////====================================================================================



/////====================================================================================
/////============================  start MapMaker.cpp   =======================
/////====================================================================================






//void MapMaker::addStereoPoints(
void Tracker::addStereoPoints(
  /*const */sptam::Map::SharedKeyFrame& keyFrame, std::vector<cv::Point3d>& points,
  std::vector<cv::Point2d>& featuresLeft, std::vector<cv::Mat>& descriptorsLeft,
  std::vector<cv::Point2d>& featuresRight, std::vector<cv::Mat>& descriptorsRight
)
{
  // TODO revisar si se puede refinar un poco el pedido de lock

  #ifdef SHOW_PROFILING
    double t_start = GetSeg();
  #endif
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - no es necesario
  // boost::unique_lock<boost::shared_mutex> lock(map_.map_mutex_);
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    double t_end = GetSeg();
    WriteToLog(" tk lock_add_points: ", t_start, t_end);
    t_start = GetSeg();
  #endif

  Eigen::Vector3d frame_position = keyFrame->GetPosition();
  forn ( i, points.size() )
  {

    Eigen::Vector3d normal = cv2eigen( points[ i ] ) - frame_position ;
    normal.normalize();
//++++++++++++++++++++++++++++++++++++++++++++++
//DSPTAM - Uso el nuevo constructor de MapPoint con id y agrego al vector de traducciones el puntero al MapPoint creado

    mapPoints_id_++;
    sptam::Map::SharedPoint mapPoint = map_.AddMapPoint( MapPoint( cv2eigen( points[ i ] ), normal, descriptorsLeft[ i ], INITIAL_POINT_COVARIANCE, mapPoints_id_ ) ); //ORIGINAL: sptam::Map::SharedPoint mapPoint = map_.AddMapPoint( MapPoint( cv2eigen( points[ i ] ), normal, descriptorsLeft[ i ], INITIAL_POINT_COVARIANCE ) );
   
    id2point.set(mapPoints_id_, mapPoint);
//++++++++++++++++++++++++++++++++++++++++++++++
    

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - genero un ptr a measurement, lo agrego al mapa y encolo para crear msg
    Measurement * measurement = new Measurement(Measurement::SRC_TRIANGULATION, featuresLeft[i], descriptorsLeft[i], featuresRight[i], descriptorsRight[i]);      // ORIGINAL:     Measurement measurement(Measurement::SRC_TRIANGULATION, featuresLeft[i], descriptorsLeft[i], featuresRight[i], descriptorsRight[i]);
    

    map_.addMeasurement( keyFrame, mapPoint, *measurement );      // ORIGINAL: map_.addMeasurement( keyFrame, mapPoint, measurement );
    
    // Agrego la info a la lista para crear el msg
    listToMakeMSG_mapDiff.push_back(std::make_tuple( 
                                            keyFrame,
                                            mapPoint, 
                                            measurement
                                    )); 

    // increase measurement counter in point
    mapPoint->IncreaseMeasurementCount();
  }

  #ifdef SHOW_PROFILING
    t_end = GetSeg();
    WriteToLog(" tk add_points: ", t_start, t_end);
  #endif
}






// void MapMaker::createNewPoints(/*const */sptam::Map::KeyFrame& keyFrame)
void Tracker::createNewPoints(/*const */sptam::Map::SharedKeyFrame& keyFrame)
{
  std::vector<cv::Point3d> points;
  std::vector<cv::Point2d> featuresLeft, featuresRight;
  std::vector<cv::Mat> descriptorsLeft, descriptorsRight;

  #ifdef SHOW_PROFILING
    double t_start = GetSeg();
  #endif
//++++++++++++++++++++++++++++++++++++++++++++++
//## DSPTAM - rowMatcher_ es std::unique_ptr<RowMatcher>, debo desreferenciarlo
  keyFrame->TriangulatePoints(
    *rowMatcher_, points,         // ORIGINAL: *rowMatcher_, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    double t_end = GetSeg();
    WriteToLog(" tk triangulate_points: ", t_start, t_end);
    WriteToLog(" tk created_new_points: ", points.size());
  #endif

  addStereoPoints(
    keyFrame, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );
}









/**
* @brief
*   add keyframe and measurements from track (SRC_TRACKER) to internal map
*
* @param frame
*   reference to current frame
* @param measurements
*
*   reference to measurements obtained from frame
*/
sptam::Map::SharedKeyFrame Tracker::AddKeyFrameWithMeasurements(const StereoFrame& frame, /*const */std::list<Match>& measurements)
{

  #ifdef SHOW_PROFILING
    double start_total = GetSeg();
  #endif
   
  sptam::Map::SharedKeyFrame keyFrame = map_.AddKeyFrame( frame );

  // Create new 3D points from unmatched features,
  // and save them in the local tracking map.
  createNewPoints( keyFrame );

  // Load matched map point measurements into the new keyframe.
  // the point could have expired in the meantime, so check it.
  for ( auto& match : measurements )
  {
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );
 
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - encolo para generar msg
  
    // Agrego la info a la lista para crear el msg
    listToMakeMSG_mapDiff.push_back( std::make_tuple( 
                                              keyFrame, 
                                              match.mapPoint, 
                                              new Measurement(match.measurement)
                                    ));
//++++++++++++++++++++++++++++++++++++++++++++++

  }

  /*#ifdef SHOW_PROFILING
    WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
    WriteToLog(" ba totalPoints: ", map_.nMapPoints());
  #endif*/

  // GENERAL MAINTAINANCE

  // Refine Newly made points (the ones added from stereo matches
  // when the last keyframe came in)

  #ifdef SHOW_PROFILING
    double start_refine = GetSeg();
  #endif

  std::list<sptam::Map::SharedPoint> aux_newpoints = getPointsCretaedBy( *keyFrame );

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - agrego print
  #ifdef SHOW_PRINTS
    std::cout << "     Cantidad de puntos creados por KeyFrame(" << keyFrame->GetId() << ") = " << aux_newpoints.size() << std::endl;
    std::cout << "     KeyFrame position:" << keyFrame->GetCameraPose() << std::endl;
  #endif
//++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM -  debo desreferenciar local_keyframe_window_ ya que lo defino como puntero
  /*int nFound = */ReFindNewlyMade( ListIterable<sptam::Map::SharedKeyFrame>::from( *local_keyframe_window_ ), ListIterable<sptam::Map::SharedPoint>::from( aux_newpoints ) );
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    double end_refine = GetSeg();
    WriteToLog(" ba refind_newly_made: ", start_refine, end_refine);
  #endif
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - lo defino como puntero, uso ->
  local_keyframe_window_->push( keyFrame );       // ORIGINAL: local_keyframe_window_.push( keyFrame )
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    double start_local = GetSeg();
  #endif
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - BA lo realiza MapNode
  // BundleAdjust( ListIterable<sptam::Map::SharedKeyFrame>::from( local_keyframe_window_ ) );
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    double end_local = GetSeg();
    WriteToLog(" ba local: ", start_local, end_local);
  #endif

  #ifdef USE_LOOPCLOSURE
  /* Notifying newly added keyframe to Loop Closure service */
  if(loopclosing_ != nullptr)
    loopclosing_->addKeyFrame(keyFrame);
  #endif

  // Remove bad points marked by BA
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM -
  // CleanupMap( ListIterable<sptam::Map::SharedKeyFrame>::from( local_keyframe_window_ ) );
//++++++++++++++++++++++++++++++++++++++++++++++

  #ifdef SHOW_PROFILING
    double end_total = GetSeg();
    WriteToLog(" ba totalba: ", start_total, end_total);
  #endif

  return keyFrame;

}




std::list<sptam::Map::SharedPoint> Tracker::getPointsCretaedBy(sptam::Map::KeyFrame& keyFrame)
{
  std::list<sptam::Map::SharedPoint> newMapPoints;

  for( const auto& measurement : keyFrame.measurements() )
    if ( measurement->GetSource() == Measurement::SRC_TRIANGULATION )
      newMapPoints.push_back( measurement->mapPoint() );

  return newMapPoints;
}


bool Tracker::isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint)
{
  return not mapPoint->IsBad() and keyFrame.canView( *mapPoint );
}

/**
 * Helper function. Filters points that could be
 * potential matches for a frame.
 */
std::list< sptam::Map::SharedPoint > Tracker::filterUnmatched(const sptam::Map::KeyFrame& keyFrame, Iterable<sptam::Map::SharedPoint>& mapPoints)
{
  std::list< sptam::Map::SharedPoint > filtered_points;

  for ( sptam::Map::SharedPoint& mapPoint : mapPoints )
    if ( isUnmatched( keyFrame, mapPoint ) )
      filtered_points.push_back( mapPoint );

  return filtered_points;
}

size_t Tracker::ReFindNewlyMade(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames, Iterable<sptam::Map::SharedPoint>&& new_points)
{
  // Is there new points?
  if( new_points.empty() )
    return 0;

  size_t nFound = 0;

  for( sptam::Map::SharedKeyFrame& keyFrame : keyFrames)
  {
    std::list< sptam::Map::SharedPoint > filtered_points = filterUnmatched(*keyFrame, new_points);

    for( sptam::Map::SharedPoint& mapPoint : filtered_points )
      mapPoint->IncreaseProjectionCount();

    std::list<Match> matches = matchToPoints(
      *keyFrame, ListIterable<sptam::Map::SharedPoint>::from( filtered_points )
      , sptam_params_.descriptorMatcher, sptam_params_.matchingNeighborhoodThreshold
      , sptam_params_.matchingDistanceThreshold, Measurement::SRC_REFIND
    );

    for (Match& match : matches) {
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );
      match.mapPoint->IncreaseMeasurementCount(); // increase measurement counter of mapPoint
    
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM -  enocolo para generar msg
      // Agrego la info a la lista para crear el msg
      listToMakeMSG_mapDiff.push_back( std::make_tuple( 
                                                keyFrame, 
                                                match.mapPoint, 
                                                new Measurement(match.measurement)
                                      ));
//++++++++++++++++++++++++++++++++++++++++++++++

    }

    nFound += matches.size();
  }

  return nFound;
}



/////====================================================================================
/////============================       end MapMaker.cpp          =======================
/////====================================================================================






/////====================================================================================
/////============================            DSPTAM               =======================
/////====================================================================================


void Tracker::callback_mapRefined(const sptam::msg_mapDiff& mapDiff)
{
  #ifdef SHOW_PRINTS
    ROS_INFO("[Tracker] - Recibiendo mapRefined");
  #endif

// Itero sobre el vector de KFs
  sptam::Map::SharedKeyFrame keyFrame;
  for (auto& msg_kf : mapDiff.KeyFramesToAddorUpdate)
  {
    // Obtengo ptr al objeto usando su id como index en Translations
    keyFrame = id2keyframe.get(msg_kf.id_KeyFrame);

    CameraPose newCameraPose = dsptam::fromMsgCameraPose(msg_kf.cameraPose);

    // std::cout << "[Tracker::callback_mapRefined] - Actualizando: KF id=" << keyFrame->GetId() << " - oldPositionCameraPose = " << "[" +  std::to_string(keyFrame->GetCameraPose().GetPosition().x()) + ", " +   std::to_string(keyFrame->GetCameraPose().GetPosition().y()) + ", " +   std::to_string(keyFrame->GetCameraPose().GetPosition().z()) +  "]" << " - newPosition = " << "[" +  std::to_string(newCameraPose.GetPosition().x()) + ", " +   std::to_string(newCameraPose.GetPosition().y()) + ", " +   std::to_string(newCameraPose.GetPosition().z()) +  "]" << std::endl;

    keyFrame->UpdateCameraPose(newCameraPose);  // REVISAR: podria hacer directamente keyFrame->UpdateCameraPose( dsptam::fromMsgCameraPose(msg_kf.cameraPose) );
  } 


// Itero sobre el vector de MPs
  sptam::Map::SharedPoint mapPoint;
  for (auto& msg_mp : mapDiff.MapPointsToAddorUpdate)
  {
    // Obtengo ptr al objeto usando su id como index en Translations
    mapPoint = id2point.get(msg_mp.id_MapPoint);

    // Genero un Eigen::Vector3d a partir de las coord de la position 
    Eigen::Vector3d newPosition;
    dsptam::fromMSG_Geometry_msgsPoint2EigenVector3d(msg_mp.position, newPosition);

    // Actualizo
    mapPoint->updatePosition(newPosition);
  }

// Itero sobre las measurements
  for (auto& p : mapDiff.MeasurementsToDelete){
    #ifdef SHOW_PRINTS
      std::cout << "[Tracker::callback_mapRefined] - eliminando meas entre: [KF,MP] = [" << p.idKF << ", " << p.idMP << "]" << std::endl;
    #endif
    map_.removeMeasurement( id2keyframe.get(p.idKF), id2point.get(p.idMP));
  }

}