#include "mapNode.hpp"

#include "../ros/parameters_opencv3.hpp"   // loadDescriptorMatcher

#include "utils/cv2eigen.hpp"
#define INITIAL_POINT_COVARIANCE Eigen::Matrix3d::Identity() * 1e-4     //to generate MapPoints

MapNode::MapNode(ros::NodeHandle& nh, ros::NodeHandle& nhp)
{
  //Set Publishers
  pub_mapRefined_ = nhp.advertise<sptam::msg_mapDiff>("msg_mapRefined",100);

  //Set Subscribers
  sub_mapDiff_ = nh.subscribe("tracker/msg_mapDiff", 100, &MapNode::callback_mapDiff, this);


// sptam_parameters
  // load descriptor matcher
  std::string matcherName;
  bool crossCheck;
  {
    nhp.param<std::string>("DescriptorMatcher/Name", matcherName, "BruteForce-Hamming");
    nhp.param<bool>("DescriptorMatcher/crossCheck", crossCheck, false);

    if (matcherName == "BruteForce-Hamming") {
      sptam_params_.descriptorMatcher = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING, crossCheck);
    }
    else if (matcherName == "BruteForce") {
      sptam_params_.descriptorMatcher = cv::makePtr<cv::BFMatcher>(cv::NORM_L2, crossCheck);
    }
    else if (matcherName == "BruteForce-L1") {
      sptam_params_.descriptorMatcher = cv::makePtr<cv::BFMatcher>(cv::NORM_L1, crossCheck);
    }
    else if (matcherName == "BruteForce-Hamming(2)") {
      sptam_params_.descriptorMatcher = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING2, crossCheck);
    }
    else throw std::runtime_error("Requested descriptor matcher algorithm not yet supported in factory method");
  }

  // Load Parameters
  int matchingCellSizeParam; 
  nhp.param<int>("MatchingCellSize", matchingCellSizeParam, 30);
  sptam_params_.matchingCellSize = matchingCellSizeParam;
  
  int matchingNeighborhoodParam;
  nhp.param<int>("MatchingNeighborhood", matchingNeighborhoodParam, 1);
  sptam_params_.matchingNeighborhoodThreshold = matchingNeighborhoodParam;
  
  nhp.param<double>("MatchingDistance", sptam_params_.matchingDistanceThreshold, 25.0);

  nhp.param<double>("EpipolarDistance", sptam_params_.epipolarDistanceThreshold, 0.0);

  // BA parameters
  int nKeyFramesToAdjustByLocal;
  nhp.param<int>("BundleAdjustmentActiveKeyframes", nKeyFramesToAdjustByLocal, 10);
  sptam_params_.nKeyFramesToAdjustByLocal = nKeyFramesToAdjustByLocal;

  int maxIterationsLocal;
  nhp.param<int>("maxIterationsLocal", maxIterationsLocal, 20);
  sptam_params_.maxIterationsLocal = maxIterationsLocal;



  #ifdef SHOW_PRINTS
    std::cout << "MapNode - MatchingCellSize: " << sptam_params_.matchingCellSize << std::endl; 
    std::cout << "MapNode - MatchingNeighborhood: " << sptam_params_.matchingNeighborhoodThreshold << std::endl; 
    std::cout << "MapNode - MatchingDistance: " << sptam_params_.matchingDistanceThreshold << std::endl; 
    std::cout << "MapNode - EpipolarDistance: " << sptam_params_.epipolarDistanceThreshold << std::endl; 
    std::cout << "MapNode - BundleAdjustmentActiveKeyframes: " << sptam_params_.nKeyFramesToAdjustByLocal << std::endl;
    std::cout << "MapNode - maxIterationsLocal: " << sptam_params_.maxIterationsLocal << std::endl;
  #endif

  #ifdef SHOW_PRINTS
    ROS_INFO("[MapNode: MapNode] - Node initialized.");
  #endif

}




void MapNode::get_params()
{
  // Getting CameraParameters    
  ros::param::get("/cameraParameters/baseline", baseline);

  //image width and height
  ros::param::get("/cameraParameters/left_width", left_img_width);
  ros::param::get("/cameraParameters/left_height", left_img_height);
  ros::param::get("/cameraParameters/right_width", right_img_width);       
  ros::param::get("/cameraParameters/right_height", right_img_height); 

  ros::param::get("FrustumNearPlaneDist", frustumNearPlaneDist);
  ros::param::get("FrustumFarPlaneDist", frustumFarPlaneDist);

  ros::param::get("/cameraParameters/intrinsic_vector", intrinsic_vector);
  
  #ifdef SHOW_PRINTS

    std::cout << "MapNode - /FeatureDetector/Name: " << detector_name << std::endl;
    std::cout << "MapNode - /DescriptorExtractor/Name: " << descriptor_name << std::endl; 
    std::cout << "MapNode - baseline: " << baseline << std::endl; 
    std::cout << "MapNode - image_size: " << cv::Size(img_width, img_height) << std::endl; 
    std::cout << "MapNode - frustumNearPlaneDist: " << frustumNearPlaneDist << std::endl;
    std::cout << "MapNode - FrustumFarPlaneDist: " << frustumFarPlaneDist << std::endl;

  #endif

  // Creo objetos a partir de los params 
  cv::Matx33d intrinsic(intrinsic_vector[0],intrinsic_vector[1],intrinsic_vector[2],intrinsic_vector[3],intrinsic_vector[4],intrinsic_vector[5],intrinsic_vector[6],intrinsic_vector[7],intrinsic_vector[8]);

  calibrationRight = new CameraParameters(
                      intrinsic,                  //cv::Matx33d intrinsic;
                      (size_t)right_img_width,    //size_t imageWidth;
                      (size_t)right_img_height,   //size_t imageHeight;
                      frustumNearPlaneDist,       //double frustumNearPlaneDist;
                      frustumFarPlaneDist,        //double frustumFarPlaneDist;
                      baseline                    //double baseline
                    );

  calibrationLeft = new CameraParameters(
                      intrinsic,                  //cv::Matx33d intrinsic;
                      (size_t)left_img_width,     //size_t imageWidth;
                      (size_t)left_img_height,    //size_t imageHeight;
                      frustumNearPlaneDist,       //double frustumNearPlaneDist;
                      frustumFarPlaneDist,        //double frustumFarPlaneDist;
                      baseline                    //double baseline
                    );

  params_were_get_ = true;

  // Inicializo MapMakerThread
  mapMaker_ = new MapMakerThread( map_, calibrationLeft->focalLengths(), calibrationLeft->principalPoint(), baseline, sptam_params_ , &pub_mapRefined_ );

  
  #ifdef SHOW_PRINTS
    ROS_INFO("[MapNode] - Params were get");
  #endif
}




void MapNode::callback_mapDiff(const sptam::msg_mapDiff& mapDiff)
{
  #ifdef SHOW_PRINTS
    ROS_INFO("[MapNode] - Recibiendo MapDIFF");
  #endif
    
  if ( !params_were_get_)
  {
    get_params();
  }

  // KeyFrames creation and adding to map
  sptam::Map::SharedKeyFrame keyFrame;
  for (auto& msg_kf : mapDiff.KeyFramesToAddorUpdate) //Siempre es de tamaño 1 al agregar
  {
    
    { //Tomo el lock interno de la clase Map hasta cerrar este scope 
      boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ );

      keyFrame = map_.AddKeyFrame(StereoFrame(
                   dsptam::fromMsgCameraPose(msg_kf.cameraPose),                                                                        //const CameraPose& camPose, 
                   *calibrationLeft,                                                                                                    //const CameraParameters& calibrationLeft,
                   baseline,                                                                                                            //const double stereo_baseline, 
                   *calibrationRight,                                                                                                   //const CameraParameters& calibrationRight,
                   dsptam::fromMsgImageFeatures(msg_kf.features_left, MatchingCellSize, cv::Size(left_img_width, left_img_height)),     //const ImageFeatures& imageFeaturesLeft, 
                   dsptam::fromMsgImageFeatures(msg_kf.features_right, MatchingCellSize, cv::Size(right_img_width, right_img_height)),  //const ImageFeatures& imageFeaturesRight,
                   msg_kf.bFixed                                                                                                        //bool bFixed  
                   )
                );
    }

    // Set id to keyframe
    keyFrame->SetId(msg_kf.id_KeyFrame ); 

    // Store keyframe in translation
    id2keyframe.set(keyFrame->GetId(), keyFrame);

    #ifdef SHOW_PRINTS
      ROS_INFO("[MapNode: callback_mapDiff] - KeyFrame Generated and Added to Map");
      std::cout << "[MapNode: callback_mapDiff] - KF generado con id =" << keyFrame->GetId()  << std::endl; 
    #endif  
  } 


  // MapPoints creation and adding to map
  sptam::Map::SharedPoint mapPoint;
  for (auto& msg_mp : mapDiff.MapPointsToAddorUpdate)
  {
    { //Tomo el lock interno de la clase Map hasta cerrar este scope
      boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ );
      mapPoint = map_.AddMapPoint( dsptam::fromMsgMapPoint(msg_mp) );
    }  

    id2point.set(mapPoint->getMapPointId(), mapPoint);
  }


  // Measurements creation and adding to map
  for (auto& msg_meas : mapDiff.MeasurementsToAddorUpdate)
  {
    { //Tomo el lock interno de la clase Map hasta cerrar este scope
      boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ );

      map_.addMeasurement(
                      id2keyframe.get(msg_meas.id_kf), 
                      id2point.get(msg_meas.id_mp), 
                      dsptam::fromMsgMeasurement(
                                        msg_meas, 
                                        id2keyframe.get((dsptam::id)msg_meas.id_kf), 
                                        id2point.get((dsptam::id)msg_meas.id_mp) )
                      );
    }  
  }

  // push new keyFrame to queue and signal
  mapMaker_->addKeyFrameForBA( keyFrame );

}