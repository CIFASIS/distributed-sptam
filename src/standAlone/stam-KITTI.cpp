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

#include "SptamWrapper.hpp"
#include "KITTIGroundTruth.hpp"


#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
  #include "configuration_parser_opencv2.hpp"
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/highgui.hpp>
  #include "configuration_parser_opencv3.hpp"
#endif

#include "../sptam/utils/macros.hpp"
#include "../sptam/utils/Time.hpp"
#include "../sptam/utils/projective_math.hpp"

#include "Timestamps.hpp"
#include "utils/ProgramOptions.hpp"
#include "FrameGenerator/FrameGeneratorFactory.hpp"

#include "StereoImageFeatures.hpp"
#include "../sptam/FeatureExtractorThread.hpp"

#include "../sptam/MotionModel.hpp"

#include <signal.h>
#include <boost/chrono.hpp>

#include <boost/thread/thread.hpp>

#ifdef USE_LOOPCLOSURE
  #include "../sptam/loopclosing/LoopClosing.hpp"
  #include "../sptam/loopclosing/LCDetector.hpp"
  #include "../sptam/loopclosing/detectors/DLDBriefLoopDetector.hpp"
#endif

#ifdef SHOW_TRACKED_FRAMES
  #include "../sptam/utils/draw/Draw.hpp"
  #include <X11/Xlib.h> // XInitThreads()
#endif // SHOW_TRACKED_FRAMES

#ifdef SHOW_POINT_CLOUD
  #include "gui/PointCloud.hpp"
#endif // SHOW_POINT_CLOUD

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

#define INITIAL_POSE_COVARIANCE Eigen::Matrix6d::Identity() * 1e-6

// Used to smoothly finish the process in case of an interruption
bool programMustEnd = false;
// handler function called by interruptions signals
void interrupt_signal(int s) { programMustEnd = true; }

int main(int argc, char* argv[])
{

#if CV_MAJOR_VERSION == 2
  // Load Nonfree OpenCv modules (SURF,SIFT)
  cv::initModule_nonfree();
#endif // CV_MAJOR_VERSION

#ifdef SHOW_TRACKED_FRAMES
  // sirve para que todos los threads puedan mostrar imagenes
  // a traves de imshow()
  XInitThreads();
#endif // SHOW_TRACKED_FRAMES

#ifdef SHOW_PROFILING
  // Show doubles with presicion with cout
  std::cout << std::fixed;
#endif // shOW_PROFILING

  std::string leftImages, rightImages, parametersFileYML, calibration_file, timestampsFile, groundTruthPath, imagesSourceType;

  /** Define the program options */

  ProgramOptions program_options( argv[0] );
  program_options.addPositionalArgument("configuration", "configuration file with SPTAM parameters.", parametersFileYML);
  program_options.addPositionalArgument("calibration", "camera calibration file.", calibration_file);
  program_options.addPositionalArgument("left-images", "left camera source (image directory, streaming device or image list file).", leftImages);
  program_options.addPositionalArgument("right-images", "right camera source (image directory, streaming device or image list file).", rightImages);
  program_options.addPositionalArgument("images-source", "source type: 'dir', 'cam' or 'list'.", imagesSourceType);

  // TODO
  // add optional 'help'
  // add optional ini frame
  // add optional max frame
  program_options.addOptionalArgument("timestamps", "file containing the timestamps for each image frame.", timestampsFile);
  program_options.addOptionalArgument("grnd-poses", "Use ground truth poses file for pose prediction step.", groundTruthPath);
  program_options.addOptionalArgumentFlag("help", "show help.");

  /** Parse the program options */

  try
  {
    // may throw
    program_options.parse( argc, argv );

    // if count 'help' show help and exit
    if (program_options.count("help") ) {
      std::cerr << program_options << std::endl;
      return 0;
    }

    // throws on error, so do after help in case there are any problems.
    program_options.notify();
  } 
  catch(boost::program_options::error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << program_options << std::endl;
    return EXIT_FAILURE;
  }

  /** Load program parameters from configuration file */

  // Load S-PTAM parameters
  Parameters sptam_params;
  double nearPlaneDist, farPlaneDist;
  cv::Ptr<cv::FeatureDetector> featureDetector;
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;

  try
  {
    YAML::Node config = YAML::LoadFile( parametersFileYML );
    nearPlaneDist = config["FrustumNearPlaneDist"].as<double>();
    farPlaneDist = config["FrustumFarPlaneDist"].as<double>();

    sptam_params.matchingCellSize = config["MatchingCellSize"].as<size_t>();
    sptam_params.matchingNeighborhoodThreshold = config["MatchingNeighborhood"].as<size_t>();
    sptam_params.matchingDistanceThreshold = config["MatchingDistance"].as<double>();
    sptam_params.epipolarDistanceThreshold = config["EpipolarDistance"].as<size_t>();

    featureDetector = loadFeatureDetector( config["FeatureDetector"] );
    descriptorExtractor = loadDescriptorExtractor( config["DescriptorExtractor"] );
    sptam_params.descriptorMatcher = loadDescriptorMatcher( config["DescriptorMatcher"] );

    sptam_params.nKeyFramesToAdjustByLocal = config["BundleAdjustmentActiveKeyframes"].as<size_t>();
    sptam_params.maxIterationsLocal = 20;
  }
  catch(YAML::BadFile& e)
  {
    std::cerr << "Could not open configuration file " << parametersFileYML << std::endl;
    return EXIT_FAILURE;
  }
  catch(YAML::ParserException& e)
  {
    std::cerr << "Could not parse configuration file " << parametersFileYML << ". " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  #ifdef USE_LOOPCLOSURE
  if (config["LoopDetectorVocabulary"])
    sptam_params.loopDetectorVocabulary = config["LoopDetectorVocabulary"].as<std::string>();
  #endif

  // write configuration to log file
  #ifdef SHOW_PROFILING
    Logger::Write( "#   matchingCellSize: " + std::to_string( sptam_params.matchingCellSize ) + "\n" );
    Logger::Write( "#   matchingNeighborhoodThreshold: " + std::to_string( sptam_params.matchingNeighborhoodThreshold ) + "\n" );
    Logger::Write( "#   matchingDistanceThreshold: " + std::to_string( sptam_params.matchingDistanceThreshold ) + "\n" );
    Logger::Write( "#   epipolarDistanceThreshold: " + std::to_string( sptam_params.epipolarDistanceThreshold ) + "\n" );
    Logger::Write( "#   FrustumNearPlaneDist: " + std::to_string( nearPlaneDist ) + "\n" );
    Logger::Write( "#   FrustumFarPlaneDist: " + std::to_string( farPlaneDist ) + "\n" );
    Logger::Write( "#   BundleAdjustmentActiveKeyframes: " + std::to_string( sptam_params.nKeyFramesToAdjustByLocal ) + "\n" );
    #ifdef USE_LOOPCLOSURE
    Logger::Write( "#   LoopDetectorVocabulary: " + sptam_params.loopDetectorVocabulary + "\n" );
    #endif
  #endif

  // write sequence image source to log file
  #ifdef SHOW_PROFILING
    Logger::Write( "#   source_left_images: " + leftImages + "\n" );
    Logger::Write( "#   source_right_images: " + rightImages + "\n" );
  #endif

  RowMatcher rowMatcher(sptam_params.matchingDistanceThreshold, sptam_params.descriptorMatcher, sptam_params.epipolarDistanceThreshold);

  const CameraParameters cameraCalibration = loadCameraCalibration(calibration_file, nearPlaneDist, farPlaneDist);
  CameraParameters cameraParametersLeft = cameraCalibration;
  CameraParameters cameraParametersRight = cameraCalibration;

  /** Read source images or video stream */

  const size_t imageBeginIndex = 0;
  const size_t imageEndIndex = std::numeric_limits<size_t>::max(); // std::numeric_limits<size_t>::max() the whole sequence;

  IFrameGenerator* frameGeneratorLeft = createFrameGenerator(leftImages, imagesSourceType, imageBeginIndex, imageEndIndex);
  IFrameGenerator* frameGeneratorRight = createFrameGenerator(rightImages, imagesSourceType, imageBeginIndex, imageEndIndex);

  /** Subscribe interruption handlers */

  signal(SIGINT, &interrupt_signal);
  signal(SIGTERM, &interrupt_signal);

  /** Initialize pose prediction model */

  const bool useMotionModel = not program_options.count("grnd-poses");
  const bool useTimestamps = program_options.count("timestamps");

  std::shared_ptr<PosePredictor> motionModel(
    useMotionModel
    // Initialize Current Camera Pose With Left Canonical Position
    ? (PosePredictor*) new MotionModel( Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), INITIAL_POSE_COVARIANCE )
    : (PosePredictor*) new KITTIGroundTruth( groundTruthPath )
  );

  #ifdef USE_LOOPCLOSURE
  /* Brief detector its the only one implemented for the moment */
  std::unique_ptr<LCDetector> loop_detector(nullptr);
  if(config["DescriptorExtractor"]["Name"].as<std::string>().compare("BRIEF") == 0){
    DLDBriefLoopDetector::Parameters lcd_param; // Detector parameters by default
    std::cout << "Initializing Loop Detector, loading vocabulary" << std::endl;
    loop_detector.reset(new DLDBriefLoopDetector(sptam_params.loopDetectorVocabulary, lcd_param));
  }
  #endif

  // Get First Frames for use in the loop
  cv::Mat imageLeft, imageRight;
  bool hasNextFrame = frameGeneratorLeft->getNextFrame( imageLeft ) and frameGeneratorRight->getNextFrame( imageRight );

  // Create SPTAM wrapper
  SptamWrapper sptamWrapper ( cameraParametersLeft, cameraParametersRight, cameraCalibration.baseline(),
    rowMatcher, sptam_params, motionModel, imageBeginIndex
  #ifdef USE_LOOPCLOSURE
    , loop_detector
  #endif
  );

  CameraPose currentCameraPose;
  #ifdef SHOW_POINT_CLOUD
    const sptam::Map &map = sptamWrapper.GetMap();
    PointCloud pointCloud(currentCameraPose, map,
                          cameraParametersLeft.horizontalFOV, cameraParametersLeft.verticalFOV,
                          cameraParametersLeft.frustumNearPlaneDist, cameraParametersLeft.frustumFarPlaneDist);
  #endif

  // TODO hardcoded rate in case of no timestamp. Parametrize.
  Timestamps timestamps = useTimestamps ? Timestamps(timestampsFile, imageBeginIndex) : Timestamps(0.1, imageBeginIndex);

  size_t frame_number = imageBeginIndex;

  // Main loop
  while( hasNextFrame and not programMustEnd )
  {
    /** Compute time remainder so we don't go too fast, sleep if necessary */
    ros::Time current_time = timestamps.getNextWhenReady();

    #ifdef SHOW_PROFILING
      double start, end, startStep, endStep;
      start = GetSeg();
      startStep = start;
    #endif

    // Detect features and extract descriptors from new frames

    FeatureExtractorThread featureExtractorThreadLeft(imageLeft, *featureDetector, *descriptorExtractor);
    FeatureExtractorThread featureExtractorThreadRight(imageRight, *featureDetector, *descriptorExtractor);

    featureExtractorThreadLeft.WaitUntilFinished();
    const std::vector<cv::KeyPoint>& keyPointsLeft = featureExtractorThreadLeft.GetKeyPoints();
    const cv::Mat& descriptorsLeft = featureExtractorThreadLeft.GetDescriptors();

    featureExtractorThreadRight.WaitUntilFinished();
    const std::vector<cv::KeyPoint>& keyPointsRight = featureExtractorThreadRight.GetKeyPoints();
    const cv::Mat& descriptorsRight = featureExtractorThreadRight.GetDescriptors();

    ImageFeatures imageFeaturesLeft(imageLeft.size(), keyPointsLeft, descriptorsLeft, sptam_params.matchingCellSize);
    ImageFeatures imageFeaturesRight(imageRight.size(), keyPointsRight, descriptorsRight, sptam_params.matchingCellSize);

    std::unique_ptr<StereoImageFeatures> stereoImageFeatures(new StereoImageFeatures(imageFeaturesLeft,imageFeaturesRight, imageLeft, imageRight) );

    #ifdef SHOW_PROFILING
      endStep = GetSeg();
      WriteToLog(" tk extraction: ", startStep, endStep);
      WriteToLog(" tk ExtractedPoints: ", keyPointsLeft.size() + keyPointsRight.size());
    #endif

    sptamWrapper.Add(frame_number, current_time, std::move(stereoImageFeatures));

    currentCameraPose = sptamWrapper.GetCameraPose();

    #ifdef SHOW_PROFILING
      end = GetSeg();
      WriteToLog(" tk trackingtotal: ", start, end);
    #endif

    #ifdef SHOW_POINT_CLOUD

      pointCloud.SetCameraPose( currentCameraPose );

    #endif // SHOW_POINT_CLOUD

    // Get Next Frames
    hasNextFrame = frameGeneratorLeft->getNextFrame( imageLeft ) and frameGeneratorRight->getNextFrame( imageRight );
    frame_number++;
  }

  #ifdef SHOW_POINT_CLOUD
    // Create PLY file to be read by meshlab software
    CreatePLYFile( map );
  #endif // SHOW_POINT_CLOUD

  std::cout << "Wait for stop..." << std::endl;

  // Wait the mapper quits the main loop and joins.
  sptamWrapper.Stop();

  #ifdef SHOW_POINT_CLOUD
    // Stop visualizer Thread
    pointCloud.Stop();
  #endif

  /*#ifdef SHOW_PROFILING

    for ( const auto& mapPoint : sptamWrapper.GetMapPoints() ) {
      WriteToLog( " tk MeasurementCount: ", mapPoint.measurements().size() );
    }

    for ( const auto& keyFrame : sptamWrapper.GetKeyFrames() ) {
      CameraPose keyFramePose = keyFrame.GetCameraPose();
      WriteToLog("BASE_LINK_KF:", keyFrame.GetId(), keyFramePose.GetPosition(), keyFramePose.GetOrientationMatrix(), keyFramePose.covariance());
    }

    // dump map file
    std::stringstream ss;
    ss.str(""); ss << Logger::FileName() << "_map_cloud.dat";
    std::ofstream out(ss.str());
    for ( const auto& mapPoint : sptamWrapper.GetMapPoints() ) {
      out << mapPoint.GetPosition().x() << " " << mapPoint.GetPosition().y() <<  " " << mapPoint.GetPosition().z() << std::endl;
    out.close();

  #endif*/

  std::cout << "Stop succesfull!" << std::endl;

  #ifdef SHOW_TRACKED_FRAMES
  cv::destroyAllWindows();
  #endif

  return 0;
}
