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

#include "sptam.hpp"
#include "Match.hpp"
#include "match_to_points.hpp"
#include "utils/macros.hpp"
#include "utils/cv2eigen.hpp"
#include "utils/projective_math.hpp"

#include "KeyFramePolicy.hpp"
#include "FeatureExtractorThread.hpp"
#include <boost/range/adaptor/indirected.hpp>

#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>

#ifdef SHOW_TRACKED_FRAMES
  #include "utils/draw/Draw.hpp"
  // window to draw tracking image
  #define KEYFRAME_TRACKER_WINDOW_NAME "Tracker KeyFrame"
#endif // SHOW_TRACKED_FRAMES

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

SPTAM::SPTAM(const CameraParameters& cameraParametersLeft,
  const CameraParameters& cameraParametersRight,
  const double stereo_baseline,
  const RowMatcher& rowMatcher,
  const Parameters& params
  )
  : mapMaker_( map_, cameraParametersLeft.focalLengths(), cameraParametersLeft.principalPoint(), stereo_baseline, params )
  , tracker_( cameraParametersLeft.focalLengths(), cameraParametersLeft.principalPoint(), stereo_baseline )
  , lastKeyFrame_( nullptr )
  , params_( params )
  , cameraParametersLeft_( cameraParametersLeft )
  , cameraParametersRight_( cameraParametersRight )
  , stereo_baseline_( stereo_baseline )
  , rowMatcher_( rowMatcher )
  , frames_since_last_kf_( 0 )
  , initialized_( false )
{}

#ifdef USE_LOOPCLOSURE
/* This constructor was introduced for optative Loop Detector definition
 * Even with the define setted, LC wont take effect unless a detector its passed */
SPTAM::SPTAM(const CameraParameters& cameraParametersLeft,
  const CameraParameters& cameraParametersRight,
  const double stereo_baseline,
  const RowMatcher& rowMatcher,
  const Parameters& params,
  const std::shared_ptr<PosePredictor> posePredictor,
  std::unique_ptr<LCDetector>& loop_detector
  )
  : mapMaker_( map_, cameraParametersLeft.focalLengths(), cameraParametersLeft.principalPoint(), stereo_baseline, params )
  , tracker_( cv2eigen( getFocalLength( cameraParametersLeft.intrinsic ) ), cv2eigen( getPrincipalPoint( cameraParametersLeft.intrinsic ) ), stereo_baseline )
  , lastKeyFrame_( nullptr )
  , posePredictor_(posePredictor)
  , params_( params )
  , cameraParametersLeft_( cameraParametersLeft )
  , cameraParametersRight_( cameraParametersRight )
  , stereo_baseline_( stereo_baseline )
  , rowMatcher_( rowMatcher )
  , frames_since_last_kf_( 0 )
  {
    if(loop_detector){
      loopclosing_.reset(new LoopClosing(*this, mapMaker_, map_, loop_detector, LoopClosing::Parameters()));
      mapMaker_.setLoopClosing(loopclosing_);
     }
  }
  #endif

bool SPTAM::initFromStereo(const CameraPose& estimatedCameraPose, const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight)
{
  // Create Initial keyFrame
  StereoFrame frame(
    estimatedCameraPose,
    cameraParametersLeft_,
    stereo_baseline_,
    cameraParametersRight_,
    imageFeaturesLeft, imageFeaturesRight, true
  );

  frame.SetId( 0 ); // Set Keyframe ID

  std::vector<cv::Point3d> points;
  std::vector<cv::Point2d> featuresLeft, featuresRight;
  std::vector<cv::Mat> descriptorsLeft, descriptorsRight;

  frame.TriangulatePoints(
    rowMatcher_, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );

  // check that there are at least a minimum number of correct matches when there is no map
  if ( points.size() < 10 )
    return false;

  // Add Keyframe to the map
  sptam::Map::SharedKeyFrame keyFrame = map_.AddKeyFrame( frame );

  mapMaker_.addStereoPoints(
    keyFrame, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );

  lastKeyFrame_ = keyFrame;

  initialized_ = true;

  return true;
}

TrackingReport SPTAM::track(
  const size_t frame_id, CameraPose estimatedCameraPose,
  const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight
#ifdef SHOW_TRACKED_FRAMES
  , const cv::Mat& imageLeft, const cv::Mat& imageRight
#endif
)
{
  while(isPaused())
    std::this_thread::yield();

  setTracking(true);

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

  StereoFrame frame(
    estimatedCameraPose, cameraParametersLeft_,
    stereo_baseline_, cameraParametersRight_,
    imageFeaturesLeft, imageFeaturesRight
  );

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
  std::list<Match> measurements = matchToPoints(
    frame, ListIterable<sptam::Map::SharedPoint>::from( filtered_points ),
    params_.descriptorMatcher, params_.matchingNeighborhoodThreshold,
    params_.matchingDistanceThreshold, Measurement::SRC_TRACKER
  );

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
  report.drawStereoFrame(frame, imageLeft, imageRight, filtered_points, measurements, params_, true);
  #endif // SHOW_TRACKED_FRAMES

  #ifdef SHOW_PROFILING
    t_start = GetSeg();
  #endif

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
  report.drawStereoFrame(frame, imageLeft, imageRight, filtered_points, measurements, params_, false);
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
    sptam::Map::SharedKeyFrame keyFrame = mapMaker_.AddKeyFrame( frame, measurements );

    #ifdef SHOW_PROFILING
      t_end = GetSeg();
      WriteToLog(" tk add_keyframe: ", t_start, t_end);
    #endif

    lastKeyFrame_ = keyFrame;
    frames_since_last_kf_ = 0;

//    std::cout << " Adding key-frame." << std::endl;
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

sptam::Map::SharedMapPointList SPTAM::filterPoints(const StereoFrame& frame)
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

bool SPTAM::shouldBeKeyframe(const StereoFrame& frame, const std::list<Match> &measurements)
{

  /* dont queue keyframes if the mapper its paused (or about to be) */
  if(mapMaker_.isPaused() or mapMaker_.isPauseRequested()){
    #ifdef SHOW_PROFILING
    WriteToLog(" tk Mapper is paused: ", 1);
    #endif
    return false;
  }

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

#ifdef USE_LOOPCLOSURE
void SPTAM::setLoopCorrection(const Eigen::Isometry3d& T)
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  lc_T_ = T; // 4x4 copy
}
#endif

/* NOTE: Pausing needs to be not blocking, because the tracker isnt actually a diferent thread. */
void SPTAM::pause(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isPaused_ = true;
}

void SPTAM::unPause(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isPaused_ = false;
}

bool SPTAM::isPaused(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  return isPaused_;
}

bool SPTAM::isTracking(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  return isTracking_;
}

void SPTAM::setTracking(bool set){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isTracking_ = set;
}
