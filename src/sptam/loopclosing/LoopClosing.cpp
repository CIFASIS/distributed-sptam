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

#include <list>
#include <tuple>

#include "../Map.hpp"
#include "../utils/Time.hpp"
#include "LoopClosing.hpp"
#include "PoseEstimator.hpp"
#include "SmoothEstimatePropagator.hpp"
#include "StereoMatcher.hpp"

#include "../MapMakerThread.hpp"
#include "../sptam.hpp"

#include "opencv2/features2d/features2d.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

// G2O
#include <g2o/config.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#ifdef SHOW_PROFILING
  #include "../utils/Profiler.hpp"
  #include "../utils/Logger.hpp"
#endif // SHOW_PROFILING

using namespace std;

LoopClosing::LoopClosing(SPTAM& tracker, MapMakerThread& mapper, sptam::Map& map, std::unique_ptr<LCDetector>& detector, const Parameters& params)
  : tracker_(tracker)
  , mapper_(mapper), map_(map)
  , loop_detector_(std::move(detector))
  , params_(params)
  , maintenanceThread_(&LoopClosing::maintenance, this)
{}

LoopClosing::~LoopClosing()
{}

void LoopClosing::addKeyFrame(sptam::Map::KeyFrame& keyframe)
{
  // push new keyframe to queue and signal
  keyFrameQueue_.push( &keyframe );

  #ifdef SHOW_PROFILING
    WriteToLog(" lc queueSize: ", keyFrameQueue_.size());
  #endif
}

void LoopClosing::addKeyFrames(const ConstIterable<sptam::Map::KeyFrame>& keyFrames)
{
  for(auto keyframe : keyFrames)
    keyFrameQueue_.push(&keyframe.get());

  #ifdef SHOW_PROFILING
    WriteToLog(" lc queueSize: ", keyFrameQueue_.size());
  #endif
}

void matchAndEstimate(const sptam::Map& map,
                      const sptam::Map::KeyFrame* query_keyframe,
                      const sptam::Map::KeyFrame* match_keyframe,
                      size_t& nMatches, size_t& inliers, cv::Matx44d& estimated_pose)
{
  // TODO: UNHACK matcher type and threhold!
  StereoMatcher stereo_matcher(25, cv::NORM_HAMMING);
  cv::BFMatcher matcher(cv::NORM_HAMMING, false); 
  double maxDistance = 25;
  
  vector<SDMatch> stereo_matches;
  
  cv::Mat descriptors1;
  cv::Mat descriptors2;
  cv::Mat descriptors3;
  cv::Mat descriptors4;
  
  vector<cv::KeyPoint> kps1;
  vector<cv::KeyPoint> kps2;
  vector<cv::KeyPoint> kps3;
  vector<cv::KeyPoint> kps4;
  
  {
    // copyng required data for better concurrency between tracking and lc
    boost::shared_lock<boost::shared_mutex> lock(map.map_mutex_);
    
    descriptors1 = query_keyframe->GetFrameLeft().GetFeatures().GetDescriptors();
    descriptors2 = query_keyframe->GetFrameRight().GetFeatures().GetDescriptors();
    descriptors3 = match_keyframe->GetFrameLeft().GetFeatures().GetDescriptors();
    descriptors4 = match_keyframe->GetFrameRight().GetFeatures().GetDescriptors();
    
    kps1 = query_keyframe->GetFrameLeft().GetFeatures().GetKeypoints();
    kps2 = query_keyframe->GetFrameRight().GetFeatures().GetKeypoints();
    kps3 = match_keyframe->GetFrameLeft().GetFeatures().GetKeypoints();
    kps4 = match_keyframe->GetFrameRight().GetFeatures().GetKeypoints();
  }
  
  StereoMatcher::match(matcher, maxDistance, 
                       kps1, descriptors1, kps2, descriptors2,
                       kps3, descriptors3, kps4, descriptors4, stereo_matches);

  /*cv::Mat outstereo;
  drawStereoMatches(*query_keyframe, *match_keyframe, stereo_matches, outstereo);
  cv::resize(outstereo, outstereo, cv::Size(), 0.45, 0.45); // resize to X%
  cv::namedWindow("Stereo matches"); cv::imshow("Stereo matches", outstereo);
  cv::waitKey(1);*/

  nMatches = stereo_matches.size();

  inliers = 0;

  #ifdef SHOW_PROFILING
  WriteToLog(" lc peMatches: ", nMatches);
  #endif

  // PoseEstimator needs at least 4 points for the minimal case.
  if(nMatches < 4)
    return;

  {
    // copyng required data for better concurrency between tracking and lc
    boost::shared_lock<boost::shared_mutex> lock(map.map_mutex_);
    
    PoseEstimator pe(PoseEstimator::PETYPE::CENTRAL, PoseEstimator::MINIMAL_ALGORITHM::KNEIP,
                     PoseEstimator::GENERIC_ALGORITHM::UPNP, true);

    double focal_length = match_keyframe->GetCameraLeft().GetCalibration().intrinsic(0,0);
    pe.setRansacPixelThreshold(1, focal_length);

    inliers = pe.estimatePose(*query_keyframe, *match_keyframe, stereo_matches, estimated_pose);
  }
}

g2o::VertexSE3* addVertex(g2o::SparseOptimizer& optimizer, const unsigned int& currentFrameIndex, const Eigen::Isometry3d& pose, bool fixed){
  g2o::VertexSE3* v_se3 = new g2o::VertexSE3;

  v_se3->setId(currentFrameIndex);
  v_se3->setEstimate(pose);
  v_se3->setFixed(fixed);

  optimizer.addVertex(v_se3);

  return v_se3;
}

void configPoseGraph(const list<sptam::Map::KeyFrame*>& keyframes, const sptam::Map::KeyFrame* queryKF, const sptam::Map::KeyFrame* matchKF,
                     const cv::Matx44d& match_pose, std::vector<std::tuple<size_t, size_t, Eigen::Isometry3d>>& loop_frames,
                     g2o::SparseOptimizer& optimizer)
{
  optimizer.setVerbose(false);
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  /* The uncertainty matrix: Represents the inverse covariance of the measurement, and thus is symmetric
   * and positive definite. */
  Eigen::Matrix<double,6,6> uncertainty = Eigen::Matrix<double,6,6>::Identity();

  g2o::VertexSE3* currentVertex = nullptr;
  g2o::VertexSE3* previousVertex = currentVertex;

  auto loops_it = loop_frames.begin();

  for(auto it = keyframes.begin(); it != keyframes.end(); it++){
    const sptam::Map::KeyFrame* keyframe = *it;

    Eigen::Isometry3d keyframe_pose = Eigen::Isometry3d::Identity();
    keyframe_pose.linear() = keyframe->GetCameraPose().GetOrientationMatrix();
    keyframe_pose.translation() = keyframe->GetCameraPose().GetPosition();

    if(it == keyframes.begin())
      currentVertex = addVertex(optimizer, keyframe->GetId(), keyframe_pose, true);
    else
      currentVertex = addVertex(optimizer, keyframe->GetId(), keyframe_pose, false);

    /* Adding edges between sucesives keyframes of the map */
    if(it != keyframes.begin()) {
      // Pose of current in previous's reference frame, t = Tpc (takes points in current's reference and returns it in previous's reference)
      Eigen::Isometry3d t = previousVertex->estimate().inverse() * currentVertex->estimate();
      g2o::EdgeSE3* e = new g2o::EdgeSE3;
      e->setVertex(0, previousVertex);
      e->setVertex(1, currentVertex);
      e->setMeasurement(t);
      e->information() = uncertainty;
      optimizer.addEdge(e);
     }

    /* Adding edges of past loops detected */
    if(loops_it != loop_frames.end() && (size_t) keyframe->GetId() == get<0>(*loops_it)){
      g2o::VertexSE3* pastLoopVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(get<1>(*loops_it)));
      g2o::EdgeSE3* e = new g2o::EdgeSE3;

      e->setVertex(0, currentVertex);
      e->setVertex(1, pastLoopVertex);

      //pastLoopVertex->setFixed(true);

      /* Relative transformation between the vertices involved on this loop detected */
      e->setMeasurement(get<2>(*loops_it));

      e->information() = uncertainty;
      optimizer.addEdge(e);

      loops_it++;
    }

    previousVertex = currentVertex; // G2O vertex update
  }

  g2o::VertexSE3* currVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(queryKF->GetId()));
  g2o::VertexSE3* loopVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(matchKF->GetId()));

  /* Relative transformation between current vertex and loop vertex using openGV estimation */
  Eigen::Matrix<double,4,4> Tcl;
  cv::cv2eigen(match_pose, Tcl); // Tcl = loopPose = Twl (converting loop vertex estimated pose to Eigen)

  /* Tcl = Twc^-1 * Twl = currentPose^-1 * loopPose = Tcw * Twl = Tcl */
  Tcl = currVertex->estimate().inverse() * Tcl;

  g2o::EdgeSE3* e = new g2o::EdgeSE3;

  e->setVertex(0, currVertex);
  e->setVertex(1, loopVertex);

  //loopVertex->setFixed(true);

  e->setMeasurement(Eigen::Isometry3d(Tcl));
  e->information() = uncertainty;
  optimizer.addEdge(e);

  loop_frames.push_back(make_tuple(queryKF->GetId(), matchKF->GetId(), Eigen::Isometry3d(Tcl)));

  #ifdef SHOW_PROFILING
  std::stringstream message;
  message << " lc RELATIVE_TRANSFORMATION:" << " " << queryKF->GetId() << " " << matchKF->GetId() << " "
    << printFullPrecision( Tcl(0,0) ) << " " << printFullPrecision( Tcl(0,1) ) << " " << printFullPrecision( Tcl(0,2) ) << " " << printFullPrecision( Tcl(0,3) ) << " "
    << printFullPrecision( Tcl(1,0) ) << " " << printFullPrecision( Tcl(1,1) ) << " " << printFullPrecision( Tcl(1,2) ) << " " << printFullPrecision( Tcl(1,3) ) << " "
    << printFullPrecision( Tcl(2,0) ) << " " << printFullPrecision( Tcl(2,1) ) << " " << printFullPrecision( Tcl(2,2) ) << " " << printFullPrecision( Tcl(2,3) ) << " "
    << std::endl;

  Logger::Write( message.str() );
  #endif
}

template<typename Iterator>
void updatePosesAndPoints(const Iterator begin, const Iterator end, const set<sptam::Map::KeyFrame*>& lba_safe_window, const g2o::SparseOptimizer& optimizer)
{
  if(begin == end /*or keyframes.size() <= lba_safe_window.size()*/) // All map was selected as for local BA
    return;

  #ifdef SHOW_PROFILING
  size_t updated_KFs = 0; // Note that safe_window ids KFs are not consecutive!! this is just for having an idea what is inside
  std::list<size_t> l;
  if(lba_safe_window.begin() != lba_safe_window.end())
  { l.push_back((*lba_safe_window.begin())->GetId()); l.push_back((*std::prev(lba_safe_window.end()))->GetId()); }
  WriteToLog(" lc KFsUpdateRange: ", l);
  #endif

  for (auto keyframe = begin; keyframe != end; keyframe++){

    if(lba_safe_window.count(*keyframe)) // avoid keyframes being use by the local BA
      continue;

    const g2o::VertexSE3* corrected_keyframe = static_cast<const g2o::VertexSE3*>(optimizer.vertex((*keyframe)->GetId()));

    /* non corrected Twi */
    Eigen::Isometry3d non_correctedTwi = Eigen::Isometry3d::Identity();
    non_correctedTwi.linear() = (*keyframe)->GetCameraPose().GetOrientationMatrix();
    non_correctedTwi.translation() = (*keyframe)->GetCameraPose().GetPosition();

    for(sptam::Map::Meas& meas : (*keyframe)->measurements()){
      if(meas.GetSource() == Measurement::SRC_TRIANGULATION){
        sptam::Map::Point& mp = meas.mapPoint();

        Eigen::Vector3d position = mp.GetPosition();

        Eigen::Vector4d hmg_position(position(0),position(1),position(2),1);

        /* corrected map point = corrTwi * non_corrTiw * position, this way we first interpret the point in
         * reference of the non corrected camera and then, as the relative direction of the point must be the same in
         * reference of the corrected camera, we transform the point in worlds reference using the corrected Twi */
        hmg_position = corrected_keyframe->estimate() * (non_correctedTwi.inverse() * hmg_position); // correcting map point position

        mp.updatePosition(Eigen::Vector3d(hmg_position(0)/hmg_position(3), hmg_position(1)/hmg_position(3), hmg_position(2)/hmg_position(3)));
      }
    }

    Eigen::Quaterniond orientation(corrected_keyframe->estimate().linear());
    (*keyframe)->UpdateCameraPose(CameraPose(corrected_keyframe->estimate().translation(), orientation, Eigen::Matrix6d::Identity()));

    #ifdef SHOW_PROFILING
    updated_KFs++;
    #endif
  }

  #ifdef SHOW_PROFILING
  WriteToLog(" lc KFsUpdated: ", updated_KFs);
  #endif
}

template<typename Iterator>
void updatePosesAndPoints(Iterator end, unsigned int size, const Eigen::Isometry3d& T)
{
  if(size == 0)
    return;

  #ifdef SHOW_PROFILING
  size_t updated_KFs = 0;
  /*std::list<size_t> l; l.push_back((*begin)->GetId()); l.push_back((*(end-1))->GetId());
  WriteToLog(" lc KFsUpdateRange: ", l);*/
  #endif

  for (auto keyframe = end; size > 0; keyframe--, size--){
    Eigen::Isometry3d correctedTwi = Eigen::Isometry3d::Identity();
    correctedTwi.linear() = keyframe->GetCameraPose().GetOrientationMatrix();
    correctedTwi.translation() = keyframe->GetCameraPose().GetPosition();

    correctedTwi = correctedTwi * T;

    Eigen::Isometry3d non_correctedTiw = Eigen::Isometry3d::Identity();
    non_correctedTiw.linear() = keyframe->GetCameraPose().GetOrientationMatrix();
    non_correctedTiw.translation() = keyframe->GetCameraPose().GetPosition();

    non_correctedTiw = non_correctedTiw.inverse();

    for(auto& meas : keyframe->measurements()){
      if(meas.GetSource() == Measurement::SRC_TRIANGULATION){
        sptam::Map::Point& mp = meas.mapPoint();

        Eigen::Vector3d position = mp.GetPosition();

        Eigen::Vector4d hmg_position(position(0),position(1),position(2),1);

        /* corrected map point = corrTwi * non_corrTiw * position, this way we first interpret the point in
         * reference of the non corrected camera and then, as the relative direction of the point must be the same in
         * reference of the corrected camera, we transform the point in worlds reference using the corrected Twi */
        hmg_position = correctedTwi * (non_correctedTiw * hmg_position); // correcting map point position

        mp.updatePosition(Eigen::Vector3d(hmg_position(0)/hmg_position(3), hmg_position(1)/hmg_position(3), hmg_position(2)/hmg_position(3)));
      }
    }

    Eigen::Quaterniond orientation(correctedTwi.linear());
    keyframe->UpdateCameraPose(CameraPose(correctedTwi.translation(), orientation, Eigen::Matrix6d::Identity()));

    #ifdef SHOW_PROFILING
    updated_KFs++;
    #endif
  }

  #ifdef SHOW_PROFILING
  WriteToLog(" lc KFsUpdateFix: ", updated_KFs);
  #endif
}

double time_last_lc = -1;

void LoopClosing::maintenance()
{

  /* Gaston: Hack to ensure that indices handled by detector correspond
   * with a vectors index for quick access.*/
  vector<sptam::Map::KeyFrame*> processed_keyframes;

  while ( not stop_ ) {
    sptam::Map::KeyFrame* keyframe;

    keyFrameQueue_.waitAndPop( keyframe );

    if(stop_) // keyframeQueue could've been awaken by a stop.
      return;

    /* TODO: Improve keyframes database for consistent and quick retrieval */
    processed_keyframes.push_back(keyframe);

    #ifdef SHOW_PROFILING
      double startStep, endStep;
    #endif

    DetectionMatch dm;
    {
      // the current keyframe was added to the map, so we need the read lock.
      boost::shared_lock<boost::shared_mutex> lock(map_.map_mutex_);
      #ifdef SHOW_PROFILING
        startStep = GetSeg();
      #endif

      dm = loop_detector_->detectloop(keyframe);

      #ifdef SHOW_PROFILING
        endStep = GetSeg();
        WriteToLog(" lc detection: ", startStep, endStep);
      #endif
    }

    /* Gaston: TODO: this is a HACK to avoid closing loops that are too old. While the map is corrected after a loop is validated,
     * several KFs are queued. If many large loops are being detected as valid in close sucession, those queued keyframes may stack
     * quite rapidly and the LoopClosure gets stuck checking really old KFs and loops. */
    if(not dm.detection() or GetSeg() - time_last_lc < 2){
      
      #ifdef SHOW_PROFILING
      if(dm.detection()){
        std::list<double> l; l.push_back(processed_keyframes[dm.query]->GetId()); l.push_back(processed_keyframes[dm.match]->GetId());
        l.push_back(keyFrameQueue_.size()); l.push_back(GetSeg() - time_last_lc);
        WriteToLog(" lc REJECTED_LOOP_TEMPORAL: ", l);
      }
      #endif
      
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }

    /* Mapping wont be paused at this stage, instead we will optimize the keyframes until this moment and
     * apply a rigid transformation for any other that gets createad afterwards */

    size_t nMatches;
    cv::Matx44d match_pose;
    size_t inliers;
    sptam::Map::KeyFrame* query_keyframe = processed_keyframes[dm.query];
    sptam::Map::KeyFrame* match_keyframe = processed_keyframes[dm.match];
    Eigen::Vector3d query_position;    

    {
      #ifdef SHOW_PROFILING
        startStep = GetSeg();
      #endif

      boost::shared_lock<boost::shared_mutex> lock(map_.map_mutex_);

      query_position = query_keyframe->GetPosition();
       
    }

    //mapper_.pauseCleanUp(); // TODO: LC doesnt support KFs removal now, but when it does we need to tell Mapping that doesnt do it now

    matchAndEstimate(map_, query_keyframe, match_keyframe, nMatches, inliers, match_pose);

    //mapper_.resumeCleanUp();

    #ifdef SHOW_PROFILING
      WriteToLog(" lc matching_pe: ", startStep, GetSeg());
    #endif

    if(not (inliers > 15 and (double) inliers >= nMatches * 0.8)){
      #ifdef SHOW_PROFILING
      std::list<size_t> l; l.push_back(query_keyframe->GetId()); l.push_back(match_keyframe->GetId());
      l.push_back(nMatches); l.push_back(inliers);
      WriteToLog(" lc REJECTED_LOOP: ", l);
      #endif
      continue;
    }

    if(std::abs(query_position(0) - match_pose(0,3)) > 5 or std::abs(query_position(1) - match_pose(1,3)) > 5 or std::abs(query_position(2) - match_pose(2,3)) > 5){
      #ifdef SHOW_PROFILING
      std::list<size_t> l; l.push_back(query_keyframe->GetId()); l.push_back(match_keyframe->GetId());
      l.push_back(nMatches); l.push_back(inliers);
      WriteToLog(" lc REJECTED_LOOP_TOOFAR: ", l);
      #endif
      continue;
    }

    #ifdef SHOW_PROFILING
    std::list<size_t> l; l.push_back(query_keyframe->GetId()); l.push_back(match_keyframe->GetId());
    l.push_back(nMatches); l.push_back(inliers);
    WriteToLog(" lc ACCEPTED_LOOP: ", l);
    #endif

    /* NOTE: Gaston: GraphVisibility measurements uses copy constructor of Measurement, which doesnt exist. How is being
     * copied the descriptor?? cv::Mat is just a pointer, default copy constructor doesnt really makes a copy
     * for(auto meas_match : stereo_matches)
      map_.addMeasurement(*query_keyframe, meas_match.second.get().mapPoint(),
                          Measurement(Measurement::STEREO, Measurement::SRC_REFIND,
                                      meas_match.first.get().GetProjection(), meas_match.first.get().GetDescriptor()));*/

    /* We have to ensure that the mapping thread is on a safe part of code, before the selection of KFs to optimize */
    //mapper_.pause();
    const set<sptam::Map::KeyFrame*>& safe_window = mapper_.establishLocalSafeWindow();
    //mapper_.resume();

    g2o::SparseOptimizer optimizer;

    /* We use the Rt of the keyframes that may be in use by the Local BA before the closure
     * to calculate the optimization that lba has introduce while we where optimizing the graph (very time consuming) */
    vector<Eigen::Isometry3d> lba_kfs_before_lc;

    list<sptam::Map::KeyFrame*> considered_keyframes;
    {
      boost::shared_lock<boost::shared_mutex> lock(map_.map_mutex_);

      /* HACK: This is awful! and must be handled by current lenght of kfs in the map. Considered KFs must have inside
       * the safe window established between mapping, this is one of the reason that KFs removal is not supported */
      {
        boost::shared_lock<boost::shared_mutex> lock(map_.map_mutex_);
        for(auto& kf : map_.GetKeyFrames())
          considered_keyframes.push_back(&kf);
      }

      // Loading keyframes poses and previous loops restritions
      configPoseGraph(considered_keyframes, query_keyframe, match_keyframe, match_pose, loop_frames_, optimizer);

      for (auto it = safe_window.begin(); it != safe_window.end(); it++){
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = (*it)->GetCameraPose().GetOrientationMatrix();
        pose.translation() = (*it)->GetCameraPose().GetPosition();

        lba_kfs_before_lc.push_back(pose.inverse()); // Copyng of each Tcw (rotation,translation) of the keyframes on the lba zone
      }
    }

    #ifdef SHOW_PROFILING
      startStep = GetSeg();
    #endif

    g2o::VertexSE3* loopVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(match_keyframe->GetId()));

    /* Note: Printing the whole pose graph its very time consuming, activate this debug output only when complete map
     * visualization its needed. */
    /*#ifdef SHOW_PROFILING
    std::stringstream ss;
    ss.str(""); ss << "./optimizations/LC_pre_" << query_keyframe->GetId() << "_" << match_keyframe->GetId() << ".g2o";
    optimizer.save(ss.str().c_str());
    #endif*/

    optimizer.initializeOptimization();

    // Propagate initial estimate through 10% of total keyframes (or at least 20 keyframes)
    double maxDistance = std::max(20.0, considered_keyframes.size() * 0.1);

    SmoothEstimatePropagator sEPropagator(&optimizer, maxDistance);

    sEPropagator.propagate(loopVertex);

    /*#ifdef SHOW_PROFILING
    ss.str(""); ss << "./optimizations/LC_prop_" << query_keyframe->GetId() << "_" << match_keyframe->GetId() << ".g2o";
    optimizer.save(ss.str().c_str());
    #endif*/

    optimizer.optimize( 20 );

    #ifdef SHOW_PROFILING
      endStep = GetSeg();
      WriteToLog(" lc optimization: ", startStep, endStep);
    #endif

    /*#ifdef SHOW_PROFILING
    ss.str(""); ss << "./optimizations/LC_optimized_" << query_keyframe->GetId() << "_" << match_keyframe->GetId() << ".g2o";
    optimizer.save(ss.str().c_str());
    #endif*/

    /* Updating keyframes and map points, we will first update those keyframes that arent being used by the mapper
     * Those keyframes are safe, no one are using them. Map points have internal locking, so there is no need for map lock */
    #ifdef SHOW_PROFILING
      startStep = GetSeg();
    #endif

    // Exclude KFs that may being use by the local BA.
    updatePosesAndPoints(considered_keyframes.begin(), considered_keyframes.end(), safe_window, optimizer);

    #ifdef SHOW_PROFILING
      endStep = GetSeg();
      WriteToLog(" lc unsafe_map_update: ", startStep, endStep);
      startStep = GetSeg();
    #endif
    
    /* Update those KFs that where created afterward */
    mapper_.pause();
    
    #ifdef SHOW_PROFILING
    WriteToLog(" lc mapping_pause: ", startStep, GetSeg());
    startStep = GetSeg();
    #endif
    
    tracker_.pause();
    // We need to wait for the end of the current frame tracking and ensure that we wont interfere with the tracker.
    while(tracker_.isTracking())
      std::this_thread::yield();
      
    #ifdef SHOW_PROFILING
    WriteToLog(" lc tracking_pause: ", startStep, GetSeg());
    #endif

    {
      // We ask for the lock as this keyframes are not safe and may be in use
      boost::unique_lock<boost::shared_mutex> lock(map_.map_mutex_);

      #ifdef SHOW_PROFILING
        startStep = GetSeg();
      #endif

      int i = 0;
      for(auto lba_keyframe : safe_window){

        Eigen::Isometry3d lba_kf_after_lc = Eigen::Isometry3d::Identity();
        lba_kf_after_lc.linear() = lba_keyframe->GetCameraPose().GetOrientationMatrix();
        lba_kf_after_lc.translation() = lba_keyframe->GetCameraPose().GetPosition();

        // Calculating inbetween optimization introduced by the LBA: Tcc' = Tcw * Twc'
        Eigen::Isometry3d lba_opt = lba_kfs_before_lc[i] * lba_kf_after_lc;

        g2o::VertexSE3* vertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(lba_keyframe->GetId()));

        // Correcting the g2o optimization carried with the adjusts introduced by the lba while we where busy.
        vertex->setEstimate(vertex->estimate() * lba_opt);

        i++;
      }

      /* Estimated error after loop closure: T = Tc1w * Twc2 = Tc1c2 with c1 the last frame before optimization and
       * c2 the last frame after optimization.
       * NOTE: We need to ensure that no other thread modifies this relative transformation! thats why we do it after locking */
      Eigen::Isometry3d non_correctedTwc = Eigen::Isometry3d::Identity();
      non_correctedTwc.linear() = (*(std::prev(considered_keyframes.end())))->GetCameraPose().GetOrientationMatrix();
      non_correctedTwc.translation() = (*(std::prev(considered_keyframes.end())))->GetCameraPose().GetPosition();

      // Corrected pose of the last optimized keyframe
      g2o::VertexSE3* last_vertex = static_cast<g2o::VertexSE3*>(optimizer.vertex((*(std::prev(considered_keyframes.end())))->GetId()));

      Eigen::Isometry3d T = Eigen::Isometry3d(non_correctedTwc).inverse() * last_vertex->estimate();

      tracker_.setLoopCorrection(T);

      // Updating keyframes and map points on the lba zone
      updatePosesAndPoints(safe_window.begin(), safe_window.end(), set<sptam::Map::KeyFrame*>(), optimizer);

      list<sptam::Map::KeyFrame>& after_lc_kfs = map_.GetKeyFrames();

      // Updating out of time keyframes created after the current loop closing procedure
      updatePosesAndPoints(std::prev(after_lc_kfs.end()), after_lc_kfs.size() - considered_keyframes.size(), T);

      #ifdef SHOW_PROFILING
        endStep = GetSeg();
        WriteToLog(" lc safe_map_update: ", startStep, endStep);
      #endif
      
      /*#ifdef SHOW_PROFILING
      g2o::SparseOptimizer optimizer2;
      list<sptam::Map::KeyFrame*> debug_keyframes;
      for(auto& kf : map_.GetKeyFrames())
        debug_keyframes.push_back(&kf);
      configPoseGraph(debug_keyframes, query_keyframe, match_keyframe, match_pose, loop_frames_, optimizer2);
      std::stringstream ss2;
      ss2.str(""); ss2 << "./optimizations/UPDATE_" << query_keyframe->GetId() << "_" << match_keyframe->GetId() << ".g2o";
      optimizer2.save(ss2.str().c_str());
      #endif*/
    }

    mapper_.freeSafeWindow();
    mapper_.resume();
    tracker_.unPause();

    time_last_lc = GetSeg();

    keyFrameQueue_.clear();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
