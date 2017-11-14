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

#include "utils/messages_handler.hpp"                   // msgs functions and types

#include "MapMakerThread.hpp"
#include "utils/macros.hpp"

#ifdef SHOW_PROFILING
#include "../sptam/utils/log/Profiler.hpp"
#endif

MapMakerThread::MapMakerThread(
  sptam::Map& map,
  const Eigen::Vector2d& focal_length,
  const Eigen::Vector2d& principal_point,
  const double stereo_baseline,
  const Parameters& params
)
  : MapMaker( map, focal_length, principal_point, stereo_baseline, params )
  , stop_(false), maintenanceThread_(&MapMakerThread::Maintenance, this)
{}

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego constructor con ros::Publisher
 MapMakerThread::MapMakerThread(
  sptam::Map& map,
  const Eigen::Vector2d& focal_length,
  const Eigen::Vector2d& principal_point,
  const double stereo_baseline,
  const Parameters& params,
  ros::Publisher * pub_mapRefined
)
  : MapMaker( map, focal_length, principal_point, stereo_baseline, params )
  , stop_(false), maintenanceThread_(&MapMakerThread::MaintenanceThread, this, pub_mapRefined )
{}
//++++++++++++++++++++++++++++++++++++++++++++++


sptam::Map::SharedKeyFrame MapMakerThread::AddKeyFrame(const StereoFrame& frame, /*const */std::list<Match>& measurements)
{

  sptam::Map::SharedKeyFrame keyFrame;
  {
    boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ );
    keyFrame = map_.AddKeyFrame( frame );
  }

  // Create new 3D points from unmatched features,
  // and save them in the local tracking map.
  createNewPoints( keyFrame );

  // Load matched map point measurements into the new keyframe.
  {
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      double t_start, t_end;
      t_start = GetSeg();
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++

    boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ );

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      t_end = GetSeg();
      WriteToLog(" tk lock_add_meas: ", t_start, t_end);
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      t_start = GetSeg();
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++

    // the point could have expired in the meantime, so check it.
    for ( auto& match : measurements )
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      t_end = GetSeg();
      WriteToLog(" tk add_meas: ", t_start, t_end);
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++
  }

  /*#ifdef SHOW_PROFILING
    WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
    WriteToLog(" ba totalPoints: ", map_.nMapPoints());
  #endif*/

  // push new keyFrame to queue and signal
  keyFrameQueue_.push( keyFrame );

  // WARNING ojo que acá podría suceder que el mapper haga un push,
  // ya que esta funcion se ejecuta en el hilo del tracker.
  // En este caso, la llamada de abajo podría tener uno (o más)
  // frames de más.

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
  #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
    WriteToLog(" tk queueSize: ", keyFrameQueue_.size());
  #endif
//++++++++++++++++++++++++++++++++++++++++++++++

  return keyFrame;
}

void MapMakerThread::Stop()
{
  // Wait until Mapper empty the queue
  std::cout << "-- -- -- WAIT QUEUE - -- --" << std::endl;
  keyFrameQueue_.waitEmpty();

  stop_ = true;

  keyFrameQueue_.stop();

  std::cout << "-- -- -- WAIT JOIN -- -- --" << std::endl;
  maintenanceThread_.join();
}

/*
 * pause/resume methods requires sincronization with maintenaince thread.
 * To archive this sincronization, mantenaince loop needs to ensure that
 * hes at a "safe" portion of code
*/
void MapMakerThread::pause()
{
  {
    std::lock_guard<std::mutex> lock(pause_mutex_);
    pauseRequested_ = true;
  }

  while(!isPaused()){
    std::this_thread::yield();
  }
}

void MapMakerThread::resume()
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  pauseRequested_ = false;
  isPaused_ = false;
}

bool MapMakerThread::isPauseRequested()
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  return pauseRequested_;
}

bool MapMakerThread::isPaused()
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  return isPaused_;
}

void MapMakerThread::MaintenanceThread(ros::Publisher * pub){

  while ( not stop_ )
  {
    /* Pausing functionality must ensure that there isnt cached keyframes, otherwise
     * the loopclosing wont be updating the most recent version of Kfs */
    {
      std::lock_guard<std::mutex> lock(pause_mutex_);
      if(pauseRequested_ and keyFrameQueue_.empty()){
        isPaused_ = true;
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
        #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
        WriteToLog(" mm pauseRequested: ", pauseRequested_);
        #endif
//++++++++++++++++++++++++++++++++++++++++++++++
      }
    }

    while(isPaused()){
      if ( stop_ ) return;
      std::this_thread::yield();
    }

    {
      /* This is a safe portion of code where to define safe window*/
      std::lock_guard<std::mutex> lock(safewindow_mutex_);

      /* TODO: Map will be read if safeWindow is requested, lock is not being asked as we know that LC
       * is waiting for the window to finish their establishment */
      if(safeWindowRequested_){
        for (sptam::Map::SharedKeyFrame& keyFrame : lastProcessedKeyFrames_){
          safe_window_.insert( keyFrame );
          for( auto it : keyFrame->covisibilityKeyFrames() )
            safe_window_.insert( it.first );
        }

        isSafeWindowEstablish_ = true;
        safeWindowRequested_ = false;
      }
    }

    // Hack for avoiding that the waitAndPop blocks execution
    if(keyFrameQueue_.empty()){

      if ( stop_ ) return;

      std::this_thread::yield();
      continue;
    }

    // Select at most 5 new unprocessed keyFrames to refine.
    std::list< sptam::Map::SharedKeyFrame > keyFrames;
    {
      sptam::Map::SharedKeyFrame keyFrame;
      bool moreData = keyFrameQueue_.waitAndPop( keyFrame );
      keyFrames.push_back( keyFrame );

      while ( (keyFrames.size() < 5) and moreData )
      {
        moreData = keyFrameQueue_.waitAndPop( keyFrame );
        keyFrames.push_back( keyFrame );
      }
    }

    // An abrupt stop is necessary because the data aquired by
    // waitAndPop() in case of a shutdown may be garbage.
    if ( stop_ )
      return;

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      double start_total = GetSeg();
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++

    // Get New MapPoints triangulated by the keyFrames
    std::list<sptam::Map::SharedPoint> new_points;

    // Fill in new_points and update lastProcessedKeyFrames_.
    for ( auto keyFrame : keyFrames )
    {
      std::list<sptam::Map::SharedPoint> newKeyFramePoints = getPointsCretaedBy( *keyFrame );

      new_points.insert(new_points.end(), newKeyFramePoints.begin(), newKeyFramePoints.end());

      // update cache buffer
      lastProcessedKeyFrames_.push_back( keyFrame );

      // maintain cache buffer size
      if ( params_.nKeyFramesToAdjustByLocal < lastProcessedKeyFrames_.size() )
        lastProcessedKeyFrames_.pop_front();
    }

    /*#if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      //std::cout << "Se agrega/n " << keyframes.size() << " KeyFrame al mapa." << std::endl;

      WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
      WriteToLog(" ba totalPoints: ", map_.nMapPoints());
    #endif*/

    // general maintenance

    // Refine Newly made points (the ones added from stereo matches
    // when the last keyframe came in)
    {
      /* We need the write map lock, ReFindNewlyMade add new measurements to keyframes in the map and modifies those refinded keyframes.
       * NOTE: Gaston: We could be changing between write and read locks when its necesary but
       * this way its ensure that the loop closure thread wont get in the way. */
      boost::unique_lock< boost::shared_mutex > lock(map_.map_mutex_);
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
      #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
        double start_refine = GetSeg();
      #endif
//++++++++++++++++++++++++++++++++++++++++++++++

      /*int nFound = */ReFindNewlyMade( ListIterable<sptam::Map::SharedKeyFrame>::from( lastProcessedKeyFrames_ ), ListIterable<sptam::Map::SharedPoint>::from( new_points ) );

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
      #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
        double end_refine = GetSeg();
        WriteToLog(" ba refind_newly_made: ", start_refine, end_refine);
      #endif
//++++++++++++++++++++++++++++++++++++++++++++++
    }

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      double start_local = GetSeg();
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++

    // No lock is required: lastProcessedKeyFrames_ is an internal variable and it is not modified by other thread
    // No lock is required: The keyframes have an internal lock for their modification
    bool completed = BundleAdjust( ListIterable<sptam::Map::SharedKeyFrame>::from( lastProcessedKeyFrames_ ) );


//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - crear msj a partir de lastProcessedKeyFrames_ y obtener sus MP 
    std::list< sptam::Map::SharedPoint > mp_list;
    
    for (sptam::Map::SharedKeyFrame& keyFrame : lastProcessedKeyFrames_)
      for ( sptam::Map::SharedPoint mp : getPointsCretaedBy(*keyFrame) ) 
        mp_list.push_back(mp);      

      pub->publish(dsptam::createMsgMapRefined(lastProcessedKeyFrames_, mp_list, idMeasToDelete));
//++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      double end_local = GetSeg();
      WriteToLog(" ba local: ", start_local, end_local);
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++

    #ifdef USE_LOOPCLOSURE
    /* Notifying newly added keyframe to Loop Closure service */
    if(loopclosing_ != nullptr)
      loopclosing_->addKeyFrames(keyFrames);
    #endif

    // If Mapper thread was intrrupted, don't do any cleanup.
    if ( not completed )
      continue;

    // Remove bad points marked by BA
//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - no realizo cleanup
    // CleanupMap( ListIterable<sptam::Map::SharedKeyFrame>::from( lastProcessedKeyFrames_ ) );
//++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Agrego condicion al idef
    #if defined(SHOW_PROFILING) && defined(SHOW_PROFILING_MAPER)
      double end_total = GetSeg();
      WriteToLog(" ba totalba: ", start_total, end_total);
    #endif
//++++++++++++++++++++++++++++++++++++++++++++++
  }

}

void MapMakerThread::Maintenance()
{
  while ( not stop_ )
  {
    /* Pausing functionality must ensure that there isnt cached keyframes, otherwise
     * the loopclosing wont be updating the most recent version of Kfs */
    {
      std::lock_guard<std::mutex> lock(pause_mutex_);
      if(pauseRequested_ and keyFrameQueue_.empty()){
        isPaused_ = true;

        #ifdef SHOW_PROFILING
        WriteToLog(" mm pauseRequested: ", pauseRequested_);
        #endif
      }
    }

    while(isPaused()){
      if ( stop_ ) return;
      std::this_thread::yield();
    }

    {
      /* This is a safe portion of code where to define safe window*/
      std::lock_guard<std::mutex> lock(safewindow_mutex_);

      /* TODO: Map will be read if safeWindow is requested, lock is not being asked as we know that LC
       * is waiting for the window to finish their establishment */
      if(safeWindowRequested_){
        for (sptam::Map::SharedKeyFrame& keyFrame : lastProcessedKeyFrames_){
          safe_window_.insert( keyFrame );
          for( auto it : keyFrame->covisibilityKeyFrames() )
            safe_window_.insert( it.first );
        }

        isSafeWindowEstablish_ = true;
        safeWindowRequested_ = false;
      }
    }

    // Hack for avoiding that the waitAndPop blocks execution
    if(keyFrameQueue_.empty()){

      if ( stop_ ) return;

      std::this_thread::yield();
      continue;
    }

    // Select at most 5 new unprocessed keyFrames to refine.
    std::list< sptam::Map::SharedKeyFrame > keyFrames;
    {
      sptam::Map::SharedKeyFrame keyFrame;
      bool moreData = keyFrameQueue_.waitAndPop( keyFrame );
      keyFrames.push_back( keyFrame );

      while ( (keyFrames.size() < 5) and moreData )
      {
        moreData = keyFrameQueue_.waitAndPop( keyFrame );
        keyFrames.push_back( keyFrame );
      }
    }

    // An abrupt stop is necessary because the data aquired by
    // waitAndPop() in case of a shutdown may be garbage.
    if ( stop_ )
      return;


    #ifdef SHOW_PROFILING
      double start_total = GetSeg();
    #endif

    // Get New MapPoints triangulated by the keyFrames
    std::list<sptam::Map::SharedPoint> new_points;

    // Fill in new_points and update lastProcessedKeyFrames_.
    for ( auto keyFrame : keyFrames )
    {
      std::list<sptam::Map::SharedPoint> newKeyFramePoints = getPointsCretaedBy( *keyFrame );

      new_points.insert(new_points.end(), newKeyFramePoints.begin(), newKeyFramePoints.end());

      // update cache buffer
      lastProcessedKeyFrames_.push_back( keyFrame );

      // maintain cache buffer size
      if ( params_.nKeyFramesToAdjustByLocal < lastProcessedKeyFrames_.size() )
        lastProcessedKeyFrames_.pop_front();
    }

    /*#ifdef SHOW_PROFILING
      //std::cout << "Se agrega/n " << keyframes.size() << " KeyFrame al mapa." << std::endl;

      WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
      WriteToLog(" ba totalPoints: ", map_.nMapPoints());
    #endif*/

    // general maintenance

    // Refine Newly made points (the ones added from stereo matches
    // when the last keyframe came in)
    {
      /* We need the write map lock, ReFindNewlyMade add new measurements to keyframes in the map and modifies those refinded keyframes.
       * NOTE: Gaston: We could be changing between write and read locks when its necesary but
       * this way its ensure that the loop closure thread wont get in the way. */
      boost::unique_lock< boost::shared_mutex > lock(map_.map_mutex_);

      #ifdef SHOW_PROFILING
        double start_refine = GetSeg();
      #endif

      /*int nFound = */ReFindNewlyMade( ListIterable<sptam::Map::SharedKeyFrame>::from( lastProcessedKeyFrames_ ), ListIterable<sptam::Map::SharedPoint>::from( new_points ) );


      #ifdef SHOW_PROFILING
        double end_refine = GetSeg();
        WriteToLog(" ba refind_newly_made: ", start_refine, end_refine);
      #endif
    }


    #ifdef SHOW_PROFILING
      double start_local = GetSeg();
    #endif

    // No lock is required: lastProcessedKeyFrames_ is an internal variable and it is not modified by other thread
    // No lock is required: The keyframes have an internal lock for their modification
    bool completed = BundleAdjust( ListIterable<sptam::Map::SharedKeyFrame>::from( lastProcessedKeyFrames_ ) );

    #ifdef SHOW_PROFILING
      double end_local = GetSeg();
      WriteToLog(" ba local: ", start_local, end_local);
    #endif

    #ifdef USE_LOOPCLOSURE
    /* Notifying newly added keyframe to Loop Closure service */
    if(loopclosing_ != nullptr)
      loopclosing_->addKeyFrames(keyFrames);
    #endif

    // If Mapper thread was intrrupted, don't do any cleanup.
    if ( not completed )
      continue;

    // Remove bad points marked by BA
    CleanupMap( ListIterable<sptam::Map::SharedKeyFrame>::from( lastProcessedKeyFrames_ ) );

    #ifdef SHOW_PROFILING
      double end_total = GetSeg();
      WriteToLog(" ba totalba: ", start_total, end_total);
    #endif
  }
}

sptam::Map::SharedKeyFrameSet MapMakerThread::getSafeCovisibleKFs(sptam::Map::SharedKeyFrameSet& baseKFs)
{
  std::lock_guard<std::mutex> lock(safewindow_mutex_);

  /* This is a safe portion of code where to define safe window*/
  if(safeWindowRequested_){
    for (const sptam::Map::SharedKeyFrame& keyFrame : lastProcessedKeyFrames_){
      safe_window_.insert( keyFrame );
      for(auto& it : keyFrame->covisibilityKeyFrames())
          safe_window_.insert( it.first );
    }
    isSafeWindowEstablish_ = true;
    safeWindowRequested_ = false;
  }

  sptam::Map::SharedKeyFrameSet covisibleKFs;

  for (const sptam::Map::SharedKeyFrame& keyFrame : baseKFs)
    for(auto& it : keyFrame->covisibilityKeyFrames())
    {
      const sptam::Map::SharedKeyFrame& covisible_keyframe = it.first;

      if(!baseKFs.count( covisible_keyframe )) {
        if(isSafeWindowEstablish_) {
          if(safe_window_.count( covisible_keyframe ) == 1)
            covisibleKFs.insert( covisible_keyframe );
        }else
          covisibleKFs.insert( covisible_keyframe );
       }
    }

  return covisibleKFs;
}

const sptam::Map::SharedKeyFrameSet& MapMakerThread::establishLocalSafeWindow()
{
  {
    // Requesting the establishment of a safe window
    std::lock_guard<std::mutex> lock(safewindow_mutex_);

    isSafeWindowEstablish_ = false;
    safeWindowRequested_ = true;

    safe_window_.clear();
  }

  while(!isSafeWindowEstablished())
    std::this_thread::yield();

  return safe_window_;
}

void MapMakerThread::freeSafeWindow()
{
  std::lock_guard<std::mutex> lock(safewindow_mutex_);

  isSafeWindowEstablish_ = false;
  safeWindowRequested_ = false;
  safe_window_.clear();
}

bool MapMakerThread::isSafeWindowEstablished()
{
  std::lock_guard<std::mutex> lock(safewindow_mutex_);
  return isSafeWindowEstablish_;
}

bool hasMeasurement(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint)
{
  for (const auto& meas : keyFrame.measurements() )
    if (meas->mapPoint().get() == mapPoint.get())
      return true;

  return false;
}

bool MapMakerThread::isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint)
{
  return not hasMeasurement(keyFrame, mapPoint) and MapMaker::isUnmatched(keyFrame, mapPoint);
}
