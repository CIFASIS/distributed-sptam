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
#pragma once
#include <ros/ros.h>

#include "MapMaker.hpp"
#include "utils/concurrent_queue.hpp"

#include <thread>

/**
 * @brief TODO
 */
class MapMakerThread : public MapMaker
{
  public:

    MapMakerThread(
      sptam::Map& map,
      const Eigen::Vector2d& focal_length,
      const Eigen::Vector2d& principal_point,
      const double stereo_baseline,
      const Parameters& params
    );

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Nuevo constructor que toma el publisher
    MapMakerThread(
      sptam::Map& map,
      const Eigen::Vector2d& focal_length,
      const Eigen::Vector2d& principal_point,
      const double stereo_baseline,
      const Parameters& params,
      ros::Publisher * pub_mapRefined
    );
//++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - Método para agregar KF a la cola (tomado de MapMakerThread::AddKeyFrame)
    inline void addKeyFrameForBA(const sptam::Map::SharedKeyFrame& keyFrame)
    {
    // push new keyFrame to queue
      keyFrameQueue_.push( keyFrame );
    } // keyFrameQueue_ es usada por MapMakerThread::Maintenance() para generar lastProcessedKeyFrames_
//++++++++++++++++++++++++++++++++++++++++++++++
    
    /**
     * @brief TODO
     */
    sptam::Map::SharedKeyFrame AddKeyFrame(const StereoFrame& frame, /*const */std::list<Match> &measurements) override;

    /**
     * Notify the mantainance loop to stop. This call blocks until
     * all pending keyFrames in the queue are properly adjusted.
     */
    void Stop();

    void MaintenanceThread(ros::Publisher * pub);

    /**
     * Pauses all Mapping activity, can be unpaused
     */
    void pause();
    void resume();
    bool isPauseRequested();
    bool isPaused();

    /*
     * Sincronization messages for establishing safe map window usage between LoopClosing and Mapping
     * TODO: Gaston: Sharing const references between threads doesnt ensures correct behaviour! */
    const sptam::Map::SharedKeyFrameSet& establishLocalSafeWindow();
    bool isSafeWindowEstablished();
    void freeSafeWindow();

  protected:

    /**
     * Since in this case, later keyframes already may have matched
     * some of the new_points, we need to check if that match already
     * has occured. If not, we would end up with duplicate measurements.
     */
    bool isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint) override;

  private:

  /*** Private properties ***/

    // Signal to stop the maintenance thread loop.
    bool stop_;

    mutable std::mutex pause_mutex_;
    bool pauseRequested_ = false;
    bool isPaused_ = false;

    // Thread continously running maintainance operations.
    std::thread maintenanceThread_;

    // Queue of new keyFrames waiting to be processed by maintainance.
    concurrent_queue<sptam::Map::SharedKeyFrame> keyFrameQueue_;

    // Keep a list of the last N inserted keyframes to do refind of new points.
    // TODO find a better way of selecting keyframes to do refind (covisibility graph?)
    std::list< sptam::Map::SharedKeyFrame > lastProcessedKeyFrames_; // Gaston: LoopClosure safe windows uses this member variable for sincronization

    /* Gaston: This mutex handles the usage and establishment of the safe usage windows between Mapping and LoopClosing
     * both writing and reading is managed with this unique mutex.
     * NOTE: To ensure that the safe window takes place at beginning of all Mapping procedures, it must be establish while Mapping is
     * in a safe stage of the code. SafeWindowRequested serves as a flag for that purpose. */
    mutable std::mutex safewindow_mutex_;
    sptam::Map::SharedKeyFrameSet safe_window_;
    bool isSafeWindowEstablish_ = false;
    bool safeWindowRequested_ = false;

    sptam::Map::SharedKeyFrameSet getSafeCovisibleKFs(sptam::Map::SharedKeyFrameSet& baseKFs) override;

  /*** Private functions ***/

    // Loop that continously runs maintainance operations.
    void Maintenance();
};
