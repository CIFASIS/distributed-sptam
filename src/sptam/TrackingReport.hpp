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
 *           Matias Nitschẹ <mnitsche at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#pragma once

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
 #include <opencv2/features2d/features2d.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/features2d.hpp>
#endif

#include "CameraPose.hpp"
#include "Map.hpp"
#include "Match.hpp"
#include "sptamParameters.hpp"

/**
 * Holds information regarding the result of a tracking operation
 * @todo to be extended to other report types
 */
class TrackingReport
{
  public:
    TrackingReport(void);

    /**
     * Draws side-by-side images of cameras with measurements and projections overlaid
     * @param before_refine if true, feature detection grid and unmatched keypoints are also drawn
     */
    void drawStereoFrame(const StereoFrame& frame,
                         const cv::Mat& left_in, const cv::Mat& right_in,
                         const sptam::Map::SharedMapPointList& filtered_points,
                         const std::list<Match>& measurements,
                         const Parameters& params,
                         bool before_refine);

    cv::Mat stereoFrameBeforeRefine, stereoFrameAfterRefine;
    CameraPose refinedCameraPose;

    /**
     * Describes the possible outcomes of tracking
     */
    enum class State {
                       OK,                  /** tracking was succesful */
                       NOT_ENOUGH_POINTS    /** there were not enough points for tracking */
                     };
    State state;
};

