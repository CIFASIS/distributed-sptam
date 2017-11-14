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


#include "TrackingReport.hpp"
#include "utils/draw/Draw.hpp"

TrackingReport::TrackingReport(void)
{
  state = State::OK;
}

void TrackingReport::drawStereoFrame(const StereoFrame& frame, const cv::Mat& left_in, const cv::Mat& right_in,
                                     const sptam::Map::SharedMapPointList& filtered_points, const std::list<Match>& measurements, const Parameters& params, bool before_refine)
{
#ifdef SHOW_TRACKED_FRAMES
  cv::Mat left_out, right_out;
  if (before_refine)
    stereoFrameBeforeRefine = makeStereoWindow(left_in, right_in, left_out, right_out);
  else
    stereoFrameAfterRefine = makeStereoWindow(left_in, right_in, left_out, right_out);

  if (before_refine)
  {
    drawGrid(left_out, params.matchingCellSize);
    drawGrid(right_out, params.matchingCellSize);
  }

  drawProjections(left_out, frame.GetFrameLeft().GetProjection(), ConstSharedPtrListIterable<sptam::Map::Point>::from( filtered_points ));
  drawProjections(right_out, frame.GetFrameRight().GetProjection(), ConstSharedPtrListIterable<sptam::Map::Point>::from( filtered_points ));

  //~ drawProjectionCovariances(frame.GetFrameLeft(), outImageLeft, filtered_points);
  //~ drawProjectionCovariances(frame.GetFrameRight(), outImageRight, filtered_points);

  // extract and draw unmatched keypoints
  if (before_refine)
  {
    std::vector<cv::KeyPoint> keyPointsLeft, keyPointsRight;
    cv::Mat descriptorsLeft, descriptorsRight;
    std::vector<size_t> indexesLeft, indexesRight;
    frame.GetFrameLeft().GetUnmatchedKeyPoints(keyPointsLeft, descriptorsLeft, indexesLeft);
    frame.GetFrameRight().GetUnmatchedKeyPoints(keyPointsRight, descriptorsRight, indexesRight);

    cv::drawKeypoints(left_out, keyPointsLeft, left_out, COLOR_CYAN);
    cv::drawKeypoints(right_out, keyPointsRight, right_out, COLOR_CYAN);
  }

  drawMeasuredFeatures(frame, measurements, left_out, right_out);
#endif
}

