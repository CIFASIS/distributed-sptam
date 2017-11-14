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

#include "Measurement.hpp"

Measurement::Measurement(const Type& type, const Source& source, const cv::Point2d& feature, const cv::Mat& descriptor)
  : projection_({ feature.x, feature.y }), type_( type ), source_(source)
{
  // assert this is treated as a monocular measurement
  assert( type != Measurement::STEREO );

  descriptor.copyTo( descriptor_ );
}

Measurement::Measurement(const Source& source, const cv::Point2d& featureLeft, const cv::Mat& descriptorLeft, const cv::Point2d& featureRight, const cv::Mat& descriptorRight)
  : projection_({ featureLeft.x, featureLeft.y, featureRight.x }), type_( Measurement::STEREO ), source_(source)
{
  descriptorLeft.copyTo( descriptor_ );
}

Measurement::Measurement(const Type& type, const Source& source, const std::vector<double>& projection, const cv::Mat& descriptor)
  : projection_(projection), type_(type), source_(source)
{
  descriptor.copyTo(descriptor_);
}

Measurement::Measurement(const Measurement& measurement)
  : projection_(measurement.GetProjection()), type_(measurement.GetType()), source_(measurement.GetSource())
{
  measurement.GetDescriptor().copyTo(descriptor_);
}
