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

#include <vector>
#include <opencv2/opencv.hpp>

class Measurement
{
  public:

    // Possible measurement type
    typedef enum { STEREO, LEFT, RIGHT } Type;

    // Possible measurement sources
    typedef enum { SRC_TRIANGULATION, SRC_TRACKER, SRC_REFIND } Source;

  public:

    Measurement(const Type& type, const Source& source, const cv::Point2d& feature, const cv::Mat& descriptor);

    Measurement(const Source& source, const cv::Point2d& featureLeft, const cv::Mat& descriptorLeft, const cv::Point2d& featureRight, const cv::Mat& descriptorRight);

    Measurement(const Type& type, const Source& source, const std::vector<double>& projection, const cv::Mat& descriptor);

    Measurement(const Measurement& measurement);

    // TODO OJO esto no es la proyeccion, es la posicion
    // del feature extraido en la imagen. Cambiar nombre!
    inline const std::vector<double>& GetProjection() const
    { return projection_; }

    inline const cv::Mat& GetDescriptor() const
    { return descriptor_; }

    inline const Type& GetType() const
    { return type_; }

    inline const Source& GetSource() const
    { return source_; }

  private:

    // Image feature position
    std::vector<double> projection_;

    // Image feature descriptor
    cv::Mat descriptor_;

    Type type_;

    Source source_;
};
