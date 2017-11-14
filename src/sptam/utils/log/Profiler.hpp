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

#include <list>
#include <eigen3/Eigen/Geometry>
// lexical cast apparently automatically sets the precision for the float string.
#include <boost/lexical_cast.hpp>

#include "Logger.hpp"
#include "../Time.hpp"
#include "../macros.hpp"

inline std::string printFullPrecision( double x )
{
  return boost::lexical_cast<std::string>( x );
}

inline void WriteToLog( const std::string& tag, double start, double end )
{
  std::stringstream message;
  message << printFullPrecision( GetSeg() ) << tag << printFullPrecision( end - start ) << std::endl;

//  std::cout << message.str();

  Logger::Write( message.str() );
}

template<typename T>
inline void WriteToLog( const std::string& tag, const T& n )
{
  std::stringstream message;
  message << printFullPrecision( GetSeg() ) << tag << n << std::endl;

//  std::cout << message.str();

  Logger::Write( message.str() );
}

template<typename T>
inline void WriteToLog(const std::string tag, const std::list<T>& list )
{
  std::stringstream message;

  message << printFullPrecision( GetSeg() ) << tag;

  for ( const auto& elem : list )
    message << " " << elem;

  message << std::endl;

//  std::cout << message.str();

  Logger::Write( message.str() );
}

namespace Eigen { typedef Matrix<double, 6, 6> Matrix6d; }

inline void poseToStream(std::ostream& os, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const Eigen::Matrix6d& covariance)
{
  os
    << printFullPrecision( orientation(0,0) ) << " " << printFullPrecision( orientation(0,1) ) << " " << printFullPrecision( orientation(0,2) ) << " " << printFullPrecision( position.x() ) << " "
    << printFullPrecision( orientation(1,0) ) << " " << printFullPrecision( orientation(1,1) ) << " " << printFullPrecision( orientation(1,2) ) << " " << printFullPrecision( position.y() ) << " "
    << printFullPrecision( orientation(2,0) ) << " " << printFullPrecision( orientation(2,1) ) << " " << printFullPrecision( orientation(2,2) ) << " " << printFullPrecision( position.z() ) << " ";

  forn(i, 6) forn(j, 6)
    os << printFullPrecision( covariance(i,j) ) << " ";
}

inline void WriteToLog(const std::string msg, const size_t currentFrameIndex, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const Eigen::Matrix6d& covariance )
{
  std::stringstream message;

  message << msg << " " << currentFrameIndex << " ";
  poseToStream( message, position, orientation, covariance );
  message << std::endl;

//  std::cout << message.str();

  Logger::Write( message.str() );
}

inline void WriteToLog(const std::string msg, const size_t currentFrameIndex, const double timestamp, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const Eigen::Matrix6d& covariance )
{
  std::stringstream message;

  message << msg << " " << currentFrameIndex << " " << printFullPrecision( timestamp ) << " ";
  poseToStream( message, position, orientation, covariance );
  message << std::endl;

//  std::cout << message.str();

  Logger::Write( message.str() );
}
