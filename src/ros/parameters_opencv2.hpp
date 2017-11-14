/**
 * This file is part of the ros-image-features package.
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
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Logger.hpp"
#endif


// Set Opencv Algorithm parameters from ROS parameter server
inline void setParameters( ros::NodeHandle& nh, cv::Ptr<cv::Algorithm>&& algorithm, const std::string& base_name )
{
  std::vector<cv::String> parameters;
  algorithm->getParams( parameters );

  for ( const auto& param : parameters )
  {
    const std::string param_url = base_name + "/" + param;

    if ( nh.hasParam( param_url ) )
    {
      int param_type = algorithm->paramType( param );

      switch ( param_type )
      {
        case cv::Param::INT:
        {
          int val;
          nh.getParam(param_url, val);
          algorithm->set(param, val);
          ROS_INFO_STREAM("  " << param << ": " << val);
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + std::to_string( val ) + "\n" );
          #endif
          break;
        }

        case cv::Param::BOOLEAN:
        {
          bool val;
          nh.getParam(param_url, val);
          algorithm->set(param, val);
          ROS_INFO_STREAM("  " << param << ": " << val);
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + std::to_string( val ) + "\n" );
          #endif
          break;
        }

        case cv::Param::REAL:
        {
          double val;
          nh.getParam(param_url, val);
          algorithm->set(param, val);
          ROS_INFO_STREAM("  " << param << ": " << val);
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + std::to_string( val ) + "\n" );
          #endif
          break;
        }

        case cv::Param::STRING:
        {
          std::string val;
          nh.getParam(param_url, val);
          algorithm->set(param, val);
          ROS_INFO_STREAM("  " << param << ": " << val);
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + val + "\n" );
          #endif
          break;
        }

        default:
          ROS_ERROR_STREAM( "unknown/unsupported parameter type for " << param );
          break;
      }
    }
  }
}


// Set Opencv Algorithm parameters from ROS parameter server
inline cv::Ptr<cv::FeatureDetector> loadFeatureDetector( ros::NodeHandle& nh, const std::string& detector_name, const std::string& base_name )
{
  cv::Ptr<cv::FeatureDetector> featureDetector = cv::FeatureDetector::create( detector_name );

  ROS_INFO_STREAM("detector: " << detector_name);


  #ifdef SHOW_PROFILING
    Logger::Write( "#   detector: " + detector_name + "\n" );
  #endif

  if ( not featureDetector )
    ROS_ERROR_STREAM("could not load feature detector with name " << detector_name);

  setParameters(nh, featureDetector, base_name);

  return featureDetector;
}

// Set Opencv Algorithm parameters from ROS parameter server
inline cv::Ptr<cv::DescriptorExtractor> loadDescriptorExtractor( ros::NodeHandle& nh, const std::string& descriptor_name, const std::string& base_name )
{
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::DescriptorExtractor::create( descriptor_name );

  ROS_INFO_STREAM("descriptor: " << descriptor_name);

  #ifdef SHOW_PROFILING
    Logger::Write( "#   descriptor: " + descriptor_name + "\n" );
  #endif

  if ( not descriptorExtractor )
    ROS_ERROR_STREAM("could not load descriptor extractor with name " << descriptor_name);

  setParameters(nh, descriptorExtractor, base_name);

  return descriptorExtractor;
}

// Set Opencv Algorithm parameters from ROS parameter server
inline cv::Ptr<cv::DescriptorMatcher> loadDescriptorMatcher( ros::NodeHandle& nh, const std::string& matcher_name, const std::string& base_name )
{
  cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = cv::DescriptorMatcher::create( matcher_name );

  ROS_INFO_STREAM("matcher: " << matcher_name);

  #ifdef SHOW_PROFILING
    Logger::Write( "#   matcher: " + matcher_name + "\n" );
  #endif

  if ( not descriptorMatcher )
    ROS_ERROR_STREAM("could not load descriptor matcher with name " << matcher_name);

  setParameters(nh, descriptorMatcher, base_name);

  return descriptorMatcher;
}





