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

#include "MotionModel.hpp"

MotionModel::MotionModel(const Eigen::Vector3d& initialPosition, const Eigen::Quaterniond& initialOrientation, const Eigen::Matrix6d& initialCovariance)
  : position_( initialPosition ), orientation_( initialOrientation ), poseCovariance_( initialCovariance )
  , linearVelocity_( Eigen::Vector3d::Zero() ), angularVelocity_( Eigen::Quaterniond::Identity() )
{}

void MotionModel::currentPose(Eigen::Vector3d& currentPosition, Eigen::Quaterniond& currentOrientation, Eigen::Matrix6d& covariance) const
{
  currentPosition = position_;
  currentOrientation = orientation_;
  covariance = poseCovariance_;
}

void MotionModel::predictPose(const ros::Time& time, Eigen::Vector3d& predictedPosition, Eigen::Quaterniond& predictedOrientation, Eigen::Matrix6d& predictionCovariance)
{
  // Compute predicted position by integrating linear velocity

  predictedPosition = position_ + linearVelocity_;

  // Compute predicted orientation by integrating angular velocity

  predictedOrientation = orientation_ * angularVelocity_;
  predictedOrientation.normalize();

  predictionCovariance = poseCovariance_;
}

// update the camera state given a new camera pose
void MotionModel::updatePose(const ros::Time& time, const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation, const Eigen::Matrix6d& covariance)
{
  // Compute linear velocity
  Eigen::Vector3d newLinearVelocity( newPosition - position_ );
  // In order to be robust against fast camera movements linear velocity is smoothed over time
  newLinearVelocity = (newLinearVelocity + linearVelocity_) * 0.5;

  // compute rotation between q1 and q2: q2 * qInverse(q1);
  Eigen::Quaterniond newAngularVelocity = newOrientation * orientation_.inverse();

  // In order to be robust against fast camera movements angular velocity is smoothed over time
  newAngularVelocity = newAngularVelocity.slerp(0.5, angularVelocity_);

  newAngularVelocity.normalize();

  // Update the current state variables

  position_ = newPosition;
  orientation_ = newOrientation;
  poseCovariance_ = covariance;
  linearVelocity_ = newLinearVelocity;
  angularVelocity_ = newAngularVelocity;
}

void MotionModel::resetPose(const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation, const Eigen::Matrix6d& covariance)
{
  /* Resetting the pose, keeping linear and angular velocity */
  position_ = newPosition;
  orientation_ = newOrientation;
  poseCovariance_ = covariance;
}
