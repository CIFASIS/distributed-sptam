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

#include "../sptam/sptam.hpp"
#include "../sptam/MotionModel.hpp"

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#ifdef USE_LOOPCLOSURE
#include "../sptam/loopclosing/LCDetector.hpp"
#endif

namespace sptam
{

class SPTAMInterface
{
  public:

    SPTAMInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    ~SPTAMInterface();

  private:

    /**
     * @brief
     *   Syncronized image callback. Tracks the features on the new images
     *   to compute the current robot position.
     */
    void onImages(
      const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& info_msg_l,
      const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& info_msg_r
    );

    bool getBaseLinkPose(const CameraPose& cameraPose, const ros::Time& t, tf2::Transform& base_to_map);

    /**
     * @brief
     *   fix the error between map and odom to correct the current pose
     *   by publishing a corrected tf from map to odom frame.
     *
     * @param camera_pose
     *   current (refined) camera pose in the map reference frame
     */
    void fixOdomFrame(const CameraPose& cameraPose, const tf2::Transform& camera_to_odom, const ros::Time& t);

    /**
     * @brief compute stereo baseline, ROI's and FOV's from camera calibration messages.
     */
    void loadCameraCalibration( const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                const sensor_msgs::CameraInfoConstPtr& r_info_msg );

    void publishMap();
    void drawStereoFrames(const TrackingReport& report);
    void publishPose(const uint32_t seq, const ros::Time& time, const CameraPose& currentCameraPose);
    void publishKFs(const uint32_t seq, const ros::Time& time);

    void publishTransform();
    void publishTransformLoop();

  // Method variables

    std::unique_ptr<SPTAM> sptam_;

    cv::Ptr<cv::FeatureDetector> featureDetector_;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor_;

    std::unique_ptr<RowMatcher> rowMatcher_;

    Parameters sptam_params_;

    double frustum_near_plane_distance_, frustum_far_plane_distance_;

    std::unique_ptr<CameraParameters> cameraParametersLeft_;
    std::unique_ptr<CameraParameters> cameraParametersRight_;

    double stereo_baseline_;
    cv::Rect left_roi_, right_roi_;

    // used to save the last computed camera pose, in the map
    // coordinate frame, when odometry is disabled.
    CameraPose cameraPose_;
    std::shared_ptr<MotionModel> motionModel_;
    
    #ifdef USE_LOOPCLOSURE
    std::unique_ptr<LCDetector> loop_detector_; // Valid until sptam_ initialization
    #endif

  // node parameters

    bool use_odometry_;
    double tf_delay_, transform_publish_freq_, base_link_to_camera_timeout_;
    std::string odom_frame_, base_frame_, camera_frame_, map_frame_;

    // The node will publish the transformation map_frame_ -> published_frame_
    // to tf. If using odometry this will be the odom_frame_ frame and if not, then base_frame_.
    std::string published_frame_;

  // ROS variables

    ros::Publisher mapPub_, keyframesPub_, posePub_,
                   stereoFrame_Pub_, stereoFrameAfter_Pub_;
    size_t lastImageSeq_;

    /**
     * Keep a local cache of current map->odom transformation, because
     * this node only publishes it when a correction is necessary and
     * the time between those corrections may be too large for tf to
     * when trying to use the last published value.
     */
    tf2::Transform odom_to_map_;
    std::mutex odom_to_map_mutex_;

    // 
    std::unique_ptr<std::thread> transform_thread_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener transform_listener_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;

    // Subscribers for the input topics
    message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_l_, sub_info_r_;

    // syncronizer for image messages. We don't know a priory which one
    // will be used, so we have to define both, no common interface :/

    // Exact time image topic synchronizer
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;

    // Approximate time image topic synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync_;


}; // class sptam

} // namespace sptam
