//include msg
#include "utils/messages_handler.hpp"
#include "utils/translations.hpp"

#include <mutex>

#include <ros/ros.h>
#include "FeatureExtractorThread.hpp"
#include "ImageFeatures.hpp"
#include "sptamParameters.hpp"
#include "MotionModel.hpp"
#include "match_to_points.hpp"
#include "KeyFramePolicy.hpp"
#include "utils/fixed_queue.hpp"

#include "utils/draw/Draw.hpp"
  #define KEYFRAME_TRACKER_WINDOW_NAME "Tracker KeyFrame"

#include "MapPoint.hpp"
#include "Frame.hpp"
#include "Map.hpp"
#include "StereoFrame.hpp"
#include "Measurement.hpp"
#include "utils/CovisibilityGraph.hpp"
#include "utils/projective_math.hpp"
#include "utils/macros.hpp"
#include "utils/cv2eigen.hpp"
#include "tracker_g2o.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include "utils/tf_utils.hpp"

#include "PosePredictor.hpp"
#include "TrackingReport.hpp"

#define INITIAL_POINT_COVARIANCE Eigen::Matrix3d::Identity() * 1e-4

#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

//Messages
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include "CameraParameters.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sys/time.h>
#include <cv_bridge/cv_bridge.h>

#ifdef SHOW_PROFILING
    #include "../sptam/utils/log/Profiler.hpp"
    #include "../sptam/utils/log//Logger.hpp"
#endif // SHOW_PROFILING

#ifdef USE_LOOPCLOSURE
  #include "loopclosing/LoopClosing.hpp"
  #include "loopclosing/LCDetector.hpp"
#endif

#if CV_MAJOR_VERSION == 2
  #include "../ros/parameters_opencv2.hpp"
#elif CV_MAJOR_VERSION == 3
  #include "../ros/parameters_opencv3.hpp"
#endif

      
class Tracker
{
public:

/////====================================================================================
/////============================           DSPTAM                =======================
/////====================================================================================
    Tracker(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    // Get parameters via getPrivateNodeHandle(), 
    // Get topics via getNodeHandle().

    ~Tracker();

    sptam::Map::SharedKeyFrame AddKeyFrameWithMeasurements(const StereoFrame& frame, /*const */std::list<Match>& measurements);

    void callback_mapRefined(const sptam::msg_mapDiff& map);


/////====================================================================================
/////============================       SPTAMInterface.cpp        =======================
/////====================================================================================
    /**
     * @brief
     *   Syncronized image callback. Tracks the features on the new images
     *   to compute the current robot position.
     */
    void callBack(const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info);

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
    // void publishKFs(const uint32_t seq, const ros::Time& time);

    void publishTransform();     
    void publishTransformLoop();



/////====================================================================================
/////============================          sptam.cpp              =======================
/////====================================================================================

    bool initFromStereo(const CameraPose& estimatedCameraPose, const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight);

    // TODO remove all this getters and implement a dump to file method.

    /* TODO: Gaston: This getters serve as a proxy to the map object for the ros version,
     * this exposes const references to internal lists of the map. The ros node doesnt've access
     * to the map and reads this lists without any lock. This may produce undefined behaviour!. */
    inline bool isInitialized() const
    { return initialized_; }

     TrackingReport track(
      const size_t frame_id, CameraPose estimatedCameraPose,
      const ImageFeatures& imageFeaturesLeft,
      const ImageFeatures& imageFeaturesRight
      #ifdef SHOW_TRACKED_FRAMES
      , const cv::Mat& imageLeft, const cv::Mat& imageRight
      #endif
    );

    /**
     * @brief select the map points that are relevant to a frame.
     *   Map points are filtered by frustum culling and by the
     *   angle-of-view of the last descriptor.
     *   TODO: esto se le deberia poder pedir al mapa, y el mapa
     *   los debería devolver de manera eficiente si fuese posible.
     */
    sptam::Map::SharedMapPointList filterPoints(const StereoFrame& frame);


    bool shouldBeKeyframe(const StereoFrame& frame, const std::list<Match>& measurements);
/////====================================================================================
/////============================        MapMaker.cpp             =======================
/////====================================================================================

    void addStereoPoints(
      /*const */sptam::Map::SharedKeyFrame& keyFrame, std::vector<cv::Point3d>& points,
      std::vector<cv::Point2d>& featuresLeft, std::vector<cv::Mat>& descriptorsLeft,
      std::vector<cv::Point2d>& featuresRight, std::vector<cv::Mat>& descriptorsRight
    );

    void createNewPoints(sptam::Map::SharedKeyFrame& keyFrame);

    // Get Triangulated MapPoints from a given KeyFrame
    std::list<sptam::Map::SharedPoint> getPointsCretaedBy(sptam::Map::KeyFrame& keyFrame);

    /**
     * This tells if refind should try to match a certain point to a certain keyframe.
     * In the Sequential case, since the new_points are from a single keyframe
     * in each iteration, and that keyframe is not checked for refinds,
     * we can assume that the point was not matched before in the given list of keyframes.
     */
    virtual bool isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint);

    // When new map points are generated, they're only created from a stereo pair
    // this tries to make additional measurements in other KFs which they might
    // be in.
    // TODO se le puede cambiar el nombre a Refind (newly made depende del contexto),
    // también que devuelva la cantidad de encontrados y sacar el cout afuera.
    size_t ReFindNewlyMade(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames, Iterable<sptam::Map::SharedPoint>&& new_points);

    std::list< sptam::Map::SharedPoint > filterUnmatched(const sptam::Map::KeyFrame& keyFrame, Iterable<sptam::Map::SharedPoint>& mapPoints);

 private:
/////====================================================================================
/////============================       SPTAMInterface.cpp        =======================
/////====================================================================================

    // std::unique_ptr<SPTAM> sptam_;

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
    size_t lastImageSeq_ = 0;

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



/////====================================================================================
/////============================          sptam.cpp              =======================
/////====================================================================================
    
    sptam::Map map_;

    /*const */sptam::Map::SharedKeyFrame lastKeyFrame_;

    bool initialized_ = false;

    size_t frames_since_last_kf_ = 0;
    size_t frames_number_ = 0;


    // This mutex must be use to protect isPause, isTracking booleans and the error loop correction lc_T matrix.
    mutable std::mutex pause_mutex_;
    bool isTracking_ = false;
    bool isPaused_ = false;
    void setTracking(bool);

/////====================================================================================
/////============================        MapMaker.cpp             =======================
/////====================================================================================

    typedef fixed_queue< sptam::Map::SharedKeyFrame > KeyFrameCache;

//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - utilizo el puntero
    KeyFrameCache * local_keyframe_window_;
//++++++++++++++++++++++++++++++++++++++++++++++++


/////====================================================================================
/////============================            DSPTAM               =======================
/////====================================================================================

    ros::Publisher pub_msg_mapDiff_;  
    ros::Subscriber sub_mapRefined_;


    ros::Time currentTime; 
    size_t currentSeq;

    bool params_were_set_ = false;
    bool cameraCalibration_was_set_ = false;

    std::string descriptor_name;
    std::string detector_name;

    // Data to send to MapNode
    std::list< std::tuple<sptam::Map::SharedKeyFrame, sptam::Map::SharedPoint, Measurement *>  > listToMakeMSG_mapDiff;

    // Contador para el Id de MapPoints
    dsptam::id mapPoints_id_ = 0;

    // Translation de ptr a objetos
    dsptam::TranslationsMapPoint id2point{2000000};
    dsptam::TranslationsKeyFrame id2keyframe{5000};
};
