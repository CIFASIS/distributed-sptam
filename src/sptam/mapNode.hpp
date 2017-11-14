#include <ros/ros.h>

#include "utils/messages_handler.hpp"               // msgs functions and types
#include "utils/translations.hpp"

#include "Map.hpp"
#include "CameraPose.hpp"                     	 	// typedef for Matrix6d
#include "MapMakerThread.hpp"

#include "match_to_points.hpp"						// Match
#include "KeyFramePolicy.hpp"						// AddKeyFrameTrackingFeaturesPolicy
#include <pcl_ros/point_cloud.h>					// pcl

#ifdef SHOW_PROFILING
    #include "../sptam/utils/log/Profiler.hpp"
    #include "../sptam/utils/log//Logger.hpp"
#endif

class MapNode
{
public:
    MapNode(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    void callback_KeyFrame();
    void callback_mapDiff(const sptam::msg_mapDiff& mapDiff);
    void get_params();

private:
    //Parameters
    bool params_were_get_ = false;
    std::string descriptor_name;
    std::string detector_name;
    double baseline;
    int MatchingCellSize;
    int left_img_width, left_img_height, right_img_width, right_img_height;
    double left_horizontalFOV, left_verticalFOV;
    double right_horizontalFOV, right_verticalFOV;
    double frustumNearPlaneDist, frustumFarPlaneDist;
    std::vector<double> intrinsic_vector;
    cv::Matx33d intrinsic;
    CameraParameters* calibrationLeft; 
    CameraParameters* calibrationRight; 
    
/////====================================================================================
/////=========================       SPTAMInterface.cpp           =======================
/////====================================================================================

    Parameters sptam_params_;

/////====================================================================================
/////============================          sptam.cpp              =======================
/////====================================================================================
    
    // Internal Map
    sptam::Map map_;  

    MapMakerThread * mapMaker_;


/////====================================================================================
/////============================            DSPTAM               =======================
/////====================================================================================

    // Subscribers
    ros::Subscriber sub_mapDiff_;
    
    // Publishers
    ros::Publisher pub_mapRefined_;

    // Data to send to MapNode
    std::tuple< std::list< sptam::Map::SharedKeyFrame >, std::list< sptam::Map::SharedPoint > > listToMakeMSG_mapRefined;

    // Contador para el Id de MapPoints
    dsptam::id mapPoints_id_ = 0;

    // Translation de ptr a objetos
    dsptam::TranslationsMapPoint id2point{2000000};
    dsptam::TranslationsKeyFrame id2keyframe{5000};

};