#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>
#include <mrs_msgs/PathSrv.h>
#include <std_msgs/Float32MultiArray.h>
#include <mrs_lib/param_loader.h>
#include <pluginlib/class_list_macros.h>
#include <coverage_planning/CPPServiceCall.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <coverage_planning/CPPServiceCall.h>
#include <coverage_planning/UpdateMap.h>
namespace ns_boundry_detection{
    class boundry_detection_node_class : public nodelet::Nodelet{
        public:
        virtual void onInit();
        // class boundry_node{
        //     public:
        //     std::vector<pixel> boundry_vector;
        // };
        ros::ServiceClient                      update_map_service;
        void potential_field_generator(const sensor_msgs::Image::ConstPtr& msg);
        ros::ServiceClient                      coverage_planning_trajectory_service_client;
        // std::vector<std::vector<float>>         waypoint_vector;
        // std::vector<std::vector<float>>         trajectory;
        std::vector<float>                      potential_field;
        std::vector<int>                        waypoint_iterator_vector;
        std::vector<mrs_msgs::Reference>        ground_waypoint_vector;
        std::vector<mrs_msgs::Reference>        visited_waypoint_vector;
        std::vector<cv::Point>                  pixel_boundry_vector;
        mrs_msgs::Reference                     waypoint;
        mrs_msgs::PathSrv::Response             Pathres;
        coverage_planning::UpdateMap::Response  UpdateMapRes;
        float                                   altitude =5;
        void waypoint_generator(std::vector<float> potential_field);
        void waypoint_wrapper(std::vector<mrs_msgs::Reference> ground_waypoint_vector);
        void pixel23D(std::vector<float> waypoint_iterator_vector);
        void TFBroadcaster(std::vector<mrs_msgs::Reference> ground_waypoint_vector);
        image_geometry::PinholeCameraModel      camera_model_;
        void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
        ros::Subscriber                         sub_camera_info_;
        ros::Subscriber                         boundry_sub;
    };
}