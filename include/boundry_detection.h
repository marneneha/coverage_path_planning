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
#include <mrs_msgs/UavState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <coverage_planning/CPPServiceCall.h>
#include <coverage_planning/UpdateMap.h>
#include <coverage_planning/CPPServiceCall.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
// #include <Eigen/Geometry.h>
namespace ns_boundry_detection{
    class boundry_detection_node_class : public nodelet::Nodelet{
        public:
        virtual void onInit();
        // class boundry_node{
        //     public:
        //     std::vector<pixel> boundry_vector;
        // };
        class coordinates_node{
            public:
            float coordinates_x;
            float coordinates_y;
            float coordinates_z;
            coordinates_node* next;
            coordinates_node* prev;
        };
        ros::ServiceClient                      update_map_service;
        ros::ServiceClient                      coverage_planning_trajectory_service_client;
        bool                                    boundry_detect = false;
        bool                                    ground_waypoint_vector_update = false;
        bool                                    reached_first_waypoint = false;
        float                                   abselon = 0.5;
        cv::Mat                                 Canny_Image;
        // std::vector<std::vector<float>>         waypoint_vector;
        // std::vector<std::vector<float>>         trajectory;
        std::vector<float>                      potential_field;
        std::vector<int>                        waypoint_iterator_vector;
        std::vector<mrs_msgs::Reference>        ground_waypoint_vector;
        std::vector<mrs_msgs::Reference>        visited_waypoint_vector;
        std::vector<cv::Point>                  pixel_boundry_vector;
        // cv::Mat                                 pixel_boundry_vector;
        mrs_msgs::Reference                     waypoint;
        std::string                             _uav_name_;
        mrs_msgs::PathSrv::Response             Pathres;
        coverage_planning::UpdateMap::Request   UpdateMapReq;
        coverage_planning::UpdateMap::Response  UpdateMapRes;
        float                                   altitude =5;
        bool                                    got_camera_info_ = false;
        void waypoint_generator();
        void waypoint_wrapper();
        // void pixel23D();
        geometry_msgs::Point TFBroadcaster(geometry_msgs::Point waypoint);
        image_geometry::PinholeCameraModel      camera_model_;
        void potential_field_generator(const sensor_msgs::Image::ConstPtr& msg);
        void callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void callbackUavStatus(const mrs_msgs::UavState::ConstPtr& msg);
        ros::Subscriber                         sub_camera_info_;
        ros::Subscriber                         boundry_sub;
        ros::Subscriber                         sub_uav_state_info_;
        ros::Publisher                          waypoint_vector_pub;
    };
}