#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <bits/stdc++.h>
#include <mrs_msgs/PathSrv.h>
#include <std_msgs/Float32MultiArray.h>
#include <mrs_lib/param_loader.h>
#include <pluginlib/class_list_macros.h>
#include <coverage_planning/CPPServiceCall.h>
#include <tf/transform_broadcaster.h>
namespace ns_boundry_detection{
    class boundry_detection_node_class : public nodelet::Nodelet{
        public:
        virtual void onInit();
        // class boundry_node{
        //     public:
        //     std::vector<pixel> boundry_vector;
        // };
        // void CameraToWorldCB(std_msgs::Float32MultiArray::ConstPtr& msg);
        ros::ServiceClient                      coverage_planning_trajectory_service_client;
        std::vector<std::vector<float>>         waypoint_vector;
    };
}