#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <bits/stdc++.h>
#include <mrs_msgs/PathSrv.h>

namespace ns_coverage_path_node{
    class coverage_planning_node_class : public nodelet::Nodelet{
        public:
        virtual void onInit();
        //ros::NodeHandle nh;
        ros::ServiceClient          coverage_planning_trajectory_service_client;
        class coordinates_node{
            public:
            float coordinates_x;
            float coordinates_y;
            coordinates_node* next;
            coordinates_node* prev;
        };
        coordinates_node*                       head = new coordinates_node();
        coordinates_node*                       temp1;

        mrs_msgs::Reference                     waypoint;           
        int read();
        void coordinate_finder();
        float dist (float x1, float y1, float x2, float y2);
        void trajectory_planner(coordinates_node* side_coordinate1, coordinates_node* side_coordinate2);

        };
}