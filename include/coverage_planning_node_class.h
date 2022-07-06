#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <bits/stdc++.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_lib/param_loader.h>
#include <pluginlib/class_list_macros.h>
#include <coverage_planning/CPPServiceCall.h>
namespace ns_boustrophedon{
    class coverage_planning_node_class : public nodelet::Nodelet{
        public:
        virtual void onInit();
        //ros::NodeHandle nh;
        ros::ServiceClient                      coverage_planning_trajectory_service_client;
        class coordinates_node{
            public:
            float coordinates_x;
            float coordinates_y;
            coordinates_node* next;
            coordinates_node* prev;
        };
        class area_node{
            public:
            coordinates_node* prev_coordinate1;
            coordinates_node* prev_coordinate2;
            coordinates_node* next_coordinate1;
            coordinates_node* next_coordinate2;
            bool concavity = false;
        };
        std::vector<area_node*>                 divided_area;
        coordinates_node*                       head = new coordinates_node();
        coordinates_node*                       temp1;
        coordinates_node*                       prev_coordinate1;
        coordinates_node*                       prev_coordinate2;
        coordinates_node*                       next_coordinate1;
        coordinates_node*                       next_coordinate2;
        coordinates_node*                       prev_intermidiate_coordinate;
        coordinates_node*                       next_intermidiate_coordinate;

        bool                                    file_read = false;
        bool                                    sorting_status = false;
        bool                                    area_division_type = false;
        bool                                    concavity = false;
        std::string                             _coverage_planning_area_;
        float                                   sweeping_dist;
        mrs_msgs::Reference                     waypoint;
        std::vector <coordinates_node*>         concave_points;
        bool read(coverage_planning::CPPServiceCall::Request& req, coverage_planning::CPPServiceCall::Response& res);
        void coordinate_finder();
        int concavity_indentifier();
        void sorting(std::vector <coordinates_node*>& concave_points);
        void boustrophedon_area_division(coordinates_node* next_coordinate1, coordinates_node* next_coordinate2, coordinates_node* prev_coordinate1, coordinates_node* prev_coordinate2);
        void boustrophedon_matrix();
        coverage_planning_node_class::coordinates_node* extremum (coordinates_node* prev_coordinate, coordinates_node* next_coordinate, bool action);
        float max(float length1, float legth2);
        float dist (coordinates_node* node1, coordinates_node* node2);
        void trajectory_planner(coordinates_node* side_coordinate1, coordinates_node* side_coordinate2);
        };
}