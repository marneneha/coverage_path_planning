#include<boundry_detection.h>
// #include<coverage_planning_node_class.h>
namespace ns_boundry_detection{
void boundry_detection_node_class::onInit(){
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    std::cout<<"INSIDE boundry detection"<<std::endl;
    coverage_planning_trajectory_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_to_follow");
    // ros::Subscriber boundry_sub = nh.subscribe <std_msgs::Float32MultiArray>("boundry_topic", 10, &CameraToWorldCB, this, ros::TransportHints().tcpNoDelay());
    ros::spin();
}

// bool boundry_detection_class::ImageToWorldCB(& msg){
// 
// }
// void boundry_detection_class::TFBroadcaster(const mrs_msgs::Pose& msg){
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     transform.setorigin(tf::Vector3(msg->x, msg->y, msg->z));
//     tf::Quaternion q;
//     q.setRPY(0, 0, msg->theta);
//     transform.setRotation(q);
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "image", "world"));
// }
// void boundry_detection_class::potential_field_generator(boundry_node& boundry_node){
//     if(image_classified != true){
//         return;
//     }
//     //this function searches extreme boundry points for each boundry 
//     vector<float> new_waypoint;
//     vector<vector<float>> waypoint_vector;
//     std::vector<float> potential_field[boundry_vector.size()];
//     for(int i=0; i<boundry_vector.size(); i++){
//         potential_field[i] = inv(abs((x-xu)))+inv(abs((y-yu)));
//     }
//     // waypoint_vector.push_back(new_waypoint);
// }

// void boundry_detection_class::waypoint_generator(){
//     mrs_msgs::PathSrv::Request              req;
//     mrs_msgs::PathSrv::Response             res;
//     float* min = potential_field[0];
//     float* start = potential_field[0];
//     for(int i =0; i<potential_field.size(); i++){
//         if(potential_field[i]<*min){
//             min = &potential_field[i];
//         }
//     }
//     new_waypoints.add((min-start)/sizeof(float));
//     // new_waypoints = max;
//     // for(int =0; i<new_waypoints.size(); i++){
//     //     waypoints.add(new_waypoints[i]);
//     // }

//     for (int i = 0; i < new_waypoints.size(); i++){
//         std::cout<<"x is"<<trajectory[i][0]<<"y is"<<trajectory[i][1]<<std::endl;
//         waypoint.position.x = boundry_vector[new_waypoints[i]][0];
//         waypoint.position.y = boundry_vector[new_waypoints[i]][1];
//         waypoint.position.z = 5;
//         // waypoint.heading = current_pose.pose.position.heading;
//         waypoint.heading = 0;
//         visited_waypoint_vector.add(waypoint);
//         req.path.points.push_back(waypoint);
//     }
//     trajectory.clear();
//     req.path.header.seq = 0;
//     //req.path.header.time_stamp = ros::time.now();
//     req.path.header.frame_id = "gps_origin";
//     req.path.use_heading = false;
//     req.path.fly_now = true;
//     req.path.stop_at_waypoints = false;
//     req.path.loop = false;
//     req.path.override_constraints = false;
//     req.path.relax_heading = true;
//     coverage_planning_trajectory_service_client.call(req, res);
// }

// void boundry_detection_class::

// void boundry_detection_class::update_map(){
//     if(image_classified != true){
//         return;
//     }
//     coordiantes_node* temp_waypoint = new coordiantes_node();
//     temp_waypoint->coordinates_x = visited_waypoint_vector[0][0];
//     temp_waypoint->coordinates_y = visited_waypoint_vector[0][1];
//     next_coordinate1->next = temp_waypoint;
//     temp_waypoint->prev = next_coordinate1;
//     for(int i = 1; i< visited_waypoint_vector.size(); i++){
//         coordiantes_node* temp_waypoint1 = new coordiantes_node();
//         temp_waypoint->coordinates_x = waypoint_vector[i][0];
//         temp_waypoint->coordinates_y = waypoint_vector[i][1];
//         temp_waypoint1->prev = temp_waypoint;
//         temp_waypoint->next = temp_waypoint1;
//         temp_waypoint = temp_waypoint1;
//     }
//     temp_waypoint->next = next_coordinate2;
//     next_coordinate2->prev = temp_waypoint;
//     ns_boustrophedon::coverage_planning_node_class::concavity_identifier();
// }
}
PLUGINLIB_EXPORT_CLASS(ns_boundry_detection::boundry_detection_node_class, nodelet::Nodelet)