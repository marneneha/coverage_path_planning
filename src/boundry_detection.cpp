#include<boundry_detection.h>
// #include<coverage_planning_node_class.h>
namespace ns_boundry_detection{
void boundry_detection_node_class::onInit(){
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
	mrs_lib::ParamLoader param_loader(nh, "coverage_planning_boundry_detection");
    param_loader.loadParam("uav_name", _uav_name_);
    std::cout<<"INSIDE boundry detection"<<std::endl;
    coverage_planning_trajectory_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_to_follow");
    update_map_service = nh.serviceClient<coverage_planning::UpdateMap>("update_map_service");
    boundry_sub = nh.subscribe<sensor_msgs::Image>("boundry_topic", 10, &boundry_detection_node_class::potential_field_generator, this);
    sub_camera_info_ = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, &boundry_detection_node_class::callbackCameraInfo, this);
    sub_uav_state_info_ = nh.subscribe<mrs_msgs::UavState>("uav_state", 1, &boundry_detection_node_class::callbackUavStatus, this);
    ground_waypoint_pub = nh.advertise<mrs_msgs::Reference>("waypoint_topic", 1);
    ros::spin();
}

// bool boundry_detection_node_class::ImageToWorldCB(const vision_msgs::Classification2D::ConstPtr& msg){
//     if(msg->results.size()==0){
//         return;
//     }
//     potential_field_generator(msg->result);
// }

void boundry_detection_node_class::potential_field_generator(const sensor_msgs::Image::ConstPtr& msg){
    // if(boundry_detect)(
    //     while(){}
    // )
    std::cout<<"INSIDE potential_field_generator1"<<std::endl;
    if(!got_camera_info_){
        return;
    }
    std::cout<<"INSIDE potential_field_generator"<<std::endl;
    //this function searches extreme boundry points for each boundry 
    if(msg->data.size() == 0){
        ROS_ERROR("[CPP]: no image detected on subscriber node");
        return;
    }
    boundry_detect = true;
    Canny_Image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(Canny_Image, Canny_Image, cv::COLOR_BGR2GRAY);
    cv::findNonZero(Canny_Image, pixel_boundry_vector);
    float xu = msg->width/2;
    float yu = msg->height/2;
    for(int i=0; i<pixel_boundry_vector.size(); i++){
        float x = pixel_boundry_vector[i].x;
        float y = pixel_boundry_vector[i].y;
        potential_field.push_back((1/(abs(x-xu)))+(1/(abs(y-yu))));
    }
    boundry_detection_node_class::waypoint_generator();
    // res = 
    // coordinates_node* next_coordinate1 = msg->next_coordinate1;
    // coordinates_node* next_coordinate2 = msg->next_coordinate2;
    // coordinates_node* prev_coordinate1 = msg->prev_coordinate1;
    // coordinates_node* prev_coordinate2 = msg->prev_coordinate2;
}
//most imp
void boundry_detection_node_class::waypoint_generator(){
    std::cout<<"INSIDE waypoint_generator"<<std::endl;
    float* min = &potential_field[0];
    float* start = &potential_field[0];
    for(int i =0; i<potential_field.size(); i++){
        if(potential_field[i]<*min){
            min = &potential_field[i];
        }
    }
    waypoint_iterator = (min-start)/sizeof(float);
    boundry_detection_node_class::waypoint_wrapper();
}
// void boundry_detection_node_class::pixel23D(){
//     std::cout<<"INSIDE pixel23D"<<std::endl;
//     //  uv_rect;
//     // std::cout<<"m here"<<std::endl;
//     for(int i = 0; i< pixel_boundry_vector.size(); i++){
//             // std::cout<<"m here1"<<std::endl;
//             // std::cout<<"m here2"<<std::endl;
//             // std::cout<<"m here4"<<std::endl;
//         ground_waypoint_vector.push_back(ground_waypoint);
//         ground_waypoint_vector_update = true;
//             // std::cout<<"m here5"<<"i is"<<i<<std::endl;
//             // i = i+10;
//     }   
// }
void boundry_detection_node_class::waypoint_wrapper(){ 
    // std::cout<<"INSIDE waypoint_wrapper"<<"ground_waypoint_vector size is"<<ground_waypoint_vector.size()<<std::endl;
        mrs_msgs::PathSrv::Request              Pathreq;
        coverage_planning::UpdateMap::Request   UpdateMapReq;
        mrs_msgs::Reference ground_waypoint;

        cv::Point2d pt2d(pixel_boundry_vector[waypoint_iterator].x, pixel_boundry_vector[waypoint_iterator].y);
        cv::circle(Canny_Image, (pt2d), 0, (255, 0, 0), -1, cv::LINE_8, 0);
        cv::imwrite("imagewithwaypoint", Canny_Image);
        cv::Point2d uv_rect = camera_model_.rectifyPoint(pt2d);
        cv::Point3d pt3d = camera_model_.projectPixelTo3dRay(uv_rect);
        waypoint.position.x = pt3d.x*altitude;
        waypoint.position.y = pt3d.y*altitude;
        waypoint.position.z = pt3d.z*altitude;
        ground_waypoint.position = boundry_detection_node_class::TFBroadcaster(waypoint.position);
        std::cout<<"x is"<<ground_waypoint.position.x<<"y is"<<ground_waypoint.position.y<<"z is"<<ground_waypoint.position.z<<"waypoint_iterator is"<<waypoint_iterator<<std::endl;
        // waypoint.heading = current_pose.pose.position.heading;
        //this is pblm look into it
        ground_waypoint.heading = 0;
        // ground_waypoint_vector.push_back(ground_waypoint);
        // if(visited_waypoint_vector.size() == 0){
        //     visited_waypoint_vector.push_back(ground_waypoint);
        // }
        // else if(ground_waypoint.position.x != visited_waypoint_vector[-1].position.x || ground_waypoint.position.y != visited_waypoint_vector[-1].position.y){
        //     visited_waypoint_vector.push_back(ground_waypoint);
        // }
        Pathreq.path.points.push_back(ground_waypoint);
    ground_waypoint_pub.publish(ground_waypoint);
    // waypoint_iterator_vector.clear();
    // ground_waypoint_vector.clear();
    potential_field.clear();
    // visited_waypoint_vector.clear();
    pixel_boundry_vector.clear();
    Pathreq.path.header.seq = 0;
    // Pathreq.path.header.stamp = ros::time.now();
    Pathreq.path.header.frame_id = "gps_origin";
    Pathreq.path.use_heading = false;
    Pathreq.path.fly_now = true;
    Pathreq.path.stop_at_waypoints = false;
    Pathreq.path.loop = false;
    Pathreq.path.override_constraints = false;
    Pathreq.path.relax_heading = true;
    coverage_planning_trajectory_service_client.call(Pathreq, Pathres);    
}
// void boundry_detection_node_class::callbackUavStatus(const mrs_msgs::UavState::ConstPtr& msg){
//         // put better condition
//     if(!boundry_detect || !ground_waypoint_vector_update){
//         return;
//     }
//     geometry_msgs::Point visited_waypoint;
//     Eigen::Vector3d init_point_to_current_point, prev_point_to_boundry, next_point_to_boundry, current_point_to_first_waypoint;
//     Eigen::Vector3d current_point(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     Eigen::Vector3d first_waypoint(ground_waypoint_vector[0].position.x, ground_waypoint_vector[0].position.y, ground_waypoint_vector[0].position.z);
//     current_point_to_first_waypoint = current_point - first_waypoint;
//     init_point_to_current_point = current_point - init_pose;
//     prev_point_to_boundry = current_point - (prev_coordinate2-prev_coordinate2->next);
//     next_point_to_boundry = current_point - (next_coordinate2-next_coordinate2->prev);
//     altitude = msg->pose.position.z;
//     if((current_point_to_first_waypoint.norm()<abselon)){
//         reached_first_waypoint = true;
//     }
//     if(!reached_first_waypoint){
//         return;
//     }
//     if(msg->pose.position.x > next_coordinate2.x){
//         next_coordinate2 = next_coordinate2->next;
//     }
//     if(msg->pose.position.x > prev_coordinate2.x){
//         prev_coordinate2 = prev_coordinate2->prev;
//     }    
//     if(init_point_to_current_point.norm()<abselon){
//         for(int i=0; i < visited_waypoint_vector.size(); i++){
//             visited_waypoint.x = visited_waypoint_vector[i].position.x;
//             visited_waypoint.y = visited_waypoint_vector[i].position.y;
//             visited_waypoint.z = visited_waypoint_vector[i].position.z;
//             UpdateMapReq.NoFlyZone = true;
//             UpdateMapReq.visited_waypoints.push_back(visited_waypoint);
//         }
//         visited_waypoint_vector.clear();
//         update_map_service.call(UpdateMapReq, UpdateMapRes);
//         boundry_detect = false;
//         reached_first_waypoint = false;
//         ground_waypoint_vector_update = false;
//     }
//     if(prev_point_to_boundry.norm()<abselon || next_point_to_boundry.norm()<abselon){
//         for(int i=0; i < visited_waypoint_vector.size(); i++){
//             visited_waypoint.x = visited_waypoint_vector[i].position.x;
//             visited_waypoint.y = visited_waypoint_vector[i].position.y;
//             visited_waypoint.z = visited_waypoint_vector[i].position.z;
//             UpdateMapReq.NoFlyZone = false;
//             UpdateMapReq.visited_waypoints.push_back(visited_waypoint);
//         }
//         visited_waypoint_vector.clear();
//         update_map_service.call(UpdateMapReq, UpdateMapRes);
//         boundry_detect = false;
//         reached_first_waypoint = false;
//         ground_waypoint_vector_update = false;        
//     }
// }
void boundry_detection_node_class::callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg){
    // std::cout<<"INSIDE callbackCameraInfo"<<std::endl;

  got_camera_info_       = true;
//   time_last_camera_info_ = ros::Time::now();

  // update the camera model using the latest camera info message
  camera_model_.fromCameraInfo(*msg);
}
geometry_msgs::Point boundry_detection_node_class::TFBroadcaster(geometry_msgs::Point waypoint){
    std::cout<<"INSIDE TFBroadcaster"<<std::endl;
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setorigin(tf::Vector3(msg->x, msg->y, msg->z));
    // tf::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "image", "world"));
}

}
PLUGINLIB_EXPORT_CLASS(ns_boundry_detection::boundry_detection_node_class, nodelet::Nodelet)