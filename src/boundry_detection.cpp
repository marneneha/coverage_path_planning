#include<boundry_detection.h>
// #include<coverage_planning_node_class.h>
namespace ns_boundry_detection{
void boundry_detection_node_class::onInit(){
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    std::cout<<"INSIDE boundry detection"<<std::endl;
    coverage_planning_trajectory_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_to_follow");
    update_map_service = nh.serviceClient<coverage_planning::UpdateMap>("update_map_service");
    boundry_sub = nh.subscribe<sensor_msgs::Image>("/uav1/coverage_planning_image_publisher/boundry_topic", 10, &boundry_detection_node_class::potential_field_generator, this);
    sub_camera_info_ = nh.subscribe<sensor_msgs::CameraInfo>("/uav1/bluefox_optflow/camera_info", 10, &boundry_detection_node_class::callbackCameraInfo, this);
    ros::spin();
}

// bool boundry_detection_node_class::ImageToWorldCB(const vision_msgs::Classification2D::ConstPtr& msg){
//     if(msg->results.size()==0){
//         return;
//     }
//     potential_field_generator(msg->result);
// }

void boundry_detection_node_class::potential_field_generator(const sensor_msgs::Image::ConstPtr& msg){
    if(!got_camera_info_){
        return;
    }
    std::cout<<"INSIDE potential_field_generator"<<std::endl;
    //this function searches extreme boundry points for each boundry 
    if(msg->data.size() == 0){
        ROS_ERROR("[CPP]: no image detected on subscriber node");
        return;
    }
    // std::cout<<"m here"<<std::endl;
    cv::Mat Canny_Image = cv_bridge::toCvShare(msg, "bgr8")->image;
    // std::cout<<"m here1"<<std::endl;
    cv::cvtColor(Canny_Image, Canny_Image, cv::COLOR_BGR2GRAY);
    cv::findNonZero(Canny_Image, pixel_boundry_vector);
    // std::cout<<"m here2"<<std::endl;
    float xu = msg->width/2;
    float yu = msg->height/2;
    for(int i=0; i<pixel_boundry_vector.size(); i++){
        float x = pixel_boundry_vector[i].x;
        float y = pixel_boundry_vector[i].y;
        potential_field.push_back((1/(abs(x-xu)))+(1/(abs(y-yu))));
    }
    boundry_detection_node_class::waypoint_generator(potential_field);
}
void boundry_detection_node_class::waypoint_generator(std::vector<float> potential_field){
    std::cout<<"INSIDE waypoint_generator"<<std::endl;
    float* min = &potential_field[0];
    float* start = &potential_field[0];
    for(int i =0; i<potential_field.size(); i++){
        if(potential_field[i]<*min){
            min = &potential_field[i];
        }
    }
    float iterator = (min-start)/sizeof(float);
    waypoint_iterator_vector.push_back(iterator);
    pixel23d(waypoint_iterator_vector);
}
void boundry_detection_node_class::pixel23D(std::vector<float> waypoint_iterator_vector){
    std::cout<<"INSIDE pixel23D"<<std::endl;
    //  uv_rect;
    float x, y, z;
    for(int i = 0; pixel_boundry_vector.size(); i++){
        cv::Point2d pt2d(pixel_boundry_vector[i].x, pixel_boundry_vector[i].y);
        cv::Point2d uv_rect = camera_model_.rectifyPoint(pt2d);
        cv::Point3d pt3d = camera_model_.projectPixelTo3dRay(uv_rect);
        //subscribe to know the altitude
        x = pt3d.x*altitude;
        y = pt3d.y*altitude;
        z = pt3d.z*altitude;
        ground_waypoint_vector[i].position.x = x;
        ground_waypoint_vector[i].position.y = y;
        ground_waypoint_vector[i].position.z = z;
    }
    boundry_detection_node_class::TFBroadcaster(ground_waypoint_vector);
    boundry_detection_node_class::waypoint_wrapper(ground_waypoint_vector);
}
void boundry_detection_node_class::waypoint_wrapper(std::vector<mrs_msgs::Reference> ground_waypoint_vector){ 
    std::cout<<"INSIDE waypoint_wrapper"<<std::endl;
        mrs_msgs::PathSrv::Request              Pathreq;
        coverage_planning::UpdateMap::Request   UpdateMapReq;

    for(int i = 0; i < ground_waypoint_vector.size(); i++){
        std::cout<<"x is"<<ground_waypoint_vector[i].position.x<<"y is"<<ground_waypoint_vector[i].position.y<<std::endl;
        waypoint.position = ground_waypoint_vector[waypoint_iterator_vector[i]].position;
        // waypoint.heading = current_pose.pose.position.heading;
        //this is pblm look into it
        waypoint.heading = 0;
        visited_waypoint_vector.push_back(waypoint);
        Pathreq.path.points.push_back(waypoint);
    }
    waypoint_iterator_vector.clear();
    ground_waypoint_vector.clear();
    potential_field.clear();
    visited_waypoint_vector.clear();
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
    // put better condition
    while(1){
        for(int i=0; i < visited_waypoint_vector.size(); i++){
            geometry_msgs::Point visited_waypoint;
            visited_waypoint.x = visited_waypoint_vector[i].position.x;
            visited_waypoint.y = visited_waypoint_vector[i].position.y;
            visited_waypoint.z = visited_waypoint_vector[i].position.z;
            UpdateMapReq.visited_waypoints.push_back(visited_waypoint);
        }
        visited_waypoint_vector.clear();
        update_map_service.call(UpdateMapReq, UpdateMapRes);
    }
    
}
void boundry_detection_node_class::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
    std::cout<<"INSIDE callbackCameraInfo"<<std::endl;

  got_camera_info_       = true;
//   time_last_camera_info_ = ros::Time::now();

  // update the camera model using the latest camera info message
  camera_model_.fromCameraInfo(*msg);
}
void boundry_detection_node_class::TFBroadcaster(std::vector<mrs_msgs::Reference> ground_waypoint_vector){
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