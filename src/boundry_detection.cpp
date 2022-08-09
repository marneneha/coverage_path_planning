#include<boundry_detection.h>
namespace ns_boundry_detection{
    void boundry_detection_node_class::onInit(){
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
        ros::subscriber camera_input = nh.subscribe<mrs_msgs::camera_feed>("camera_feed_topic", CB_Camera_feed);
        ros::spin();
    }
}

bool boundry_detection_class::ImageCB(){
        //call Unet to get algea classified
        //detect boundry from segmented image to get vectorS of boundry
        //image classified =  true
    //hard code boundry_nodes
    //boundry nodes is set of vector
    boundrysearcher(boundry_nodes[0]);
    boundrysearcher(boundry_nodes[1]);
    boundrysearcher(boundry_nodes[2]);
}

bool boundry_detection_class::cameratoworld(){

}
void boundry_detection_class::boundrysearcher(boundry_node& boundry_node){
    if(image_classified != true){
        return;
    }
    //this function searches extreme boundry points for each boundry 
    
    for(int i=0; i<boundry_vector.size(); i++){
        
    }

    // concave_point_detect = false;
    // boundry_detect = false;
    // while((!concave_point_detect)||(!boundry_detect)){
    //     new_waypoint = boundry_vector[0];
    //     if()
    //     concave_point_detect = true;
    //     if()
    //     boundry_detect = true;
    // }
    //waypount_vector is in world frame mentioning x & y coordinate of points
    waypoint_vector.push_back(new_waypoint);
}

void boundry_detection_class::boundry_exploitation(){
    while(!boundry_reached || !loop_detect){
        new_waypoint = boundry_vector[0];
        if()
        concave_point_detect = true;
        if()
        boundry_detect = true;
    }
}

void boundry_detection_class::update_map(){
    if(image_classified != true){
        return;
    }
    coordiantes_node* temp_waypoint = new coordiantes_node();
    temp_waypoint->coordinates_x = waypoint_vector[0][0];
    temp_waypoint->coordinates_y = waypoint_vector[0][1];
    next_coordinate1->next = temp_waypoint;
    temp_waypoint->prev = next_coordinate1;
    for(int i = 1; i< waypoint_vector.size(); i++){
        coordiantes_node* temp_waypoint1 = new coordiantes_node();
        temp_waypoint->coordinates_x = waypoint_vector[i][0];
        temp_waypoint->coordinates_y = waypoint_vector[i][1];
        temp_waypoint1->prev = temp_waypoint;
        temp_waypoint->next = temp_waypoint1;
        temp_waypoint = temp_waypoint1;
    }
    temp_waypoint->next = next_coordinate2;
    next_coordinate2->prev = temp_waypoint;
    concavity_identifier();
}

PLUGINLIB_EXPORT_CLASS(ns_boundry_detection::coverage_planning_node_class, nodelet::Nodelet)