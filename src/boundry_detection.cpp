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
    boundrysearcher(boundry_nodes[0]);
    boundrysearcher(boundry_nodes[1]);
    boundrysearcher(boundry_nodes[2]);
}
void boundry_detection_class::boundrysearcher(boundry_node& boundry_node){
    if(image_classified != true){
        return;
    }
    int k = 1;
    boundry_node->max_x_boundry = boundry_node->boundry_vector[0];
    boundry_node->min_y_boundry = boundry_node->boundry_vector[0];
    boundry_node->max_y_boundry = boundry_node->boundry_vector[0];
    //sort acc to x
    for(int i = 0; i< boundry_node->boundry_vector.size()-1; i++){
        if(boundry_node->boundry_vector[i]->pixel_x>boundry_node->max_x_boundry->pixel_x){
            boundry_node->max_x_boundry = boundry_node->boundry_vector[i];
        }
        if(boundry_node->boundry_vector[i]->pixel_y<boundry_node->min_y_boundry->pixel_y){
            boundry_node->min_y_boundry = boundry_node->boundry_vector[i];
        }
        if(boundry_node->boundry_vector[i]->pixel_y>boundry_node->max_y_boundry->pixel_y){
            boundry_node->max_y_boundry = bboundry_node->boundry_vector[i];
        }
    }
    if(boundry_node == boundry_nodes[-1]){
        BoundrySearcherComplete = true;
    }
}
void boundry_detection_class::WPPlanner(){
    if(image_classified != true){
        return;
    }
    
    bspline(waypoint_vector);
}
void boundry_detection_class::bspline(){

}
void boundry_detection_class::update_map(){
    if(image_classified != true){
        return;
    }

}

void boundry_detection_class::algea_map(){
    
}
PLUGINLIB_EXPORT_CLASS(ns_boundry_detection::coverage_planning_node_class, nodelet::Nodelet)