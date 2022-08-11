#include<coverage_planning_node_class.h>
#include<pluginlib/class_list_macros.h>

//here assumed coordinates cant be 3d
//look for memory deletion after use otherwise it will just keep adding to previous vector
using namespace std;
namespace ns_coverage_path_node{
void coverage_planning_node_class::onInit(){
    std::cout<<"INSIDE node class"<<std::endl;
	ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    mrs_lib::ParamLoader param_loader(nh, "coverage_planning");
	param_loader.loadParam("coverage_planning_area", _coverage_planning_area_);
        //string _coverage_planning_area_ = nh.get_param("coverage_planning_area");
        //advertise service or topic to publish 
    coverage_planning_trajectory_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_to_follow");
    // coverage_planning_node_class::read();
    // coverage_planning_node_class::coordinate_finder();
    ros::spin();
}
// int coverage_planning_node_class::read(){
//     string coverage_planning_area(_coverage_planning_area_);
//     float data;
//     ifstream coverage_planning_area_file(coverage_planning_area);
//     if(coverage_planning_area_file.is_open()) {
//         coordinates_node* temp = head;
//         coverage_planning_area_file >> data;
//         temp->coordinates_x = data;
//         coverage_planning_area_file >> data;
//         temp->coordinates_y = data;
//         while (coverage_planning_area_file >> data)
//         {
//             temp1 = new coordinates_node();
//             temp1->coordinates_x = data;
//             coverage_planning_area_file >> data;
//             temp1->coordinates_y = data;
//             temp1->prev = temp;
//             temp->next = temp1; 
//             temp = temp1;
//         }
//         temp->next = head;
//         head->prev = temp;
//         coverage_planning_area_file.close();
//     }
//     return 0;
// }
// float coverage_planning_node_class::dist (float x1, float y1, float x2, float y2){
//     float dist_value = sqrt(pow((x1-x2), 2)+pow((y1-y2), 2));
//     return dist_value;
// }
// void coverage_planning_node_class::coordinate_finder(head){
//     float side_max=0;
//     coordinates_node* iterator = head;
//     coordinates_node* side_coordinate1;
//     coordinates_node* side_coordinate2;
//     float side_length = dist(iterator, (iterator->next));
//     do{
//         iterator = iterator->next;
//         if(side_length>side_max){
//             side_max = side_length;
//             side_coordinate1 = iterator;
//             side_coordinate2 = iterator->next;
//         }    
//         side_length = dist(iterator, (iterator->next));
//     }
//     while (iterator->next!=head);

//     trajectory_planner(side_coordinate1, side_coordinate2);
//     //cout<<"coordinate side_coordinate1"<<side_coordinate1->coordinates_x <<","<<side_coordinate1->coordinates_y<<endl;
//     //cout<<"coordinate side_coordinate2"<<side_coordinate2->coordinates_x <<","<<side_coordinate2->coordinates_y<<endl;
// }
}
PLUGINLIB_EXPORT_CLASS(ns_coverage_path_node::coverage_planning_node_class, nodelet::Nodelet)
