#include<coverage_planning_node_class.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>

//take input from file

int coverage_planning_node_class::read(){

    string coverage_planning_area(_coverage_planning_area_);
    float data;
    ifstream coverage_planning_area_file(coverage_planning_area);
    
    if(coverage_planning_area_file.is_open()) {
        coordinates_node* temp = head;
        coverage_planning_area_file >> data;
        temp->coordinates_x = data;
        coverage_planning_area_file >> data;
        temp->coordinates_y = data;
        int number_of_data_points = 0;
        while (coverage_planning_area_file >> data)
        {
            temp1 = new coordinates_node();
            temp1->coordinates_x = data;
            coverage_planning_area_file >> data;
            temp1->coordinates_y = data;
            temp1->prev = temp;
            temp->next = temp1; 
            temp = temp1;
            number_of_data_points++;
        }
        temp->next = head;
        head->prev = temp;
        coverage_planning_area_file.close();
        sorting(head, number_of_data_points);
    }
    return 0;
}
void sorting (head, number_of_data_points){
    float sorted_array[number_of_data_points][2]

    for(int i=0; i<number_of_data_points; number_of_data_points++){
        
    }
}
float coverage_planning_node_class::dist(float x1, float y1, float x2, float y2){
    float dist_value = sqrt(pow((x1-x2), 2)+pow((y1-y2), 2));
    return dist_value;
}
//find longest edge
void coverage_planning_node_class::coordinate_finder(){
    float side_max=0;
    coordinates_node* iterator = head;
    coordinates_node* side_coordinate1;
    coordinates_node* side_coordinate2;
    float side_length = dist(iterator->coordinates_x, iterator->coordinates_y, (iterator->next)->coordinates_x, (iterator->next)->coordinates_y);
    do
    {
        iterator = iterator->next;
        if(side_length>side_max){
            side_max = side_length;
            side_coordinate1 = iterator;
            side_coordinate2 = iterator->next;
        }    
    }
    while (iterator->next!=head);

    trajectory_planner(side_coordinate1, side_coordinate2);
    //cout<<"coordinate side_coordinate1"<<side_coordinate1->coordinates_x <<","<<side_coordinate1->coordinates_y<<endl;
    //cout<<"coordinate side_coordinate2"<<side_coordinate2->coordinates_x <<","<<side_coordinate2->coordinates_y<<endl;

}

void coverage_planning_node_class::trajectory_planner(){
    vector<vector<float>> trajectory;
    vector<float> trajectory_coordinate;
    float x= prev_coordinate1->coordinates_x, y = prev_coordinate1->coordinates_y, r = 0.5;
    float slope1 = (prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x);
    float slope2 = (next_coordinate2->coordinates_y-next_coordinate1->coordinates_y)/(next_coordinate2->coordinates_x-next_coordinate1->coordinates_x);
    trajectory_coordinate.push_back(x);
    trajectory_coordinate.push_back(y);
    trajectory.push_back(trajectory_coordinate);
    int n = 0;
    while(prev_coordinate2->next!= next_coordinate2){
        y = n*r;
        x = prev_coordinate1->coordinates_x + (y)/slope1;
        trajectory_coordinate.push_back(x);
        trajectory_coordinate.push_back(y);
        trajectory.push_back(trajectory_coordinate);
        x = next_coordinate1->coordinates_x + (y)/slope2;
        trajectory_coordinate.push_back(x);
        trajectory_coordinate.push_back(y);
        trajectory.push_back(trajectory_coordinate);        
        n++;
    }
}
//transform trajectory in the world frame back
PLUGINLIB_EXPORT_CLASS(ns_coverage_path_node::coverage_planning_node_class, nodelet::Nodelet)?