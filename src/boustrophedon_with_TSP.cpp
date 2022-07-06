#include<coverage_planning_node_class.h>

namespace ns_boustrophedon{
void coverage_planning_node_class::onInit() {
	ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    coverage_planning_trajectory_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_to_follow");
    ros::ServiceServer CPP_call_file = nh.advertiseService("CPP_service_call", &coverage_planning_node_class::read, this);
    std::cout<<"INSIDE ONINIT"<<std::endl;
    ros::spin();
}
bool coverage_planning_node_class::read( coverage_planning::CPPServiceCall::Request& req,  coverage_planning::CPPServiceCall::Response& res){
    std::string _coverage_planning_area_ = req.file_name;
    std::string coverage_planning_area(_coverage_planning_area_);
    float data;
    std::ifstream coverage_planning_area_file(coverage_planning_area);  
    if(!coverage_planning_area_file.is_open()){
        ROS_ERROR("[CPP]:trajectory file provided cant be opened");
        return 0;
    } 
    if(coverage_planning_area_file.is_open()){
        coordinates_node* temp = head;
        coverage_planning_area_file >> data;
        temp->coordinates_x = data;
        coverage_planning_area_file >> data;
        temp->coordinates_y = data;
        int number_of_data_points = 0;
        while(coverage_planning_area_file >> data){
            temp1 = new coordinates_node();
            temp1->coordinates_x = data;
            coverage_planning_area_file >> data;
            temp1->coordinates_y = data;
            temp1->prev = temp;
            temp->next = temp1; 
            temp = temp1;
            number_of_data_points++;
            file_read = true;
        }
        temp->next = head;
        head->prev = temp;
        coverage_planning_area_file.close();
    }
    coverage_planning_node_class::concavity_indentifier();
    return 0;
}
int coverage_planning_node_class::concavity_indentifier(){
    //check if file is read or not
    if(!file_read){
        ROS_ERROR("[CPP]: file is opened but cant be read or there is no data saved in the file");
        return 0;
    }
    coordinates_node* prev_coordinate = head->prev;
    coordinates_node* mid_coordinate =  head;
    coordinates_node* next_coordinate = head->next;
    float product;
    do{
        product = (next_coordinate->coordinates_x- mid_coordinate->coordinates_x) * (mid_coordinate->coordinates_y - prev_coordinate->coordinates_x);
        if(product < 0){
            //add to concavity point vector
            concave_points.push_back(mid_coordinate);
        }
        prev_coordinate = prev_coordinate->next;
        mid_coordinate = mid_coordinate->next;
        next_coordinate = next_coordinate->next;
    }
    while(mid_coordinate->next!=head);

    sorting(concave_points);
    return 0;
}
//implement in place sorting algo along x for concave points
void coverage_planning_node_class::sorting(std::vector <coordinates_node*>& concave_points){

    boustrophedon_area_division(concave_points[0], concave_points[0]->next, concave_points[0], concave_points[0]->prev);
    sorting_status = true;
}
//put it in include
void coverage_planning_node_class::boustrophedon_area_division(coordinates_node* next_coordinate1, coordinates_node* next_coordinate2, coordinates_node* prev_coordinate1, coordinates_node* prev_coordinate2){
    if(!sorting_status){
        ROS_ERROR("[CPP] :  sorting was not successfull");
        return;
    }
    int i = 0;
    coordinates_node* concave_point = concave_points[0];
    float x = concave_point->coordinates_x;
    std::vector<area_node*> divided_area;
    area_node* temp_area_node;
    float slope1, slope2;
    do{
        if(((x<next_coordinate2->coordinates_x)&&(x>next_coordinate1->coordinates_x))&& ((x<prev_coordinate2->coordinates_x)&&(x>prev_coordinate1->coordinates_x))){
            //area division
            slope1 = (next_coordinate2->coordinates_y-next_coordinate1->coordinates_y)/(next_coordinate2->coordinates_x-next_coordinate1->coordinates_x);
            slope2 = (prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x);
            next_intermidiate_coordinate->coordinates_y = slope1*x + next_coordinate1->coordinates_y - slope1*next_coordinate1->coordinates_x;
            prev_intermidiate_coordinate->coordinates_y = slope2*x + prev_coordinate1->coordinates_y - slope2*prev_coordinate1->coordinates_x;
            prev_intermidiate_coordinate->coordinates_x = x;
            next_intermidiate_coordinate->coordinates_x = x;
            temp_area_node->prev_coordinate1 = prev_coordinate1;
            temp_area_node->prev_coordinate2 = prev_intermidiate_coordinate;
            temp_area_node->next_coordinate1 = next_coordinate1;
            temp_area_node->next_coordinate2 = next_intermidiate_coordinate;
            divided_area.push_back(temp_area_node);
            boustrophedon_area_division(next_intermidiate_coordinate, next_coordinate2, concave_point, concave_point->next);
            boustrophedon_area_division(concave_point, concave_point->prev, prev_intermidiate_coordinate, prev_coordinate2);
            i++;
            x = concave_points[i]->coordinates_x;
            area_division_type = true;
        }
        else if((x<next_coordinate2->coordinates_x)&&(x>next_coordinate1->coordinates_x)){
            prev_coordinate1 = prev_coordinate1->next;
            prev_coordinate2 = prev_coordinate2->next;
            concavity = true;
        }
        else if((x<prev_coordinate2->coordinates_x)&&(x>prev_coordinate1->coordinates_x)){
            next_coordinate1 = next_coordinate1->next;
            next_coordinate2 = next_coordinate2->next;
            concavity = true;
        }
        else{
            //improve
            prev_coordinate1 = prev_coordinate1->next;
            prev_coordinate2 = prev_coordinate2->next;
            next_coordinate1 = next_coordinate1->next;
            next_coordinate2 = next_coordinate2->next;
            concavity = true;
        }
        
    }
    while(prev_coordinate2->next!= next_coordinate2);
}
void coverage_planning_node_class::boustrophedon_matrix(){
    if(!area_division_type){
        ROS_ERROR("[CPP] : Area division did not happen right");
        return;
    }
    float total_distance, distance1, distance2, distance3, prev_length, next_length;
    std::vector<float> weighted_array;
    std::vector<std::vector<float>> weighted_matrix;
    // coordinates_node* prev_coordinate1, prev_coordinate2, next_coordinate1, next_coordinate2;
    for(int i = 0; i< divided_area.size(); i++){
        for(int k = 0; k<4; k++){
            prev_length = dist(prev_coordinate1, prev_coordinate2);
            next_length = dist(next_coordinate1, next_coordinate2);
            distance1 = 2*(max(prev_length, next_length)/sweeping_dist);
            for(int j = 0; j<divided_area.size(); j++){
                if(i == j){
                    total_distance = 1000;
                    weighted_array.push_back(total_distance);
                    printf("%f",total_distance);
                }
                else{
                    distance2 = dist(extremum(prev_coordinate2, next_coordinate2, 1), extremum(prev_coordinate1, next_coordinate1, 0));
                    for(int l = 0; l<4; l++){
                        prev_length = dist(prev_coordinate1, prev_coordinate2);
                        next_length = dist(next_coordinate1, next_coordinate2);
                        distance3 = 2*(max(prev_length, next_length)/sweeping_dist);
                        total_distance = distance1 + distance2 + distance3;
                        weighted_array.push_back(total_distance);
                        if(concavity){
                            k++;
                            prev_coordinate1 = prev_coordinate1->prev;
                            prev_coordinate2 = prev_coordinate2->prev;
                            next_coordinate1 = next_coordinate1->prev;
                            next_coordinate2 = next_coordinate2->prev;
                        }
                        prev_coordinate1 = prev_coordinate1->prev;
                        prev_coordinate2 = prev_coordinate2->prev;
                        next_coordinate1 = next_coordinate1->prev;
                        next_coordinate2 = next_coordinate2->prev;
                    }
                }
            }
            printf("/n");
            weighted_matrix.push_back(weighted_array);
            if(concavity){
                k++;
                prev_coordinate1 = prev_coordinate1->prev;
                prev_coordinate2 = prev_coordinate2->prev;
                next_coordinate1 = next_coordinate1->prev;
                next_coordinate2 = next_coordinate2->prev;
            }
            prev_coordinate1 = prev_coordinate1->prev;
            prev_coordinate2 = prev_coordinate2->prev;
            next_coordinate1 = next_coordinate1->prev;
            next_coordinate2 = next_coordinate2->prev;            
        }
        
    }
}
//node_read (read from the file tours what is the order for sweeping)
//trajectory planner for sweeping the area this trajectory will be sent to trajectory planner servis 
coverage_planning_node_class::coordinates_node* coverage_planning_node_class::extremum (coordinates_node* prev_coordinate, coordinates_node* next_coordinate, bool action){
    if(action){
        if(prev_coordinate->coordinates_x > next_coordinate->coordinates_x){
            return prev_coordinate;
        }
        else
        return next_coordinate;
    }
    else{
        if(prev_coordinate->coordinates_x < next_coordinate->coordinates_x){
            return prev_coordinate;
        }
        else
        return next_coordinate;
    }
}
float coverage_planning_node_class::dist(coordinates_node* node1, coordinates_node* node2){
    float dist_value = sqrt(pow((node1->coordinates_x -node2->coordinates_x), 2)+pow((node1->coordinates_y -node2->coordinates_y), 2));
    return dist_value;
}
float coverage_planning_node_class::max(float length1, float length2){
    if(length1>= length2)
        return length1;
    else
        return length2;
}
}
PLUGINLIB_EXPORT_CLASS(ns_boustrophedon::coverage_planning_node_class, nodelet::Nodelet)