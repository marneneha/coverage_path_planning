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
    // points are read in clockwise format
    std::string _coverage_planning_area_ = req.file_name;
    std::string coverage_planning_area(_coverage_planning_area_);
    coordinates_node* data_iterator;
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

    std::cout<<"file has been read"<<std::endl;
    data_iterator = head;
    do
    {
        std::cout<<data_iterator->coordinates_x<<","<<data_iterator->coordinates_y<<std::endl;  
        data_iterator = data_iterator->next;      
    } while ( data_iterator != head);
    
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
        product = (next_coordinate->coordinates_x- mid_coordinate->coordinates_x) * (mid_coordinate->coordinates_x - prev_coordinate->coordinates_x);
        // std::cout<<"point is"<<mid_coordinate->coordinates_x<<","<<mid_coordinate->coordinates_y<<"product is"<<product<<"\n"<<std::endl; 
        //judge onthe basis of angle greater than 180 or not 
        if(product < 0 && (mid_coordinate->coordinates_x - prev_coordinate->coordinates_x) <0 ){
            concave_points.push_back(mid_coordinate);
        }
        prev_coordinate = prev_coordinate->next;
        mid_coordinate = mid_coordinate->next;
        next_coordinate = next_coordinate->next;
    }
    while(mid_coordinate !=head);
    ROS_INFO("[CPP] : list of unsorted concave points are");
    for(int i = 0; i<concave_points.size(); i++){
        std::cout<<concave_points[i]->coordinates_x<<","<<concave_points[i]->coordinates_y<<std::endl;  
    }
    sorting(concave_points);
    return 0;
}
void coverage_planning_node_class::sorting(std::vector <coordinates_node*>& concave_points){
    int k = 1;
    coordinates_node* temp;
    
    while(k){
        k = 0;
        for(int i = 0; i< concave_points.size()-1; i++){
            if(concave_points[i+1]->coordinates_x<concave_points[i]->coordinates_x){
                temp = concave_points[i];
                concave_points[i] = concave_points[i+1];
                concave_points[i+1] = temp;
                k++;
            }
        }
    }
    ROS_INFO("[CPP] : list of sorted concave points are");
    for(int i = 0; i<concave_points.size(); i++){
        std::cout<<concave_points[i]->coordinates_x<<","<<concave_points[i]->coordinates_y<<std::endl;  
    }
    sorting_status = true;
    coordinates_node* concave_point = concave_points[0];
    concave_points_iterator = concave_points.begin();
    concave_points.erase(concave_points_iterator);
    boustrophedon_area_division(concave_point, concave_point->next, concave_point, concave_point->prev);
}
void coverage_planning_node_class::boustrophedon_area_division(coordinates_node* next_coordinate1_orig, coordinates_node* next_coordinate2_orig, coordinates_node* prev_coordinate1_orig, coordinates_node* prev_coordinate2_orig){
    if(!sorting_status){
        ROS_ERROR("[CPP] :  sorting was not successfull");
        return;
    }
    float x, y, slope1, slope2;
    area_node* temp_area_node = new area_node();
    coordinates_node* prev_coordinate1;
    coordinates_node* prev_coordinate2;
    coordinates_node* next_coordinate1;
    coordinates_node* next_coordinate2;    for(int i=0; i<concave_points.size(); i++){
        coordinates_node* concave_point = concave_points[i];
        x = concave_point->coordinates_x;
        y = concave_point->coordinates_y;
        prev_coordinate1 = prev_coordinate1_orig;
        prev_coordinate2 = prev_coordinate2_orig;
        next_coordinate1 = next_coordinate1_orig;
        next_coordinate2 = next_coordinate2_orig;
        ROS_ERROR("[CPP] : at the start of do-while loop");
        std::cout<<"next_coordinate1_orig is"<<next_coordinate1->coordinates_x<<","<<next_coordinate1->coordinates_y<<"\n"<<std::endl;  
        std::cout<<"next_coordinate2_orig is"<<next_coordinate2->coordinates_x<<","<<next_coordinate2->coordinates_y<<"\n"<<std::endl;  
        std::cout<<"prev_coordinate1_orig is"<<prev_coordinate1->coordinates_x<<","<<prev_coordinate1->coordinates_y<<"\n"<<std::endl;  
        std::cout<<"prev_coordinate2_orig is"<<prev_coordinate2->coordinates_x<<","<<prev_coordinate2->coordinates_y<<"\n"<<std::endl;
        std::cout<<"next prev_coordinate2_orig is"<<prev_coordinate2->next->coordinates_x<<","<<prev_coordinate2->next->coordinates_y<<"\n"<<std::endl;
        while((prev_coordinate2 != next_coordinate2)&&(next_coordinate2->next != prev_coordinate2)){
            std::cout<<"next_coordinate1 is"<<next_coordinate1->coordinates_x<<","<<next_coordinate1->coordinates_y<<"\n"<<std::endl;  
            std::cout<<"next_coordinate2 is"<<next_coordinate2->coordinates_x<<","<<next_coordinate2->coordinates_y<<"\n"<<std::endl;  
            std::cout<<"prev_coordinate1 is"<<prev_coordinate1->coordinates_x<<","<<prev_coordinate1->coordinates_y<<"\n"<<std::endl;  
            std::cout<<"prev_coordinate2 is"<<prev_coordinate2->coordinates_x<<","<<prev_coordinate2->coordinates_y<<"\n"<<std::endl;
            std::cout<<"x is"<<x<<"y is"<<y<<std::endl; 
            if(((x<next_coordinate2->coordinates_x)&&(x>next_coordinate1->coordinates_x))&& ((x<prev_coordinate2->coordinates_x)&&(x>prev_coordinate1->coordinates_x))){
                //area division
                ROS_INFO("i here1");
                if(((y<next_coordinate2->coordinates_y)||(y<next_coordinate1->coordinates_y))&& ((y>prev_coordinate2->coordinates_y)||(y>prev_coordinate1->coordinates_y))){
                    prev_intermidiate_coordinate = new coordinates_node();
                    next_intermidiate_coordinate = new coordinates_node();
                    slope1 = (next_coordinate2->coordinates_y-next_coordinate1->coordinates_y)/(next_coordinate2->coordinates_x-next_coordinate1->coordinates_x);
                    slope2 = (prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x);
                    next_intermidiate_coordinate->coordinates_y = slope1*x + next_coordinate1->coordinates_y - slope1*next_coordinate1->coordinates_x;
                    prev_intermidiate_coordinate->coordinates_y = slope2*x + prev_coordinate1->coordinates_y - slope2*prev_coordinate1->coordinates_x;
                    prev_intermidiate_coordinate->coordinates_x = x;
                    next_intermidiate_coordinate->coordinates_x = x;
                    next_intermidiate_coordinate->next = next_coordinate2;
                    next_intermidiate_coordinate->prev = concave_point;
                    prev_intermidiate_coordinate->next = prev_coordinate2;
                    prev_intermidiate_coordinate->prev = concave_point;
                    std::cout<<"prev_intermidiate_coordinate is"<<prev_intermidiate_coordinate->coordinates_x<<","<<prev_intermidiate_coordinate->coordinates_y<<"\n"<<std::endl;  
                    std::cout<<"next_intermidiate_coordinate is"<<next_intermidiate_coordinate->coordinates_x<<","<<next_intermidiate_coordinate->coordinates_y<<"\n"<<std::endl;  
                    temp_area_node->prev_coordinate1 = prev_coordinate1_orig;
                    temp_area_node->prev_coordinate2 = prev_intermidiate_coordinate;
                    temp_area_node->next_coordinate1 = next_coordinate1_orig;
                    temp_area_node->next_coordinate2 = next_intermidiate_coordinate;
                    std::cout<<"pushed coordinate is"<<temp_area_node->next_coordinate1->coordinates_x<<","<<temp_area_node->next_coordinate1->coordinates_y<<"\n"<<std::endl;  
                    std::cout<<"pushed coordinate is"<<temp_area_node->next_coordinate2->coordinates_x<<","<<temp_area_node->next_coordinate2->coordinates_y<<"\n"<<std::endl;  
                    std::cout<<"pushed coordinate is"<<temp_area_node->prev_coordinate1->coordinates_x<<","<<temp_area_node->prev_coordinate1->coordinates_y<<"\n"<<std::endl;  
                    std::cout<<"pushed coordinate is"<<temp_area_node->prev_coordinate2->coordinates_x<<","<<temp_area_node->prev_coordinate2->coordinates_y<<"\n"<<std::endl;
                    divided_area.push_back(temp_area_node);
                    ROS_INFO("[CPP] : list of boustrophedon_area points are");
                    for(int i = 0; i<divided_area.size(); i++){
                        std::cout<<divided_area[i]->prev_coordinate1->coordinates_x<<","<<divided_area[i]->prev_coordinate1->coordinates_y<<" "<<std::endl;  
                        std::cout<<divided_area[i]->prev_coordinate2->coordinates_x<<","<<divided_area[i]->prev_coordinate2->coordinates_y<<" "<<std::endl;  
                        std::cout<<divided_area[i]->next_coordinate1->coordinates_x<<","<<divided_area[i]->next_coordinate1->coordinates_y<<" "<<std::endl;  
                        std::cout<<divided_area[i]->next_coordinate2->coordinates_x<<","<<divided_area[i]->next_coordinate2->coordinates_y<<" "<<std::endl;  
                    }
                    concave_points_iterator = concave_points.begin();
                    concave_points.erase(concave_points_iterator+i);
                    boustrophedon_area_division(next_intermidiate_coordinate, next_coordinate2, concave_point, concave_point->prev);
                    ROS_ERROR("HI NEHA M HERE");
                    boustrophedon_area_division(concave_point, concave_point->next, prev_intermidiate_coordinate, prev_coordinate2);
                    return;
                }
                else{
                    prev_coordinate1 = prev_coordinate1->prev;
                    prev_coordinate2 = prev_coordinate2->prev;
                    next_coordinate1 = next_coordinate1->next;
                    next_coordinate2 = next_coordinate2->next;
                }// i++;
                // x = concave_points[i]->coordinates_x;
            }
            else if((x<next_coordinate2->coordinates_x)&&(x>next_coordinate1->coordinates_x)){
                prev_coordinate1 = prev_coordinate1->prev;
                prev_coordinate2 = prev_coordinate2->prev;
                concavity = true;
                ROS_INFO("i here2");
            }
            else if((x<prev_coordinate2->coordinates_x)&&(x>prev_coordinate1->coordinates_x)){
            next_coordinate1 = next_coordinate1->next;
            next_coordinate2 = next_coordinate2->next;
            ROS_INFO("i here3");
            }
            else{
                prev_coordinate1 = prev_coordinate1->prev;
                prev_coordinate2 = prev_coordinate2->prev;
                next_coordinate1 = next_coordinate1->next;
                next_coordinate2 = next_coordinate2->next;
                ROS_INFO("i here4");
            }
            std::cout<<"m here"<<std::endl;
            concavity = true;
        }
    }
            //improve
    ROS_INFO("i here5");
    //last two areas to be added
    // prev_coordinate2 = prev_coordinate2->prev;
    temp_area_node->prev_coordinate1 = prev_coordinate1_orig;
    temp_area_node->prev_coordinate2 = prev_coordinate2;
    temp_area_node->next_coordinate1 = next_coordinate1_orig;
    temp_area_node->next_coordinate2 = next_coordinate2;
    divided_area.push_back(temp_area_node);
    area_division_type = true;
    ROS_INFO("[CPP] : list of boustrophedon_area points are");
    for(int i = 0; i<divided_area.size(); i++){
            std::cout<<divided_area[i]->prev_coordinate1->coordinates_x<<","<<divided_area[i]->prev_coordinate1->coordinates_y<<" "<<std::endl;  
            std::cout<<divided_area[i]->prev_coordinate2->coordinates_x<<","<<divided_area[i]->prev_coordinate2->coordinates_y<<" "<<std::endl;  
            std::cout<<divided_area[i]->next_coordinate1->coordinates_x<<","<<divided_area[i]->next_coordinate1->coordinates_y<<" "<<std::endl;  
            std::cout<<divided_area[i]->next_coordinate2->coordinates_x<<","<<divided_area[i]->next_coordinate2->coordinates_y<<" "<<std::endl;  
        }
    // coverage_planning_node_class::boustrophedon_matrix();
}
void coverage_planning_node_class::boustrophedon_matrix(){
    if(!area_division_type){
        ROS_ERROR("[CPP] : Area division did not happen right");
        return;
    }
    float total_distance, distance1, distance2, distance3, prev_length, next_length;
    std::vector<float> weighted_array;
    std::vector<std::vector<float>> weighted_matrix;
    coordinates_node* prev_coordinate1;
    coordinates_node* prev_coordinate2;
    coordinates_node* next_coordinate1;
    coordinates_node* next_coordinate2;
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