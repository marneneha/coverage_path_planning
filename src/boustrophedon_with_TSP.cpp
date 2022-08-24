#include<coverage_planning_node_class.h>
namespace ns_boustrophedon{
void coverage_planning_node_class::onInit(){
	ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
	mrs_lib::ParamLoader param_loader(nh, "coverage_planning_boustrophedon");
    param_loader.loadParam("uav_name", _uav_name_);
    coverage_planning_trajectory_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_to_follow");
    CPP_call_file = nh.advertiseService("CPP_service_call", &coverage_planning_node_class::read, this);
    // ros::ServiceServer update_map = nh.advertiseService("update_map_service", &coverage_planning_node_class::update_map, this);
    curent_area_finder = nh.advertiseService("curent_area_service", &coverage_planning_node_class::curent_area_finder, this);
    ground_waypoint_vector_sub = nh.subscribe<coverage_planning::WaypointVector>("waypoint_vector_topic", 1, &coverage_planning_node_class::updateMap, this);
    std::cout<<"INSIDE BOUSTROPHEDON"<<std::endl;
    ros::spin();
}
bool coverage_planning_node_class::read(coverage_planning::CPPServiceCall::Request& Pathreq, coverage_planning::CPPServiceCall::Response& Pathres){
    // points are read in clockwise format
    std::string _coverage_planning_area_ = Pathreq.file_name;
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
// head
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
    coordinates_node* next_coordinate1_orig = new coordinates_node();
    coordinates_node* prev_coordinate1_orig = new coordinates_node();
    next_coordinate1_orig->coordinates_x = concave_point->coordinates_x;
    next_coordinate1_orig->coordinates_y = concave_point->coordinates_y;
    next_coordinate1_orig->prev = prev_coordinate1_orig;
    next_coordinate1_orig->next = concave_point->next;
    prev_coordinate1_orig->coordinates_x = concave_point->coordinates_x;
    prev_coordinate1_orig->coordinates_y = concave_point->coordinates_y;
    prev_coordinate1_orig->prev = concave_point->prev;
    prev_coordinate1_orig->next = next_coordinate1_orig;
    boustrophedon_area_division(next_coordinate1_orig, concave_point->next, prev_coordinate1_orig, concave_point->prev);
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
    coordinates_node* next_coordinate2;  
    coordinates_node* prev_intermidiate_coordinate;
    coordinates_node* next_intermidiate_coordinate;
    coordinates_node* prev_intermidiate_coordinate_continue;
    coordinates_node* next_intermidiate_coordinate_continue;
    coordinates_node* next_concave_point;
    for(int i=0; i<concave_points.size(); i++){
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
                    prev_intermidiate_coordinate_continue = new coordinates_node();
                    next_intermidiate_coordinate_continue = new coordinates_node();
                    next_concave_point = new coordinates_node();
                    slope1 = (next_coordinate2->coordinates_y-next_coordinate1->coordinates_y)/(next_coordinate2->coordinates_x-next_coordinate1->coordinates_x);
                    slope2 = (prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x);
                    next_intermidiate_coordinate->coordinates_y = slope1*x + next_coordinate1->coordinates_y - slope1*next_coordinate1->coordinates_x;
                    prev_intermidiate_coordinate->coordinates_y = slope2*x + prev_coordinate1->coordinates_y - slope2*prev_coordinate1->coordinates_x;
                    prev_intermidiate_coordinate->coordinates_x = x;
                    next_intermidiate_coordinate->coordinates_x = x;
                    next_intermidiate_coordinate->next = prev_intermidiate_coordinate;
                    next_intermidiate_coordinate->prev = next_coordinate1;
                    prev_intermidiate_coordinate->next = prev_coordinate1;
                    prev_intermidiate_coordinate->prev = next_intermidiate_coordinate;
                    prev_coordinate1->next = prev_intermidiate_coordinate;
                    next_coordinate1->next = next_intermidiate_coordinate;

                    next_intermidiate_coordinate_continue->coordinates_y = slope1*x + next_coordinate1->coordinates_y - slope1*next_coordinate1->coordinates_x;
                    prev_intermidiate_coordinate_continue->coordinates_y = slope2*x + prev_coordinate1->coordinates_y - slope2*prev_coordinate1->coordinates_x;
                    prev_intermidiate_coordinate_continue->coordinates_x = x;
                    next_intermidiate_coordinate_continue->coordinates_x = x;

                    std::cout<<"prev_intermidiate_coordinate is"<<prev_intermidiate_coordinate->coordinates_x<<","<<prev_intermidiate_coordinate->coordinates_y<<"\n"<<std::endl;  
                    std::cout<<"next_intermidiate_coordinate is"<<next_intermidiate_coordinate->coordinates_x<<","<<next_intermidiate_coordinate->coordinates_y<<"\n"<<std::endl;  
                    std::cout<<"previous of next_intermidiate_coordinate is"<<next_coordinate1_orig->prev->coordinates_x<<","<<next_coordinate1_orig->prev->coordinates_y<<"\n"<<std::endl;  
                    temp_area_node->prev_coordinate1 = prev_coordinate1_orig;
                    temp_area_node->prev_coordinate2 = prev_intermidiate_coordinate;
                    temp_area_node->next_coordinate1 = next_coordinate1_orig;
                    temp_area_node->next_coordinate2 = next_intermidiate_coordinate;
                    cycle_area_node(temp_area_node);
                    divided_area.push_back(temp_area_node);
                    next_concave_point->coordinates_x = concave_point->coordinates_x;
                    next_concave_point->coordinates_y = concave_point->coordinates_y;
                    next_concave_point->prev = concave_point->prev;
                    next_concave_point->next = concave_point->next;
                    ROS_INFO("[CPP] : list of boustrophedon_area points are");
                    for(int i = 0; i<divided_area.size(); i++){
                        std::cout<<divided_area[i]->prev_coordinate1->coordinates_x<<","<<divided_area[i]->prev_coordinate1->coordinates_y<<" "<<std::endl;  
                        std::cout<<divided_area[i]->prev_coordinate2->coordinates_x<<","<<divided_area[i]->prev_coordinate2->coordinates_y<<" "<<std::endl;  
                        std::cout<<divided_area[i]->next_coordinate1->coordinates_x<<","<<divided_area[i]->next_coordinate1->coordinates_y<<" "<<std::endl;  
                        std::cout<<divided_area[i]->next_coordinate2->coordinates_x<<","<<divided_area[i]->next_coordinate2->coordinates_y<<" "<<std::endl;  
                    }
                    boustrophedon_area_division(next_intermidiate_coordinate_continue, next_coordinate2, concave_point, concave_point->prev);
                    ROS_ERROR("HI NEHA M HERE");
                    boustrophedon_area_division(next_concave_point, next_concave_point->next, prev_intermidiate_coordinate_continue, prev_coordinate2);

                    concave_points_iterator = concave_points.begin();
                    concave_points.erase(concave_points_iterator+i);
                    if(concave_points.size() == 0){
                        area_division_type = true;
                    }
                    coverage_planning_node_class::boustrophedon_matrix();
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
                temp_area_node->concavity = true;
                ROS_INFO("i here2");
            }
            else if((x<prev_coordinate2->coordinates_x)&&(x>prev_coordinate1->coordinates_x)){
                next_coordinate1 = next_coordinate1->next;
                next_coordinate2 = next_coordinate2->next;
                temp_area_node->concavity = true;
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
    cycle_area_node(temp_area_node);
    std::cout<<"pushed coordinate is"<<temp_area_node->next_coordinate1->prev->coordinates_x<<","<<temp_area_node->next_coordinate1->prev->coordinates_y<<"\n"<<std::endl;  
    std::cout<<"pushed coordinate is"<<temp_area_node->next_coordinate2->prev->coordinates_x<<","<<temp_area_node->next_coordinate2->prev->coordinates_y<<"\n"<<std::endl;  
    std::cout<<"pushed coordinate is"<<temp_area_node->prev_coordinate1->prev->coordinates_x<<","<<temp_area_node->prev_coordinate1->prev->coordinates_y<<"\n"<<std::endl;  
    std::cout<<"pushed coordinate is"<<temp_area_node->prev_coordinate2->prev->coordinates_x<<","<<temp_area_node->prev_coordinate2->prev->coordinates_y<<"\n"<<std::endl;
    std::cout<<"prev pushed coordinate is"<<temp_area_node->next_coordinate1->prev->prev->coordinates_x<<","<<temp_area_node->next_coordinate1->prev->prev->coordinates_y<<"\n"<<std::endl;  
    std::cout<<"prev pushed coordinate is"<<temp_area_node->next_coordinate2->prev->prev->coordinates_x<<","<<temp_area_node->next_coordinate2->prev->prev->coordinates_y<<"\n"<<std::endl;  
    std::cout<<"prev pushed coordinate is"<<temp_area_node->prev_coordinate1->prev->prev->coordinates_x<<","<<temp_area_node->prev_coordinate1->prev->prev->coordinates_y<<"\n"<<std::endl;  
    std::cout<<"prev pushed coordinate is"<<temp_area_node->prev_coordinate2->prev->prev->coordinates_x<<","<<temp_area_node->prev_coordinate2->prev->prev->coordinates_y<<"\n"<<std::endl;
    divided_area.push_back(temp_area_node);
    ROS_INFO("[CPP] : list of boustrophedon_area points are");
    for(int i = 0; i<divided_area.size(); i++){
            std::cout<<divided_area[i]->prev_coordinate1->coordinates_x<<","<<divided_area[i]->prev_coordinate1->coordinates_y<<" "<<std::endl;  
            std::cout<<divided_area[i]->prev_coordinate2->coordinates_x<<","<<divided_area[i]->prev_coordinate2->coordinates_y<<" "<<std::endl;  
            std::cout<<divided_area[i]->next_coordinate1->coordinates_x<<","<<divided_area[i]->next_coordinate1->coordinates_y<<" "<<std::endl;  
            std::cout<<divided_area[i]->next_coordinate2->coordinates_x<<","<<divided_area[i]->next_coordinate2->coordinates_y<<" "<<std::endl;  
            std::cout<<"\n"<<std::endl;
        }
}
void coverage_planning_node_class::boustrophedon_matrix(){
    if(!area_division_type){
        ROS_ERROR("[CPP] : Area division did not happen right");
        return;
    }
    float total_distance, distance1, distance2, distance3, prev_length, next_length;
    std::vector<float> weighted_array;
    std::vector<std::vector<float>> weighted_matrix;
    coordinates_node* prev_coordinate1_node1;
    coordinates_node* prev_coordinate2_node1;
    coordinates_node* next_coordinate1_node1;
    coordinates_node* next_coordinate2_node1;
    coordinates_node* prev_coordinate1_node2;
    coordinates_node* prev_coordinate2_node2;
    coordinates_node* next_coordinate1_node2;
    coordinates_node* next_coordinate2_node2;
        for(int i = 0; i< divided_area.size(); i++){
        prev_coordinate1_node1 = divided_area[i]->prev_coordinate1;
        prev_coordinate2_node1 = divided_area[i]->prev_coordinate2;
        next_coordinate1_node1 = divided_area[i]->next_coordinate1;
        next_coordinate2_node1 = divided_area[i]->next_coordinate2;
        for(int k = 0; k<4; k++){
            // std::cout<<"next_coordinate1_node1 is"<<next_coordinate1_node1->coordinates_x<<","<<next_coordinate1_node1->coordinates_y<<"\n"<<std::endl;  
            // std::cout<<"next_coordinate2_node1 is"<<next_coordinate2_node1->coordinates_x<<","<<next_coordinate2_node1->coordinates_y<<"\n"<<std::endl;  
            // std::cout<<"prev_coordinate1_node1 is"<<prev_coordinate1_node1->coordinates_x<<","<<prev_coordinate1_node1->coordinates_y<<"\n"<<std::endl;  
            // std::cout<<"prev_coordinate2_node1 is"<<prev_coordinate2_node1->coordinates_x<<","<<prev_coordinate2_node1->coordinates_y<<"\n"<<std::endl;
            prev_length = coverage_planning_node_class::dist(prev_coordinate1_node1, prev_coordinate2_node1);
            next_length = coverage_planning_node_class::dist(next_coordinate1_node1, next_coordinate2_node1);
            distance1 = 2*(coverage_planning_node_class::max(prev_length, next_length)/sweeping_dist);
            for(int j = 0; j<divided_area.size(); j++){
                prev_coordinate1_node2 = divided_area[j]->prev_coordinate1;
                prev_coordinate2_node2 = divided_area[j]->prev_coordinate2;
                next_coordinate1_node2 = divided_area[j]->next_coordinate1;
                next_coordinate2_node2 = divided_area[j]->next_coordinate2;
                for(int l = 0; l<4; l++){
                    // std::cout<<"i is"<<i<<"k is"<<k<<"j is"<<j<<"l is"<<l<<std::endl;
                    // std::cout<<"next_coordinate1_node2 is"<<next_coordinate1_node2->coordinates_x<<","<<next_coordinate1_node2->coordinates_y<<"\n"<<std::endl;  
                    // std::cout<<"next_coordinate2_node2 is"<<next_coordinate2_node2->coordinates_x<<","<<next_coordinate2_node2->coordinates_y<<"\n"<<std::endl;  
                    // std::cout<<"prev_coordinate1_node2 is"<<prev_coordinate1_node2->coordinates_x<<","<<prev_coordinate1_node2->coordinates_y<<"\n"<<std::endl;  
                    // std::cout<<"prev_coordinate2_node2 is"<<prev_coordinate2_node2->coordinates_x<<","<<prev_coordinate2_node2->coordinates_y<<"\n"<<std::endl;
                    if(i == j){
                        total_distance = 1000;
                        weighted_array.push_back(total_distance);
                        // std::cout<<"i is"<<i<<"j is"<<j<<"k is"<<k<<"total distance is"<<total_distance<<std::endl;
                        printf("%f  " ,total_distance);
                    }
                    else{
                        distance2 = coverage_planning_node_class::dist(extremum(prev_coordinate2_node1, next_coordinate2_node1, 1), extremum(prev_coordinate1_node2, next_coordinate1_node2, 0));
                        prev_length = coverage_planning_node_class::dist(prev_coordinate1_node2, prev_coordinate2_node2);
                        next_length = coverage_planning_node_class::dist(next_coordinate1_node2, next_coordinate2_node2);
                        distance3 = 2*(coverage_planning_node_class::max(prev_length, next_length)/sweeping_dist);
                        total_distance = distance1 + distance2 + distance3;
                        weighted_array.push_back(total_distance);
                        // std::cout<<"i is"<<i<<"j is"<<j<<"k is"<<k<<"l is"<<l<<"total distance is"<<total_distance<<std::endl;
                        printf("%f  " ,total_distance);
                    }
                    if(divided_area[j]->concavity){
                        l++;
                        prev_coordinate1_node2 = prev_coordinate1_node2->prev;
                        prev_coordinate2_node2 = prev_coordinate2_node2->prev;
                        next_coordinate1_node2 = next_coordinate1_node2->prev;
                        next_coordinate2_node2 = next_coordinate2_node2->prev;
                    }
                    prev_coordinate1_node2 = prev_coordinate1_node2->prev;
                    prev_coordinate2_node2 = prev_coordinate2_node2->prev;
                    next_coordinate1_node2 = next_coordinate1_node2->prev;
                    next_coordinate2_node2 = next_coordinate2_node2->prev;                 
                }
            }
            weighted_matrix.push_back(weighted_array);
            std::cout<<"\n"<<std::endl;
            if(divided_area[i]->concavity){
                k++;
                prev_coordinate1_node1 = prev_coordinate1_node1->prev;
                prev_coordinate2_node1 = prev_coordinate2_node1->prev;
                next_coordinate1_node1 = next_coordinate1_node1->prev;
                next_coordinate2_node1 = next_coordinate2_node1->prev;
            }
                prev_coordinate1_node1 = prev_coordinate1_node1->prev;
                prev_coordinate2_node1 = prev_coordinate2_node1->prev;
                next_coordinate1_node1 = next_coordinate1_node1->prev;
                next_coordinate2_node1 = next_coordinate2_node1->prev;
         }
        
    }
}
//node_read (read from the file tours what is the order for sweeping)
//trajectory planner for sweeping the area this trajectory will be sent to trajectory planner servis 
void coverage_planning_node_class::cycle_area_node(area_node* temp_area_node){
    if(temp_area_node->prev_coordinate2 == temp_area_node->next_coordinate2){
        temp_area_node->prev_coordinate2 = new coordinates_node();
        temp_area_node->prev_coordinate2->coordinates_x = temp_area_node->next_coordinate2->coordinates_x;
        temp_area_node->prev_coordinate2->coordinates_y = temp_area_node->next_coordinate2->coordinates_y;
    }
    temp_area_node->prev_coordinate1->prev = temp_area_node->prev_coordinate2;
    temp_area_node->prev_coordinate1->next = temp_area_node->next_coordinate1;
    temp_area_node->prev_coordinate2->prev = temp_area_node->next_coordinate2;
    temp_area_node->prev_coordinate2->next = temp_area_node->prev_coordinate1;
    temp_area_node->next_coordinate1->prev = temp_area_node->prev_coordinate1;
    temp_area_node->next_coordinate1->next = temp_area_node->next_coordinate2;
    temp_area_node->next_coordinate2->prev = temp_area_node->next_coordinate1;
    temp_area_node->next_coordinate2->next = temp_area_node->prev_coordinate2;

    // std::cout<<"pushed coordinate is"<<temp_area_node->next_coordinate1->coordinates_x<<","<<temp_area_node->next_coordinate1->coordinates_y<<"\n"<<std::endl;  
    // std::cout<<"pushed coordinate is"<<temp_area_node->next_coordinate2->coordinates_x<<","<<temp_area_node->next_coordinate2->coordinates_y<<"\n"<<std::endl;  
    // std::cout<<"pushed coordinate is"<<temp_area_node->prev_coordinate1->coordinates_x<<","<<temp_area_node->prev_coordinate1->coordinates_y<<"\n"<<std::endl;  
    // std::cout<<"pushed coordinate is"<<temp_area_node->prev_coordinate2->coordinates_x<<","<<temp_area_node->prev_coordinate2->coordinates_y<<"\n"<<std::endl;
}
void coverage_planning_node_class::trajectory_planner(coordinates_node* side_coordinate1, coordinates_node* side_coordinate2){
    std::vector<std::vector<float>> trajectory;
    std::vector<float> trajectory_coordinate;
    float slope1 = (side_coordinate2->coordinates_y-side_coordinate1->coordinates_y)/(side_coordinate2->coordinates_x-side_coordinate1->coordinates_x);
    float C;
    if(!(1/slope1)){
        C = side_coordinate1->coordinates_x;
    }
    else
        C = side_coordinate1->coordinates_y-(slope1*side_coordinate1->coordinates_x);
    //cout<<"C is"<<C<<endl;
    coordinates_node* prev_coordinate1 = side_coordinate1; 
    coordinates_node* next_coordinate1 = side_coordinate2;
    coordinates_node* prev_coordinate2 = side_coordinate1->prev;
    coordinates_node* next_coordinate2 = side_coordinate2->next;
    std::cout<<"prev_coordinate1 are"<<prev_coordinate1->coordinates_x<<","<<prev_coordinate1->coordinates_y<<std::endl;
    //cout<<"prev_coordinate2 are"<<prev_coordinate2->coordinates_x<<","<<prev_coordinate2->coordinates_y<<endl;
    //cout<<"next_coordinate1 are"<<next_coordinate1->coordinates_x<<","<<next_coordinate1->coordinates_y<<endl;
    //cout<<"next_coordinate1 are"<<next_coordinate2->coordinates_x<<","<<next_coordinate2->coordinates_y<<endl;

    float x= prev_coordinate1->coordinates_x, y = prev_coordinate1->coordinates_y, r = 0.5;
    trajectory_coordinate.push_back(x);
    trajectory_coordinate.push_back(y);
    trajectory.push_back(trajectory_coordinate);    
    int i = 0;
    while(i<20 && prev_coordinate2->next!= next_coordinate2){
        //main lines added in alternative fashion
        if(i%2==0){
            float slope2 = (next_coordinate2->coordinates_y-next_coordinate1->coordinates_y)/(next_coordinate2->coordinates_x-next_coordinate1->coordinates_x);
            //cout<<"m here 1"<<"slope2 is"<<slope2<<"slope1 is"<<slope1<<endl;
            if(!(1/slope2)){
                //cout<<"m here"<<endl;
                x = next_coordinate1->coordinates_x;
                y = slope1*x+C;
            }
            else if(!(1/slope1)){
                //cout<<"m here 1"<<endl;
                x = C;
                y = (slope2*x) -(slope2*next_coordinate1->coordinates_x) + (next_coordinate1->coordinates_y);
                //cout<<"values are"<<slope2*x<<"and"<<slope2*next_coordinate1->coordinates_x<<"and"<<next_coordinate1->coordinates_y<<endl;
            }
            else{
                //cout<<"m here 2"<<endl;
                x = (slope2*next_coordinate1->coordinates_x+C-next_coordinate1->coordinates_y)/(slope2-slope1);
                y = slope1*x+C;
            }
            if(slope1<=1){
                if(y>(next_coordinate2->coordinates_y)){
                    next_coordinate1 = next_coordinate2;
                    next_coordinate2 = next_coordinate2->next;
                }
                else{
                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }
                    //line perpendicular to base line    
                    float theta =  atan(1/slope1);
                    x = x + ((prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/abs(prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y))*r*cos(theta);
                    y = y + ((prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/abs(prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y))*r*sin(theta);

                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }
                    i++;
                    //cout<<"after perpendicular extension x is"<<x<<"y is"<<y<<endl;
                }
                //cout<<"need to worry"<<endl;
                C = C + ((prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/abs(prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y))*r*(sqrt(1+slope1*slope1));
            }
            else{
                if(x<(next_coordinate2->coordinates_x)){
                        next_coordinate1 = next_coordinate2;
                        next_coordinate2 = next_coordinate2->next;
                    }
                else{
                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }
                        //line perpendicular to base line    
                        float theta =  atan(1/slope1);
                        x = x + ((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*cos(theta);
                        y = y + ((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*sin(theta);

                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }                        i++;
                        //cout<<"after perpendicular extension x is"<<x<<"y is"<<y<<endl;
                    }
                if(!(1/slope1))
                    C = C + ((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r;
                else
                    C = C + -1*((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*(sqrt(1+slope1*slope1));
                //cout<<"NEED NOT TO WORRY"<<"C is"<<C<<"added value is"<<((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*sqrt(1+slope1*slope1)<<endl;

            }
            //cout<<"1st is"<<"x is"<<x<<"y is"<<y<<endl;                      
        }
        else{
            float slope2 = (prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x);
            //cout<<"prev_coordinate1 is"<<prev_coordinate1->coordinates_x<<","<<prev_coordinate1->coordinates_y<<"prev_coordinate2 is"<<prev_coordinate2->coordinates_x<<","<<prev_coordinate2->coordinates_y<<endl;
            //cout<<"m here 2"<<"slope2 is"<<slope2<<"slope1 is"<<slope1<<endl;
            if(!(1/slope2)){
                //cout<<"m here"<<endl;
                x = prev_coordinate1->coordinates_x;
                y = slope1*x+C;
            }
            else if(!(1/slope1)){
                //cout<<"m here 1"<<endl;
                x = C;
                y = (slope2*x) - (slope2*prev_coordinate1->coordinates_x) + (prev_coordinate1->coordinates_y);
                //cout<<"values are"<<slope2*x<<"and"<<slope2*prev_coordinate1->coordinates_x<<"and"<<prev_coordinate1->coordinates_y<<endl;
            }
            else{
                //cout<<"m here 2"<<endl;
                x = (slope2*prev_coordinate1->coordinates_x+C-prev_coordinate1->coordinates_y)/(slope2-slope1);
                y = slope1*x+C;
            }          
            if(slope1<=1){
                if(y>(prev_coordinate2->coordinates_y)){
                    prev_coordinate1 = prev_coordinate2;
                    prev_coordinate2 = prev_coordinate2->prev;
                }
                else{
                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }                    //cout<<"2nd is"<<"x is"<<x<<"y is"<<y<<"slope1 is"<<slope1<<"slope2 is"<<slope2<<endl;
                    //line perpendicular to base line    
                    float theta =  atan(1/slope1);
                    x = x + ((prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/abs(prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y))*r*cos(theta);
                    y = y + ((prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/abs(prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y))*r*sin(theta);
                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }                    i++;
                    //cout<<"after perpendicular extension x is"<<x<<"y is"<<y<<endl;
                }
                //cout<<"need to worry"<<endl;
                C = C + ((prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/abs(prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y))*r*(sqrt(1+slope1*slope1));

            }
            else{
                if(x<(prev_coordinate2->coordinates_x)){
                    prev_coordinate1 = prev_coordinate2;
                    prev_coordinate2 = prev_coordinate2->prev;
                }
                else{
                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }                    //line perpendicular to base line    
                    float theta =  atan(1/slope1);
                    x = x + ((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*cos(theta);
                    y = y + ((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*sin(theta);
                    if(x<100 &&y< 100){
                        trajectory_coordinate[0] = x;
                        trajectory_coordinate[1] = y;
                        trajectory.push_back(trajectory_coordinate);
                    }                    i++;
                    //cout<<"after perpendicular extension x is"<<x<<"y is"<<y<<endl;
                }
                if(!(1/slope1))
                    C = C + ((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r;
                else
                    C = C + -1*((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*(sqrt(1+slope1*slope1));
                //cout<<"NEED NOT TO WORRY"<<"C is"<<C<<"added value is"<<((prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x)/abs(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x))*r*sqrt(1+slope1*slope1)<<endl;

            }

        }
    }
    mrs_msgs::PathSrv::Request              req;
    mrs_msgs::PathSrv::Response             res;
    for(int i = 0; i < trajectory.size(); i++){
        std::cout<<"x is"<<trajectory[i][0]<<"y is"<<trajectory[i][1]<<std::endl;
        waypoint.position.x = trajectory[i][0];
        waypoint.position.y = trajectory[i][1];
        waypoint.position.z = 5;
        waypoint.heading = 0;
        req.path.points.push_back(waypoint);
    }
    trajectory.clear();
    req.path.header.seq = 0;
    //req.path.header.time_stamp = ros::time.now();
    req.path.header.frame_id = "gps_origin";
    req.path.use_heading = false;
    req.path.fly_now = true;
    req.path.stop_at_waypoints = false;
    req.path.loop = false;
    req.path.override_constraints = false;
    req.path.relax_heading = true;
    coverage_planning_trajectory_service_client.call(req, res);
}
bool coverage_planning_node_class::update_map(const coverage_planning::WaypointVector::ConstPtr& msg){
    // if(image_classified != true){
    //     return;
    // }
        ground_waypoint = msg->ground_waypoint_vector[i];
        Eigen::Vector3d first_waypoint_to_next_boundry = 
    if(first_waypoint_to_next_boundry.norm()<abselon && next_boundary != true){
        next_boundary = true;
    }
    if(first_waypoint_to_prev_boundry.norm()<abselon && prev_boundary != true){
        prev_boundary = true;
    }
    else if(no_fly_zone != true){
        no_fly_zone = true;
    }

    if(next_boundary){
        if(ground_waypoint.position.x > next_coordinate2->coordinates_x){
            next_coordinate2 = next_coordinate2->next;
        }
        if(last_point_to_current_point.norm()abselon){
            visited_waypoint_vector.push_back(ground_waypoint);
        }
        if(first_waypoint_to_next_boundry.norm()<abselon){
            for(int i = 1; i< visited_waypoint_vector.size(); i++){
                coordinates_node* temp_waypoint1 = new coordinates_node();
                temp_waypoint->coordinates_x = visited_waypoint_vector[i].position.x;
                temp_waypoint->coordinates_y = visited_waypoint_vector[i].position.y;
                temp_waypoint->coordinates_z = visited_waypoint_vector[i].position.z;
                temp_waypoint1->prev = temp_waypoint;
                temp_waypoint->next = temp_waypoint1;
                temp_waypoint = temp_waypoint1;
            }
            temp_waypoint->next = next_coordinate2;
            next_coordinate2->prev = temp_waypoint;
            next_boundary = false;
            coverage_planning_node_class::concavity_indentifier();
        }
    }

    if(prev_boundary){
        if(ground_waypoint.position.x > prev_coordinate2->coordinates_x){
            prev_coordinate2 = prev_coordinate2->prev;
        }        
        if(last_point_to_current_point.norm()abselon){
            visited_waypoint_vector.push_back(ground_waypoint);
        }
        if(first_waypoint_to_prev_boundry.norm()<abselon){
            for(int i = 1; i< visited_waypoint_vector.size(); i++){
                coordinates_node* temp_waypoint1 = new coordinates_node();
                temp_waypoint->coordinates_x = visited_waypoint_vector[i].position.x;
                temp_waypoint->coordinates_y = visited_waypoint_vector[i].position.y;
                temp_waypoint->coordinates_z = visited_waypoint_vector[i].position.z;
                temp_waypoint1->prev = temp_waypoint;
                temp_waypoint->next = temp_waypoint1;
                temp_waypoint = temp_waypoint1;
            }
            temp_waypoint->next = next_coordinate2;
            next_coordinate2->prev = temp_waypoint;
            prev_boundary = false;
            coverage_planning_node_class::concavity_indentifier();
        }
    }

    if(no_fly_zone){

    }








    mrs_msgs::Reference ground_waypoint;
    std::vector<mrs_msgs::Reference> visited_waypoint_vector;
    for(int i=0; i<msg->ground_waypoint_vector.size(); i++){
        ground_waypoint = msg->ground_waypoint_vector[i];
        if(ground_waypoint.position.x > next_coordinate2->coordinates_x){
            next_coordinate2 = next_coordinate2->next;
        }
        if(ground_waypoint.position.x > prev_coordinate2->coordinates_x){
            prev_coordinate2 = prev_coordinate2->prev;
        }

    }
            std::cout<<"inside update_map"<<std::endl;
            coordinates_node* next_coordinate1;
            coordinates_node* next_coordinate2;
            coordinates_node* temp_waypoint = new coordinates_node();
            temp_waypoint->coordinates_x = UpdateMapReq.visited_waypoints[0].x;
            temp_waypoint->coordinates_y = UpdateMapReq.visited_waypoints[0].y;
            next_coordinate1->next = temp_waypoint;
            temp_waypoint->prev = next_coordinate1;
    if(UpdateMapReq.NoFlyZone){
        for(int i = 1; i< UpdateMapReq.visited_waypoints.size(); i++){
            NoFlyZoneVector.push_back(UpdateMapReq.visited_waypoints[i]);
        }
    }
    else{
        for(int i = 1; i< UpdateMapReq.visited_waypoints.size(); i++){
            coordinates_node* temp_waypoint1 = new coordinates_node();
            temp_waypoint->coordinates_x = UpdateMapReq.visited_waypoints[i].x;
            temp_waypoint->coordinates_y = UpdateMapReq.visited_waypoints[i].y;
            temp_waypoint1->prev = temp_waypoint;
            temp_waypoint->next = temp_waypoint1;
            temp_waypoint = temp_waypoint1;
        }
        temp_waypoint->next = next_coordinate2;
        next_coordinate2->prev = temp_waypoint;
        coverage_planning_node_class::concavity_indentifier();
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
bool coverage_planning_node_class::curent_area_finder(coverage_planning::UpdateMap::Request& UpdateMapReq,  coverage_planning::UpdateMap::Response& UpdateMapRes){
}
}

PLUGINLIB_EXPORT_CLASS(ns_boustrophedon::coverage_planning_node_class, nodelet::Nodelet)