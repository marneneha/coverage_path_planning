#include<coverage_planning_node_class.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>

int coverage_planning_node_class::read(){

    string coverage_planning_area(_coverage_planning_area_);
    float data;
    ifstream coverage_planning_area_file(coverage_planning_area);
    
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
        }
        temp->next = head;
        head->prev = temp;
        coverage_planning_area_file.close();
        sorting(head, number_of_data_points);
    }
    return 0;
}

void concavity_indentifier(){
    coordinates_node* prev_coordinate = head->prev;
    coordinates_node* mid_coordinate =  head;
    coordinates_node* next_coordinate = head->next;
    concave_point[1][2];
    std::vector concave_points;
    do{
        product = (next_coordinate->coordinate_x- mid_coordinate->coordinate_x) * (mid_coordinate->coordinate_y - prev_coordinate->coordinate_x);
        if(product < 0){
            //add to concavity point vector
            concave_point[0][0] = mid_coordinate->coordinate_x;
            concave_point[0][1] = mid_coordinate->coordinate_y;
            concave_points.add(concave_point);
        }
        prev_coordinate = prev_coordinate->next;
        mid_coordinate = mid_coordinate->next;
        next_coordinate = next_coordinate->next;
    }
    while(mid_coordinate->next!=head);

    sorting(concave_points);
}

void sorting(){
    //implement in place sorting algo along x
}
//put it in include
class area_node{
    public:
    coordinates_node* prev_coordinate1;
    coordinates_node* prev_coordinate2;
    coordinates_node* next_coordinate1;
    coordinates_node* next_coordinate2;
    bool concavity = false;
}
void boustrophedon_area_division(next_coordinate1, next_coordinate2, prev_coordinate1, prev_coordinate2){
    int i = 0;
    coordinate_node* concave_point = concave_points[0];
    float x = concave_point->coordinate_x;
    std::vector<area_node> divided_area;
    area_node* temp_area_node;
    do{
        if((x<next_coordinate2->coordinate_x)&&(x>next_coordinate1->coordinate_x))&& ((x<prev_coordinate2->coordinate_x)&&(x>prev_coordinate1->coordinate_x)){
            //area division
            coordinate_node* prev_intermidiate_coordinate, next_intermidiate_coordinate;
            slope1 = (next_coordinate2->coordinate_y-next_coordinate1->coordinate_y)/(next_coordinate2->coordinate_x-next_coordinate1->coordinate_x);
            slope2 = (prev_coordinate2->coordinate_y-prev_coordinate1->coordinate_y)/(prev_coordinate2->coordinate_x-prev_coordinate1->coordinate_x);
            next_intermidiate_coordinate->coordinate_y = slope1*x + next_coordinate1->coordinate_y - slope1*next_coordinate1->coordinate_x;
            prev_intermidiate_coordinate->coordinate_y = slope2*x + prev_coordinate1->coordinate_y - slope2*prev_coordinate1->coordinate_x;
            prev_intermidiate_coordinate->coordinate_x = x;
            next_intermidiate_coordinate->coordinate_x = x;
            temp_area_node.prev_coordinate1 = prev_coordinate1;
            temp_area_node.prev_coordinate2 = prev_intermidiate_coordinate;
            temp_area_node.next_coordinate1 = next_coordinate1;
            temp_area_node.next_coordinate2 = next_intermidiate_coordinate;
            divided_area.prev_coordinate1.push_back(temp_area_node);
            boustrophedon_area_division(next_intermidiate_coordinate, next_coordinate2, concave_point, concave_point->next);
            boustrophedon_area_division(concave_point, concave_point->prev, prev_intermidiate_coordinate, prev_coordinate2);
            i++;
            x = concave_points[i]->coordinate_x;
        }
        else if((x<next_coordinate2->coordinate_x)&&(x>next_coordinate1->coordinate_x)){
            prev_coordinate1 = prev_coordinate1->next;
            prev_coordinate2 = prev_coordinate2->next;
            concavity = true;
        }
        else if((x<prev_coordinate2->coordinate_x)&&(x>prev_coordinate1->coordinate_x)){
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
void boustrophedon_matrix(){
    float total_distance, distance1, distance2, distance3;
    std::vector<float> weighted_row;
    std::vector<float, float> weighted_matrix;
    coordinate_node* prev_coordinate1, prev_coordinate2, next_coordinate1, next_coordinate2;
    for(int i = 0; i< divided_area.size(); i++){
        for(int k = 0; k<4; k++){
            prev_length = dist(prev_coordinate1, prev_coordinate2);
            next_length = dist(next_coordinate1, next_coordinate2);
            distance1 = 2*(max(prev_length, next_length)/r);
            for(int j = 0; j<divided_area.size(); j++){
                distance2 = dist(extremum(prev_coordinate2, next_coordinate2, 1), extremum(prev_coordinate1, next_coordinate1, 0));
                for(int l = 0; l<4; l++){
                    prev_length = dist(prev_coordinate1, prev_coordinate2);
                    next_length = dist(next_coordinate1, next_coordinate2);
                    distance3 = 2*(max(prev_length, next_length)/r);
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
coordinate_node* extremum (coordinate_node* prev_coordinate, coordinate_node* next_coordinate, bool action){
    if(type){
        if(prev_coordinate->coordinate_x > next_coordinate->coordinate_x){
            return prev_coordinate;
        }
        else
        return next_coordinate;
    }
    else{
        if(prev_coordinate->coordinate_x < next_coordinate->coordinate_x){
            return prev_coordinate;
        }
        else
        return next_coordinate;
    }
}
float coverage_planning_node_class::dist(coordinate_node* node1, coordinate_node* node2){
    float dist_value = sqrt(pow((node1->coordinate_x -node2->coordinate_x), 2)+pow((node1->coordinate_y -node2->coordinate_y), 2));
    return dist_value;
}
float max(float length1, float legth2){
    if(length1>= length2)
        return length1;
    else
        return legth2;
}

PLUGINLIB_EXPORT_CLASS(ns_coverage_path_node::coverage_planning_node_class, nodelet::Nodelet)