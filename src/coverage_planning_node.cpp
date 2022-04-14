#include <iostream>
#include <fstream>
#include <string>
#include <bits/stdc++.h>

using namespace std;
class coordinates_node{
    public:
    float coordinates_x;
    float coordinates_y;
    coordinates_node* next;
    coordinates_node* prev;
};
coordinates_node* head = new coordinates_node();
coordinates_node* temp1;
coordinates_node* data_iterator;
//here assumed coordinates cant be 3d
//look for memory deletion after use otherwise it will just keep adding to previous vector
int read(){

    string coverage_planning_area("/home/neha/coverage_planning_area.txt");
    float data;
    ifstream coverage_planning_area_file(coverage_planning_area);
    
    if(coverage_planning_area_file.is_open()) {
        coordinates_node* temp = head;
        while (coverage_planning_area_file >> data)
        {
            temp->coordinates_x = data;
            coverage_planning_area_file >> data;
            temp->coordinates_y = data;
            temp1 = new coordinates_node();
            temp->next = temp1;
            temp1->prev = temp;
            temp = temp1;
            temp->next = head;
            head->prev = temp;

        }
        coverage_planning_area_file.close();
    }
    data_iterator = head;
    while ( data_iterator->next!=head)
    {
        cout<<data_iterator->coordinates_x<<","<<data_iterator->coordinates_y<<endl;  
        data_iterator = data_iterator->next;
    }
    return 0;
}
float dist (float x1, float y1, float x2, float y2){
    float dist_value = sqrt(pow((x1-x2), 2)+pow((y1-y2), 2));
    return dist_value;
}

void coordinate_finder(){
    float side_max=0;
    //select the side for moving
    coordinates_node* iterator = head;
    coordinates_node* side_coordinate1;
    coordinates_node* side_coordinate2;
    while(iterator->next!=head) {
        float side_length = dist(iterator->coordinates_x, iterator->coordinates_y, (iterator->next)->coordinates_x, (iterator->next)->coordinates_y);
        if(side_length>side_max){
            side_max = side_length;
            side_coordinate1 = iterator;
            side_coordinate1 = iterator->next;
        }

    }
    //find coordinates along same direction
    
}

void trajectory_planner(coordinates_node* side_coordinate1, coordinates_node* side_coordinate2){
    vector<vector<float>> trajectory;
    vector<float> trajectory_coordinate;
    float slope1 = (side_coordinate2->coordinates_y-side_coordinate1->coordinates_y)/(side_coordinate2->coordinates_x-side_coordinate1->coordinates_x);
    coordinates_node* prev_coordinate1 = side_coordinate1; 
    coordinates_node* prev_coordinate2 = side_coordinate2;
    coordinates_node* next_coordinate1 = side_coordinate1->prev;
    coordinates_node* next_coordinate2 = side_coordinate2->next;
    float x= prev_coordinate1->coordinates_x, y = prev_coordinate1->coordinates_y;
    int i = 0;   
    while(prev_coordinate2!= next_coordinate2){
        
        if(i%2==0){
            float slope2 = (next_coordinate2->coordinates_y-next_coordinate1->coordinates_y)/(next_coordinate2->coordinates_x-next_coordinate2->coordinates_x);
            x = (slope2*next_coordinate1->coordinates_x-next_coordinate2->coordinates_y)/(slope2-slope1);
        }
        else{
            float slope2 = (prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x);
            x = (slope2*prev_coordinate1->coordinates_x-prev_coordinate1->coordinates_y)/(slope2-slope1);
        }
        
        y = slope1*x;        
        if( y>prev_coordinate2->coordinates_y){
            prev_coordinate1 = prev_coordinate2;
            prev_coordinate2 = prev_coordinate2->prev;
        }
        if(y>next_coordinate2->coordinates_y){
            next_coordinate1 = next_coordinate2;
            next_coordinate2 = next_coordinate2->next;
        }
        
        else
        {
            trajectory_coordinate[0] = x;
            trajectory_coordinate[1] = y;
            trajectory.push_back(trajectory_coordinate);
        }

        for (int i = 0; i < trajectory.size(); i++)
        {
            cout<<trajectory[i][0]<<",";
            cout<<trajectory[i][1];
            cout<<endl;
        }
        
    }
        
}

int main(){
    read();
    //coordinate_finder();
    //trajectory_planner();
}