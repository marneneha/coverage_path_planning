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
            head->prev = temp;
            temp = temp1;
            temp->next = head;

        }
        coverage_planning_area_file.close();
    }
    data_iterator = head;
    return 0;
}
float dist (float x1, float y1, float x2, float y2){
    float dist_value = sqrt(pow((x1-x2), 2)+pow((y1-y2), 2));
    return dist_value;
}

void trajectory_planner(coordinates_node* side_coordinate1, coordinates_node* side_coordinate2){
    vector<vector<float>> trajectory;
    vector<float> trajectory_coordinate;
    float slope1 = (side_coordinate2->coordinates_y-side_coordinate1->coordinates_y)/(side_coordinate2->coordinates_x-side_coordinate1->coordinates_x);
    float C;
    if(!(1/slope1)){
        C = side_coordinate1->coordinates_x;
    }
    else
        C = side_coordinate1->coordinates_y-(slope1*side_coordinate1->coordinates_x);
    coordinates_node* prev_coordinate1 = side_coordinate1; 
    coordinates_node* next_coordinate1 = side_coordinate2;
    coordinates_node* prev_coordinate2 = side_coordinate1->prev;
    coordinates_node* next_coordinate2 = side_coordinate2->next;
    cout<<"coordinates are"<<endl;
    cout<<prev_coordinate2->coordinates_x<<","<<prev_coordinate2->coordinates_y<<endl;
    cout<<prev_coordinate1->coordinates_x<<","<<prev_coordinate1->coordinates_y<<endl;
    cout<<next_coordinate1->coordinates_x<<","<<next_coordinate1->coordinates_y<<endl;
    cout<<next_coordinate2->coordinates_x<<","<<next_coordinate2->coordinates_y<<endl;

    float x= prev_coordinate1->coordinates_x, y = prev_coordinate1->coordinates_y, r = 0.5;
    trajectory_coordinate.push_back(x);
    trajectory_coordinate.push_back(y);
    trajectory.push_back(trajectory_coordinate);    
    int i = 0;  
    while(i<10 && prev_coordinate2->next!= next_coordinate2){
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
                //cout<<"values are"<<slope2*x<<"and"<<slope2*next_coordinate1->coordinates_x<<"and"<<next_coordinate1->coordinates_y;
            }
            else{
                //cout<<"m here 2"<<endl;
                x = (slope2*next_coordinate1->coordinates_x+C-next_coordinate1->coordinates_y)/(slope2-slope1);
                y = slope1*x+C;
            }
                      
            cout<<"1st is"<<"x is"<<x<<"y is"<<y<<endl;
        }
        else{
            float slope2 = (prev_coordinate2->coordinates_y-prev_coordinate1->coordinates_y)/(prev_coordinate2->coordinates_x-prev_coordinate1->coordinates_x);
            cout<<"prev_coordinate1 is"<<prev_coordinate1->coordinates_x<<","<<prev_coordinate1->coordinates_y<<"prev_coordinate2 is"<<prev_coordinate2->coordinates_x<<","<<prev_coordinate2->coordinates_y<<endl;
            cout<<"m here 2"<<"slope2 is"<<slope2<<"slope1 is"<<slope1<<endl;
            if(!(1/slope2)){
                cout<<"m here"<<endl;
                x = prev_coordinate1->coordinates_x;
                y = slope1*x+C;
            }
            else if(!(1/slope1)){
                cout<<"m here 1"<<endl;
                x = C;
                y = (slope2*x) -(slope2*prev_coordinate1->coordinates_x) + (prev_coordinate1->coordinates_y);
                cout<<"values are"<<slope2*x<<"and"<<slope2*prev_coordinate1->coordinates_x<<"and"<<prev_coordinate1->coordinates_y<<endl;
            }
            else{
                cout<<"m here 2"<<endl;
                x = (slope2*prev_coordinate1->coordinates_x+C-next_coordinate1->coordinates_y)/(slope2-slope1);
                y = slope1*x+C;
            }          
            cout<<"2nd is"<<"x is"<<x<<"y is"<<endl;
        }
        if(slope1<=1){
            if(y>(prev_coordinate2->coordinates_y)){
                prev_coordinate1 = prev_coordinate2;
                prev_coordinate2 = prev_coordinate2->prev;
                cout<<"M HERE NEHA"<<endl;
            }
            if(y>(next_coordinate2->coordinates_y)){
                next_coordinate1 = next_coordinate2;
                next_coordinate2 = next_coordinate2->next;
                cout<<"M HERE NEHA1"<<endl;
            }
            else
            {
                trajectory_coordinate[0] = x;
                trajectory_coordinate[1] = y;
                trajectory.push_back(trajectory_coordinate);
            }
        }
        else{
            if(x<(prev_coordinate2->coordinates_x)){
                prev_coordinate1 = prev_coordinate2;
                prev_coordinate2 = prev_coordinate2->prev;
                cout<<"M HERE NEHA2"<<endl;
            }
            if(x<(next_coordinate2->coordinates_x)){
                next_coordinate1 = next_coordinate2;
                next_coordinate2 = next_coordinate2->next;
                cout<<"M HERE NEHA3"<<endl;
            }
            else
            {
                trajectory_coordinate[0] = x;
                trajectory_coordinate[1] = y;
                trajectory.push_back(trajectory_coordinate);
            }
        }
            
        float theta =  atan(1/slope1);
        x = x + r*cos(theta);
        y = y + r*sin(theta);
        trajectory_coordinate[0] = x;
        trajectory_coordinate[1] = y;
        trajectory.push_back(trajectory_coordinate);
        i++;
        C = C + r;
    }
    for (int i = 0; i < trajectory.size(); i++)
        {
            cout<<trajectory[i][0]<<",";
            cout<<trajectory[i][1]<<endl;
        }    
        cout<<trajectory.size()<<endl;
        trajectory.clear();
}

void coordinate_finder(){
    float side_max=0;
    coordinates_node* iterator = head;
    coordinates_node* side_coordinate1;
    coordinates_node* side_coordinate2;
    while(iterator->next!=head) {
        float side_length = dist(iterator->coordinates_x, iterator->coordinates_y, (iterator->next)->coordinates_x, (iterator->next)->coordinates_y);
        if(side_length>side_max){
            side_max = side_length;
            side_coordinate1 = iterator;
            side_coordinate2 = iterator->next;
        }
        iterator = iterator->next;
    }
    trajectory_planner(side_coordinate1, side_coordinate2);
    cout<<"coordinate side_coordinate1"<<side_coordinate1->coordinates_x <<","<<side_coordinate1->coordinates_y<<endl;
    cout<<"coordinate side_coordinate2"<<side_coordinate2->coordinates_x <<","<<side_coordinate2->coordinates_y<<endl;

}


int main(){
    read();
    coordinate_finder();
}