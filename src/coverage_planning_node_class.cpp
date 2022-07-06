#include<coverage_planning_node_class.h>
#include <pluginlib/class_list_macros.h>

//here assumed coordinates cant be 3d
//look for memory deletion after use otherwise it will just keep adding to previous vector
using namespace std;

namespace ns_coverage_path_node{
void coverage_planning_node_class::onInit() {
    std::cout<<"INSIDE ONINIT"<<std::endl;

	ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    mrs_lib::ParamLoader param_loader(nh, "coverage_planning");
	param_loader.loadParam("coverage_planning_area", _coverage_planning_area_);
        //string _coverage_planning_area_ = nh.get_param("coverage_planning_area");
        //advertise service or topic to publish 
    coverage_planning_trajectory_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_to_follow");
    coverage_planning_node_class::read();
    coverage_planning_node_class::coordinate_finder();
    ros::spin();
}

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
        while (coverage_planning_area_file >> data)
        {
            temp1 = new coordinates_node();
            temp1->coordinates_x = data;
            coverage_planning_area_file >> data;
            temp1->coordinates_y = data;
            temp1->prev = temp;
            temp->next = temp1; 
            temp = temp1;
        }
        temp->next = head;
        head->prev = temp;
        coverage_planning_area_file.close();
    }
    return 0;
}
// float coverage_planning_node_class::dist (float x1, float y1, float x2, float y2){
//     float dist_value = sqrt(pow((x1-x2), 2)+pow((y1-y2), 2));
//     return dist_value;
// }
void coverage_planning_node_class::trajectory_planner(coordinates_node* side_coordinate1, coordinates_node* side_coordinate2){
    vector<vector<float>> trajectory;
    vector<float> trajectory_coordinate;
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
    cout<<"prev_coordinate1 are"<<prev_coordinate1->coordinates_x<<","<<prev_coordinate1->coordinates_y<<endl;
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
    for (int i = 0; i < trajectory.size(); i++)
        {
            cout<<"x is"<<trajectory[i][0]<<"y is"<<trajectory[i][1]<<endl;
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
void coverage_planning_node_class::coordinate_finder(){
    float side_max=0;
    coordinates_node* iterator = head;
    coordinates_node* side_coordinate1;
    coordinates_node* side_coordinate2;
    float side_length = dist(iterator, (iterator->next));
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

}

PLUGINLIB_EXPORT_CLASS(ns_coverage_path_node::coverage_planning_node_class, nodelet::Nodelet)
