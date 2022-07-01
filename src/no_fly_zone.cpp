#include<stdio.h>
sorted_array[number_of_data_points]
//save pointer of particular point in the sorted order 
 brustophedon(sorted_array){
     int i=1, x_iterator = 0;
     up1 = sorted_array[0];
     up2 = up1->next;
     down1 = sorted_array[0];
     down2 = down1->next;
     r = 0.5;
    while(){
        while(sorted_array[i]->coordinate_x -x <r){
            //swap between up and down
            //if is for up to down
            if(x_iterator % 2 == 0){
                x = up1->coordinate_x + x_iterator*r;
                slope1 = (up2->coordinate_y-up1->coordinate_y)/(up2->coordinate_x-up1->coordinate_x);
                slope2 = (down2->coordinate_y-down1->coordinate_y)/(down2->coordinate_x-down1->coordinate_x);
                y = slope1*x + up1->coordinate_y - slope1*up1->coordinate_x;
                trajectory_coordinate.push_back(x);
                trajectory_coordinate.push_back(y);
                trajectory.push_back(trajectory_coordinate);
                y = slope2*x + down1->coordinate_y - slope2*down1->coordinate_x;
                trajectory_coordinate.push_back(x);
                trajectory_coordinate.push_back(y);
                trajectory.push_back(trajectory_coordinate);    
            }
            //else is for down to up
            else{
                x = up1->coordinate_x + x_iterator*r;
                slope1 = (up2->coordinate_y-up1->coordinate_y)/(up2->coordinate_x-up1->coordinate_x);
                slope2 = (down2->coordinate_y-down1->coordinate_y)/(down2->coordinate_x-down1->coordinate_x);
                y = slope2*x + down1->coordinate_y - slope2*down1->coordinate_x;
                trajectory_coordinate.push_back(x);
                trajectory_coordinate.push_back(y);
                trajectory.push_back(trajectory_coordinate);
                y = slope1*x + up1->coordinate_y - slope1*up1->coordinate_x;
                trajectory_coordinate.push_back(x);
                trajectory_coordinate.push_back(y);
                trajectory.push_back(trajectory_coordinate);    
            }
            
            x_iterator++;

            if(up2->coordinate_x - x < r){
                up1 = up1->next;
                up2 = up2->next;
                x_iterator = 0;
            }
            if(down2->coordinate_x - x < r){
                down1 = down1->next;
                down2 = down2->next;
                x_iterator = 0;
            }
        }
      }
     
}