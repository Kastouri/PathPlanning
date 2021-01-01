#include "planner.h"
#include "spline.h"
#include "helpers.h"
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;
using tk::spline;


Planner::Planner(vector<double> map_waypoints_x_i, vector<double> map_waypoints_y_i, 
                vector<double> map_waypoints_s_i)  
{
    target_speed = 0.0;
    collision_warning = false;
    map_waypoints_x = map_waypoints_x_i;
    map_waypoints_y = map_waypoints_y_i;
    map_waypoints_s = map_waypoints_s_i;
 
} 
Planner::Planner()  
{

} 

Planner::~Planner()
{
}
void Planner::sensor_fusion(Car car, vector<vector<double>> sensor_fusion_data_new){
    sensor_fusion_data = sensor_fusion_data_new;
    
    // calculates car's current lane
    int lane = car.d / 4;

    // set collision warning flag to false
    collision_warning = false;
    // check if  the left lane is safe, set to true and change if car is found in lane
    lane_0_safe = true;
    lane_1_safe = true;
    lane_2_safe = true;

    // for every vehicle in the traffic
    for (int i = 0; i < sensor_fusion_data.size(); ++i) {
        // extracts vehicles data from sensor fusion data
        double o_car_id = sensor_fusion_data[i][0];
        double o_car_vx = sensor_fusion_data[i][3];
        double o_car_vy = sensor_fusion_data[i][4];
        double o_car_s = sensor_fusion_data[i][5];
        double o_car_d = sensor_fusion_data[i][6];
        // calculates vehicles lane
        int o_car_lane = o_car_d / 4;

        // check if lane is safe
        if ((o_car_lane == 0) && (abs(o_car_s - car.s) < safe_distance)){
            lane_0_safe = false;
        }

        // predict the other car's s coordinate
        o_car_s += 0.02 * sqrt(o_car_vx * o_car_vx + o_car_vy * o_car_vy);
        // if an other car is in the same lane
        if ((o_car_lane == lane) && (o_car_s > car.s) && ((o_car_s - car.s) < safe_distance)) {  // only consider cars in front
            cout << "The current lane  is " << lane << endl; 
            cout << "Car (" << o_car_id << ") in the same lane (" << o_car_lane << ") was detected..." << endl;
            collision_warning = true;
        }           
    }

}

void Planner::spline_from_rem_path(double &car_prev_p_last_x,
                                    double &car_prev_p_last_y, 
                                    double &car_prev_p_last_yaw,
                                    vector<vector<double>> &remaining_prev_path,  
                                    vector<double> &spline_pts_x, vector<double> &spline_pts_y){
          
    // number of remaining points of the prev. path
    int prev_path_size = remaining_prev_path[0].size();

    // use 2 points from the previous path 
    if (prev_path_size >= 2) {  // use last 2 points as anchor points
        for (int i = 2; i >= 1; --i ){
            // push min_num_prev_pt point to the splines anchor vector
            spline_pts_x.push_back(remaining_prev_path[0][prev_path_size - i]);
            spline_pts_y.push_back(remaining_prev_path[1][prev_path_size - i]);
        }
        car_prev_p_last_x = remaining_prev_path[0][prev_path_size - 1];
        car_prev_p_last_y = remaining_prev_path[1][prev_path_size - 1];
        car_prev_p_last_yaw = atan2(car_prev_p_last_y - remaining_prev_path[1][prev_path_size - 2],
                                        car_prev_p_last_x - remaining_prev_path[0][prev_path_size - 2]);
    } else { // no previous path, use cars position and orientation
        // get a previous point using the cars position and orientation
        double prev_point_dist = 1.0;
        double car_prev_x = car_prev_p_last_x - cos(car_prev_p_last_yaw) * prev_point_dist;
        double car_prev_y = car_prev_p_last_y - sin(car_prev_p_last_yaw) * prev_point_dist; 
        // push calculated previous point to the spline anchor vector
        spline_pts_x.push_back(car_prev_x);
        spline_pts_y.push_back(car_prev_y);
        // push the current car position to the spline anchor vector
        spline_pts_x.push_back(car_prev_p_last_x);
        spline_pts_y.push_back(car_prev_p_last_y);
    }
}

vector<vector<double>> Planner::path_from_spline(double &car_prev_p_last_x, double &car_prev_p_last_y, 
                                                double &car_prev_p_last_yaw,
                                                vector<vector<double>> &remaining_prev_path,
                                                vector<double> &spline_pts_x, vector<double> &spline_pts_y){

    // x and y values for the new path
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // number of remaining points of the prev. path
    int prev_path_size = remaining_prev_path[0].size();

    /**
     * Generating the new path
     *  start by adding new points to the splines Anchor
     *  generate new points using the spline
     */
        
    // add the remaining points from the previous path to the new path
    for(int i = 0; i < prev_path_size; ++i) {
        next_x_vals.push_back(remaining_prev_path[0][i]);
        next_y_vals.push_back(remaining_prev_path[1][i]);
    }

    double car_spline_x_l = car_prev_p_last_x;
    double car_spline_y_l = car_prev_p_last_y;
    double car_spline_yaw_l = car_prev_p_last_yaw;
    
    double car_spline_x = car_prev_p_last_x;
    double car_spline_y = car_prev_p_last_y;
    double car_spline_yaw = car_prev_p_last_yaw;
    
    // generate new points using the spline
    spline s;
    s.set_points(spline_pts_x, spline_pts_y);

    for (int i = 0; i < 50 - prev_path_size; ++i) {
        // generate new point
        car_spline_x = car_spline_x_l +  cos(car_spline_yaw_l) * 0.02 * target_speed; 
        car_spline_y = s(car_spline_x);
        car_spline_yaw = atan2(car_spline_y - car_spline_y_l, car_spline_x - car_spline_x_l);
        
        // push to path vector
        next_x_vals.push_back(car_spline_x);
        next_y_vals.push_back(car_spline_y);

        // update last point in path
        car_spline_x_l = car_spline_x;
        car_spline_y_l = car_spline_y;
        car_spline_yaw_l = car_spline_yaw;   
    }      

    return {next_x_vals, next_y_vals};

}

vector<vector<double>> Planner::trajectory(Car car, vector<vector<double>> remaining_prev_path){

    // calculated x and y values for the new path
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<vector<double>> next_xy_vals;
    
    // speed up if too slow and not close too other vehicles
    if (!collision_warning && (target_speed < speed_limit)) {
        target_speed += 0.35;
    }
    else if (target_speed > 5.0 && state == "KL"){// only slow down if not changing lane
        target_speed -= 0.1;
    }
    
    // anchor vector for the spline
    vector<double> spline_pts_x;
    vector<double> spline_pts_y;
    vector<vector<double>> spline_pts_xy;      

    // number of remaining points of the prev. path
    int prev_path_size = remaining_prev_path[0].size();

    // get last point's position and orientation from previous path (currently last pt in spline vec.)
    double car_prev_p_last_x = car.x;
    double car_prev_p_last_y = car.y;
    double car_prev_p_last_yaw = car.yaw;

    // use 2 points from the previous path  to start a smooth spline
    spline_from_rem_path(car_prev_p_last_x, car_prev_p_last_y, car_prev_p_last_yaw, 
                         remaining_prev_path,  
                         spline_pts_x, spline_pts_y);
    /**
     * start from last point and create new points for the spline anchor vector
     */
    // transfrom last point in previous path to frenet
    vector<double> car_p_last_frenet = getFrenet(car_prev_p_last_x, car_prev_p_last_y, car_prev_p_last_yaw,
                                                map_waypoints_x, map_waypoints_y);

    /** 
     * Error d coordinate: d_target - d_current
     * depending on the current behavioral state:
     * Keep Lane: d_err = 0
     * Change to Right Lane: d_err = +4
     * Change to Left Lane: d_err = -4
     */
    // calculate car's current lane
    int lane = car.d / 4;
    double d_err = 0;
    if (state == "CL-R") d_err = 4.0;
    else if (state == "CL-L") d_err = -4.0;


    // add 3 new points to the spline
    for (int i = 0; i < 3; ++i) {
        double spline_s_n = car_p_last_frenet[0] + 30.0 * (i + 1);
        double spline_d_n = lane * 4.0 + 2.0 + d_err; 
        // transform back to x, y coordinates
        vector<double> spline_n = getXY(spline_s_n, spline_d_n, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
        spline_pts_x.push_back(spline_n[0]);
        spline_pts_y.push_back(spline_n[1]);
    }

    /**
     * Generating the new path
     *  use the remaining points from the previous path and new points using a spline  
     */
    next_xy_vals = path_from_spline(car_prev_p_last_x, car_prev_p_last_y, car_prev_p_last_yaw,
                                     remaining_prev_path, spline_pts_x, spline_pts_y);    

    return next_xy_vals;
}

void Planner::behaviour(Car car) {
    // TODO : Use Prediction data
    // Use sensor fusion data
    
    // calculate car's current lane
    int lane_current = car.d / 4;
    
    state = "KL";
    if (collision_warning) {
        if (lane_current == 0){
            if  (lane_1_safe) state = "CL-R";
        }
        else if (lane_current == 1) {
            if (lane_0_safe) state = "CL-L";
            else if (lane_2_safe) state = "CL-R";
        }
        else if (lane_current == 2) {
            if (lane_1_safe) state = "CL-L";
        }

    }
}