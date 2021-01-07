#include "planner.h"
#include "spline.h"
#include "helpers.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <map>

using std::cout;
using std::endl;
using tk::spline;
using std::map;

Planner::Planner(vector<double> map_waypoints_x_i, vector<double> map_waypoints_y_i, 
                vector<double> map_waypoints_s_i)  
{
    target_speed = 0.0;
    //collision_warning = false;
    map_waypoints_x = map_waypoints_x_i;
    map_waypoints_y = map_waypoints_y_i;
    map_waypoints_s = map_waypoints_s_i;
    
 
} 
Planner::Planner()  
{
    state = "KL";
    intended_lane = 1;
    // speed of each lane
    lanes_speed = {speed_limit, speed_limit, speed_limit};

} 

Planner::~Planner()
{
}
void Planner::sensor_fusion(Car car, vector<vector<double>> sensor_fusion_data_new){
    sensor_fusion_data = sensor_fusion_data_new;
    
    // calculates car's current lane
    int lane = car.d / 4;

    lanes_speed = {speed_limit, speed_limit, speed_limit};

    /* // set collision warning flag to false
    collision_warning = false;
    // check if  the left lane is safe, set to true and change if car is found in lane
    lane_0_safe = true;
    lane_1_safe = true;
    lane_2_safe = true;
 */
    // s offset to the next vehicle
    double s_offset = 30.0;
    // next vehicle's index 
    int next_car_index = -1;
    // next vehicle's speed 
    double next_car_speed = 0;

    // first vehilce determining the speed of each lane
    //vector<Vehicle> detected_vehicles;
    // s values of first vehicle in target lane
    vector<double> target_vehicle_s = {-1, -1, -1}; 
    vector<Vehicle> target_vehicle;
    
    // save all detected vehicles position by id
    map<int, Vehicle> detected_vehicles;
    
    
    vehicles_in_lane = {};
    
    // for every vehicle in the traffic
    for (int i = 0; i < sensor_fusion_data.size(); ++i) {
        // extracts vehicles data from sensor fusion data
        int o_car_id = sensor_fusion_data[i][0];
        double o_car_vx = sensor_fusion_data[i][3];
        double o_car_vy = sensor_fusion_data[i][4];
        double o_car_s = sensor_fusion_data[i][5];
        double o_car_d = sensor_fusion_data[i][6];
        
        // calculates vehicles lane
        int o_car_lane = o_car_d / 4;
        
        // calculate vehicles speed
        double o_car_speed = sqrt(o_car_vx * o_car_vx + o_car_vy * o_car_vy); 

        // Vehicle 
        Vehicle o_car;
        o_car.id = sensor_fusion_data[i][0];
        o_car.s = sensor_fusion_data[i][5];
        o_car.d = sensor_fusion_data[i][6];
        o_car.speed = o_car_speed;
        // save in the detected vehicles list
        detected_vehicles[o_car.id] = o_car;
        // save in the lanes vehicles list
        vehicles_in_lane[o_car_lane].push_back(o_car);  // TODO: change with push back function and 

        // predict the other car's s coordinate
        o_car_s += 0.02 * o_car_speed;

        /**
         * Find the slow vehicle
         * check if vehilce is in the same lane and is next whithin a limited range
         */
        if ((lane == o_car_lane) && ((o_car_s - car.s) < s_offset) && (o_car_s > car.s)) {
            next_car_index = i;
            s_offset = (o_car_s - car.s);
            next_car_speed = o_car_speed; 
        } 
        
        /**
         * find the slowest vehicle in the lane to determine the lane's speed
         */
        /* 
        // check if lane is safe
        if ((o_car_lane == 0) && (abs(o_car_s - car.s) < safe_distance)){
            lane_0_safe = false;
        }
         */
        /* // if an other car is in the same lane
        if ((o_car_lane == lane) && (o_car_s > car.s) && ((o_car_s - car.s) < safe_distance)) {  // only consider cars in front
            //cout << "The current lane  is " << lane << endl; 
            //cout << "Car (" << o_car_id << ") in the same lane (" << o_car_lane << ") was detected..." << endl;
            collision_warning = true;
        }            */
    }
    if (next_car_index != -1) {
        cout << "A car was detected whithin the safe distance in the same line." << endl;
        cout << "Car ID " << sensor_fusion_data[next_car_index][0] << ", Speed : " << next_car_speed  << endl;
    }
    else {
        //cout << "No car detected in the same lane." << endl;
    }

    slow_car.id = next_car_index;
    slow_car.speed = next_car_speed;
    
    // no vehicle
    Vehicle no_target;
    no_target.id = -1;
    no_target.speed = 0.0;
    target_vehicles = {no_target,no_target,no_target};
    // find the vehicle that determines the lines speed
    for (int lane_i = 0; lane_i <= 2; ++lane_i) {
        double lowest_s = car.s + 40.0;
        for (auto vehicle : vehicles_in_lane[lane_i]) {
            if ((vehicle.s < lowest_s) && (vehicle.s > car.s - 10.0)) {
                lanes_speed[lane_i] = vehicle.speed;
                target_vehicles[lane_i] = vehicle;
            }
        }
        // cout << "Lane " << lane_i << " speed is " << lanes_speed[lane_i] << endl; 
    // set lane speed to speed limit if lane is empty
    
    
    }
}


void Planner::spline_from_rem_path(double &car_prev_p_last_x,
                                    double &car_prev_p_last_y, 
                                    double &car_prev_p_last_yaw,
                                    vector<vector<double>> &remaining_prev_path,  
                                    vector<double> &spline_pts_x, vector<double> &spline_pts_y){
          
    // number of remaining points of the prev. path
    int prev_path_size = remaining_prev_path[0].size();
    //cout << "previous path has " << prev_path_size << " points" << endl; 
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
/*     cout << "using " << spline_pts_x.size() << " points from previous path for the spline." << endl;
    for (int i = 0; i < spline_pts_x.size(); ++i){
        cout << "x = " << spline_pts_x[i] << " y = " << spline_pts_y[i] << endl;
    } */
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
    
    /**
     *  generate new points using the spline
     */
    // convert spline points to local coordinates
    double ref_x = car_prev_p_last_x;
    double ref_y = car_prev_p_last_y;
    double ref_yaw = car_prev_p_last_yaw;
    
    for (int i = 0 ; i < spline_pts_x.size(); ++i){
        double shift_x = spline_pts_x[i] - ref_x;
        double shift_y = spline_pts_y[i] - ref_y;
        spline_pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        spline_pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }


    spline s;
    s.set_points(spline_pts_x, spline_pts_y);

/*     // prit all the used points
    for (int i = 0; i < spline_pts_x.size(); ++i){
        cout << "Point x = "<< spline_pts_x[i] << "y = "<< spline_pts_y[i] << endl;
    }
    cout << " finished" << endl;
 */

    double car_loacal_x = ref_x;

    double x_add_on = 0;
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    

    for (int i = 0; i < 50 - prev_path_size; ++i) {

        /** 
         * approach from Q&A video
         */
        double N = (target_dist / (0.02 * target_speed));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);
        x_add_on = x_point;
        double x_ref = x_point;
        double y_ref = y_point;
        
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
        
        x_point += ref_x;
        y_point += ref_y;
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
  
    }      

    return {next_x_vals, next_y_vals};

}


vector<vector<double>> Planner::trajectory(Car car, vector<vector<double>> remaining_prev_path){

    // calculated x and y values for the new path
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<vector<double>> next_xy_vals;
    
    // speed up if too slow and not close too other vehicles
    
    //if (!collision_warning && (target_speed < speed_limit)) {
    //    target_speed += 0.35;
    //    if (target_speed > speed_limit) target_speed = speed_limit;
    //}
    //else if (target_speed > 5.0 && state == "KL"){/** only slow down if not changing lane
    //                                               *  slowing down while changing lane is dangerous
    //                                               */
    //    target_speed -= 0.01 * target_speed;
    //}
    
    /**
     * If we are in the keep line state: match the slow car's speed if one is detected
     *  or the speed limit.
     */ 
    if (state == "KL") {
        // if there is a slower car
        if (slow_car.id != -1) {
            cout << "slow car's id " << slow_car.id << endl; 
            // emergency brake
            if ((slow_car.s - car.s) < 5.0) {
                target_speed -= 0.015 * target_speed;
            }   
            // slow daown if we're faster
            else if (target_speed > slow_car.speed) {
                target_speed -= 0.010 * target_speed;
            } 
            // speed up to match slow car's or limit speed
            else if ((target_speed < slow_car.speed) && (target_speed < speed_limit) ){
                target_speed += 0.35;
            }
        }
        // if no slower car is detected, match the speed limit
        else if (target_speed < speed_limit){
            target_speed += 0.35;
            if (target_speed > speed_limit) target_speed = speed_limit;
        }
    } 
    if (state == "PCL-R" || state == "PCL-L") {
        
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
    double d_err = (intended_lane - lane) * 4.0;
   /*  if (state == "CL-R") d_err = 4.0;
    else if (state == "CL-L") d_err = -4.0; */


    // add 3 new points to the spline
    cout << "Adding new points to the spline" << endl;
    for (int i = 0; i < 3; ++i) {
        double spline_s_n = car_p_last_frenet[0] + 30.0 * (i + 1);
        double spline_d_n = lane * 4.0 + 2.0 + d_err; 
        // transform back to x, y coordinates
        vector<double> spline_n = getXY(spline_s_n, spline_d_n, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
        cout << " x = " << spline_n[0] << " y = " <<  spline_n[1] << endl; 
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
    
    // calculate car's current lane
    int lane_current = car.d / 4;
    
    //state = "KL";  // TODO: go back here using transition if the Lane Change is done
    /* if (collision_warning) {
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
    } */
    /**
     * TODO:
     * - list all ppossible states and generate a rough idea of a trajectory ??
     * - calculate the cost of each of the states 
     * - choose the best state to go to
     */
    string best_next_state = state;
    double best_cost = 9999999;
    vector<string> next_states = get_succ_states(state);
    for (auto next_state: next_states){
        double cost = 0.0;
        // lane speed cost 
        cost += 1.0 * lane_speed_cost(car, next_state);
        cost += 3.0 * safety_cost(car, next_state);
        cost += 0.005 * lane_position_cost(car, next_state);
        // TODO: more cost functions

        // if better cost is found
        if (cost < best_cost) {
            best_next_state = next_state;
            best_cost = cost;
        }
    }
    cout << "current state : " << state << endl;
    cout << "best next state : " << best_next_state << endl; 
    cout << "total cost: intended lane speed : " << best_next_state << " cost is "  << best_cost << endl;
    /**
     * TODO: depending on the chosen state
     * for the KL state set target lane (d) to same lane
     * for the PLC State: the target lane is empty or contains vehicles
     */
    int current_lane = car.d / 4;
    intended_lane = current_lane;
    if (state == "PCL-L" || state == "CL-L") { // KL", "PCL-L", "PCL-R"};  ")
        intended_lane -= 1;
    } 
    else if (state == "PCL-R" || state == "CL-R") {
        intended_lane += 1;
    }
    state = best_next_state;
}

double Planner::lane_speed_cost(Car car, string state) {
    double cost = 0.0;
    // determine the intended lane
    int current_lane = car.d / 4;
    int intended_lane = current_lane;
    if (state == "PCL-L" || state == "CL-L") { // KL", "PCL-L", "PCL-R"};  ")
        intended_lane -= 1;
    } 
    else if (state == "PCL-R" || state == "CL-R") {
        intended_lane += 1;
    }
    
    if (intended_lane < 0 || intended_lane > 2) { 
        cost = 999; // stay in the road!!!
    }
    else {
        cost = (speed_limit - lanes_speed[intended_lane]);
        cost = cost / speed_limit; // nomalize to values from 0 to 1
        if (cost < 0.0) cost = 0.0;  // if fastest car is driving faster than the speed limit 
    } 
    cout << "cost function: intended lane speed : " << state << " cost is "  << cost << endl;
    return cost; // TODO: make sure lane speed is not higher than speed limit
}

double Planner::safety_cost(Car car,  string state){
    double cost = 0;
    // determine the intended lane
    int current_lane = car.d / 4;
    int intended_lane = current_lane;
    if (state == "PCL-L" || state == "CL-L") { // KL", "PCL-L", "PCL-R"};  ")
        intended_lane -= 1;
    } 
    else if (state == "PCL-R" || state == "CL-R") {
        intended_lane += 1;
    }
    if (intended_lane < 0 || intended_lane > 2) return  999; // stay in the road!!!

    // check if there are close vehicles in the intended lane
    if (intended_lane == current_lane){
        // keeping lane is safe -> cost = 0
        cost = 0.0;
    }
    // if there are vehicles in the other lanes
    else if (vehicles_in_lane[intended_lane].size() > 0){
        // find the closest car in the intended lane
        double closest_vehicle_s = 10.0;
        for (auto vehicle : vehicles_in_lane[intended_lane]) {
            double s_dist = abs(vehicle.s - car.s);
            if (s_dist < closest_vehicle_s){
                closest_vehicle_s = s_dist;
            }
        }
        cost = 1.0 - (closest_vehicle_s / 10.0);  // normalize
    }
    else{ 
        cost = 0.0; // TODO: change to penalize changing lane 
    }
    cout << "cost function: transition safety : " << state << " cost is "  << cost << endl;
    return cost;
}

double Planner::lane_position_cost(Car car,  string state) {
    double cost = 0;
    // determine the intended lane
    int current_lane = car.d / 4;
    int intended_lane = current_lane;
    if (state == "PCL-L" || state == "CL-L") { // KL", "PCL-L", "PCL-R"};  ")
        intended_lane -= 1;
    } 
    else if (state == "PCL-R" || state == "CL-R") {
        intended_lane += 1;
    }
    if (intended_lane < 0 || intended_lane > 2) return  999; // stay in the road!!!

    // penalize the most left and most right lanes 
    if (intended_lane == 1) {
        cost = 0.0;
    } 
    else if (intended_lane == 0){
        cost = 1.0;
    } 
    else if (intended_lane == 2){
        cost = 1.0;
    }
    cout << "cost function: lane position: " << state << " cost is "  << cost << endl;
    return cost;
}

