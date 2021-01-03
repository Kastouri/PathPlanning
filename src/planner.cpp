#include "planner.h"
#include "spline.h"
#include "helpers.h"
#include <iostream>
#include <cmath>
#include <algorithm>

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
    cout << "using " << spline_pts_x.size() << " points from previous path for the spline." << endl;
    for (int i = 0; i < spline_pts_x.size(); ++i){
        cout << "x = " << spline_pts_x[i] << " y = " << spline_pts_y[i] << endl;
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

    // prit all the used points
    for (int i = 0; i < spline_pts_x.size(); ++i){
        cout << "Point x = "<< spline_pts_x[i] << "y = "<< spline_pts_y[i] << endl;
    }
    cout << " finished" << endl;


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
        /* // generate new point
        car_spline_x = car_loacal_x +  0.005; 
        car_spline_y = s(car_spline_x);
        // car_spline_yaw = atan2(car_spline_y - car_spline_y_l, car_spline_x - car_spline_x_l);
        car_loacal_x = car_spline_x;
        
        double car_global_x = car_spline_x * cos(ref_yaw) - car_spline_y * sin(ref_yaw);
        double car_global_y = car_spline_x * sin(ref_yaw) + car_spline_y * cos(ref_yaw);

        car_global_x += ref_x; 
        car_global_y += ref_y;

        // push to path vector
        next_x_vals.push_back(car_global_x);
        next_y_vals.push_back(car_global_y); */

  
    }      

    return {next_x_vals, next_y_vals};

}

vector<vector<double>> Planner::path_from_spline_old(double &car_prev_p_last_x, double &car_prev_p_last_y, 
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
    /**
     * Check if the points are ordered in x and y directions
     * generate the spline using the directiion that is ordered.
     *  if x is ordered: 
     *     use points (x_i, s(x_i))
     *  esle if y is ordered: 
     *     use points (s(y_i), y_i)
     * in order for both x and y to be an ordered the car will need to do a complete turn
     * which I will assume to be impossible because I'm generating  only short paths (last anchor is 
     * 90m away in s-coordinate direction, which is not enough to make a full turn).
     */
    double test_x_asc = -99999999.0;
    double test_y_asc = -99999999.0;
    double test_x_dsc = 99999999.0;
    double test_y_dsc = 99999999.0;

    bool x_is_ordered = true;
    bool y_is_ordered = true;
    
    bool x_is_asc = true;
    bool y_is_asc = true;
    
    bool x_is_dsc = true;
    bool y_is_dsc = true;
    
    for (int i = 0; i < spline_pts_x.size(); ++i) {
        if ((spline_pts_x[i] <  test_x_asc) || (abs(spline_pts_x[i]-test_x_asc)) < 0.000001)
            x_is_asc = false;
        if ((spline_pts_y[i] <  test_y_asc) || (abs(spline_pts_y[i]-test_y_asc)) < 0.000001)
            y_is_asc = false;
        test_x_asc = spline_pts_x[i];
        test_y_asc = spline_pts_y[i];
    
        if ((spline_pts_x[i] >=  test_x_dsc) || (abs(spline_pts_x[i]-test_x_dsc)) < 0.000001) 
            x_is_dsc = false;
        if (spline_pts_y[i] >=  test_y_dsc || (abs(spline_pts_y[i]-test_y_dsc)) < 0.000001) 
            y_is_dsc = false;
        test_x_dsc = spline_pts_x[i];
        test_y_dsc = spline_pts_y[i];
    }
    x_is_ordered = x_is_asc || x_is_dsc;
    y_is_ordered = y_is_asc || y_is_dsc;
    
    cout << " x values : ";
    for (int i = 0; i < spline_pts_x.size(); ++i){
        cout << spline_pts_x[i] << ", ";
    }
    cout << endl;
    cout << " y values : ";
    for (int i = 0; i < spline_pts_y.size(); ++i){
        cout << spline_pts_y[i] << ", ";
    }
    cout << endl;
    if (!x_is_asc) cout << " Spline points are not in ascending order in x direction" << endl; 
    else cout << " Spline points are in ascending order in x direction" << endl;

    if (!x_is_dsc) cout << " Spline points are not in descending order in x direction" << endl; 
    else cout << " Spline points are in descending order in x direction" << endl;

    if (!y_is_asc) cout << " Spline points are not in ascending order in y direction" << endl; 
    else cout << " Spline points are in ascending order in y direction" << endl;

    if (!y_is_dsc) cout << " Spline points are not in descending order in y direction" << endl; 
    else cout << " Spline points are in descending order in y direction" << endl;


    spline s;
    cout << "Setting spline anchor points ..." << endl;
    if (x_is_ordered){
        if (x_is_dsc) { // revert points order if x order is descending
            std::reverse(spline_pts_x.begin(), spline_pts_x.end());    
            std::reverse(spline_pts_y.begin(), spline_pts_y.end());
            }
        s.set_points(spline_pts_x, spline_pts_y);
    }
    else {
        if (y_is_dsc) { // revert points order if x order is descending
            std::reverse(spline_pts_x.begin(), spline_pts_x.end());    
            std::reverse(spline_pts_y.begin(), spline_pts_y.end());
            }
        s.set_points(spline_pts_y, spline_pts_x);
    }
    // prit all the used points
    for (int i = 0; i < spline_pts_x.size(); ++i){
        cout << "Point x = "<< spline_pts_x[i] << "y = "<< spline_pts_y[i] << endl;
    }
    cout << " finished" << endl;

    for (int i = 0; i < 50 - prev_path_size; ++i) {
        if (!x_is_ordered && !y_is_ordered) break;
        // generate new point
        if (x_is_ordered) {
            cout << " using x direction" << endl;
            car_spline_x = car_spline_x_l +  cos(car_spline_yaw_l) * 0.02 * target_speed; 
            car_spline_y = s(car_spline_x);
            car_spline_yaw = atan2(car_spline_y - car_spline_y_l, car_spline_x - car_spline_x_l);
        }
        else {
            cout << " using y direction" << endl;
            car_spline_y =  car_spline_y_l +  sin(car_spline_yaw_l) * 0.02 * target_speed;
            car_spline_x = s(car_spline_y); 
            car_spline_yaw = atan2(car_spline_y - car_spline_y_l, car_spline_x - car_spline_x_l);
        }
        
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
        if (target_speed > speed_limit) target_speed = speed_limit;
    }
    else if (target_speed > 5.0 && state == "KL"){/** only slow down if not changing lane
                                                   *  slowing down while changing lane is dangerous
                                                   */
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
    
    state = "KL";  // TODO: go back here using transition if the Lane Change is done
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

    // TODO: 
}



vector<vector<double>> Planner::trajectory_sd(Car car, vector<vector<double>> remaining_prev_path){

    
    // speed up if too slow and not close too other vehicles
    if (!collision_warning && (target_speed < speed_limit)) {
        target_speed += 0.35;
        if (target_speed > speed_limit) target_speed = speed_limit;
    }
    else if (target_speed > 5.0 && state == "KL"){/** only slow down if not changing lane
                                                   *  slowing down while changing lane is dangerous
                                                   */
        target_speed -= 0.1;
    }
    
    // anchor vector for the spline
    vector<double> spline_pts_s;
    vector<double> spline_pts_d;

    // number of remaining points of the prev. path
    int prev_path_size = remaining_prev_path[0].size();

    // get last point's position and orientation from previous path (currently last pt in spline vec.)
    double car_prev_p_last_x = car.x;
    double car_prev_p_last_y = car.y;
    double car_prev_p_last_yaw = car.yaw;

    /**
     * Get 2 points from the previous path
     */

    if (prev_path_size >= 2) {  // use last 2 points as anchor points
        for (int i = 2; i >= 1; --i ){
            // push min_num_prev_pt point to the splines anchor vector
            double x = remaining_prev_path[0][prev_path_size - i];
            double y = remaining_prev_path[1][prev_path_size - i];
            double yaw = car_prev_p_last_yaw;
            vector<double> sd = getFrenet(x, y, yaw, map_waypoints_x, map_waypoints_y);
            spline_pts_s.push_back(sd[0]);
            spline_pts_d.push_back(sd[1]);
        }
        car_prev_p_last_x = remaining_prev_path[0][prev_path_size - 1];
        car_prev_p_last_y = remaining_prev_path[1][prev_path_size - 1];
        car_prev_p_last_yaw = atan2(car_prev_p_last_y - remaining_prev_path[1][prev_path_size - 2],
                                        car_prev_p_last_x - remaining_prev_path[0][prev_path_size - 2]);
    } else { // no previous path, use cars position and orientation
        // get a previous point using the cars position and orientation
        double prev_point_dist = 3.0;
        double car_prev_x = car_prev_p_last_x - cos(car_prev_p_last_yaw) * prev_point_dist;
        double car_prev_y = car_prev_p_last_y - sin(car_prev_p_last_yaw) * prev_point_dist; 

        double x = car_prev_x;
        double y = car_prev_y;
        double yaw = car.yaw;

        vector<double> sd = getFrenet(x, y, yaw, map_waypoints_x, map_waypoints_y);
        spline_pts_s.push_back(sd[0]);
        spline_pts_d.push_back(sd[1]);

        x = car_prev_p_last_x;
        y = car_prev_p_last_y;
        yaw = car.yaw;

        sd = getFrenet(x, y, yaw, map_waypoints_x, map_waypoints_y);
        spline_pts_s.push_back(sd[0]);
        spline_pts_d.push_back(sd[1]);
    }

    
    /**
     * Generate new points
     */
    // s and d coordinates of the last point
    vector<double> last_sd = getFrenet(car_prev_p_last_x, car_prev_p_last_y, car_prev_p_last_yaw, map_waypoints_x, map_waypoints_y);
    for (int i = 1; i <= 3; ++i){
        spline_pts_s.push_back(last_sd[0] + 30.0 * i);
        spline_pts_d.push_back(last_sd[1]);        
    }
    /**
     * add remaining path points
     */
        // x and y values for the new path
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<double> next_xy_vals;

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

    /** 
     * Generate Path points using spline
     */
    cout << "s values : ";
    for (int i = 0; i< spline_pts_s.size(); ++i){
        cout << spline_pts_s[i] << ", "; 
    }
    cout << endl;

    spline s;
    s.set_points(spline_pts_s, spline_pts_d);
    double point_s = last_sd[0];
    double point_d = last_sd[1];
    
    for(int i = 0; i < 50 - prev_path_size; ++i) {
        // generate s , d coordinates using the spline
        point_s = point_s + target_speed * 0.02;
        point_d = point_d;
        vector<double> xy = getXY(point_s, point_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
        // convert to x, y 
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }



    

    return {next_x_vals, next_y_vals};
}