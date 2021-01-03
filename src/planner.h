#ifndef PLANNER_H
#define PLANNER_H 
#include <vector>
#include <string>
#include "car.h"
using std::vector;
using std::string;


class Planner
{
private:
    /**
     * Data from the sensor fusion module containing a list of double vectors
     * for each detected car on the road: id, x, y, vx, vy, s, d
     */
    vector<vector<double>> sensor_fusion_data;
    /**
     * Current state of the behavioral planner
     * possible states: Keep Lane (KL), Prepare Change Lane Right (PCL-R),
     *   Prepare Change Lane LEft (PCL-L), Change Lane Right (CL-R),
     *   Change Lane Left (CL-L).
     */
    string state;
    
    /**
     * Speed Limit: assumed to be lower 50 mph = 22.352 metres per second
     * and the same for all the lanes
     */
    const double speed_limit = 100.0 ;

    /**
     * Safe distance: minimum distance to vehicles in front of our car
     */
    const double safe_distance = 100.0;
    /**
     * Target Speed: speed that the vehicle should reach.
     * The traget speed will be incremented gradually (to avoid exceeding the aceleration limit)
     * until the speed limit is reached.
     */
    double target_speed;

    /**
     * Helper function to add 2 points from the previous path to the spline anchor vector
     * in order to achieve a smooth transition
     */
    void spline_from_rem_path(double &car_prev_p_last_x,
                                double &car_prev_p_last_y, 
                                double &car_prev_p_last_yaw,
                                vector<vector<double>> &remaining_prev_path,  
                                vector<double> &spline_pts_x, vector<double> &spline_pts_y);

    /**
     * Helper function to generate a path out of the remaining path points
     * and using a spline going through the anchor points
     */ 
    vector<vector<double>> path_from_spline(double &car_prev_p_last_x, double &car_prev_p_last_y, 
                                                double &car_prev_p_last_yaw,
                                                 vector<vector<double>> &remaining_prev_path,
                                             vector<double> &spline_pts_x, vector<double> &spline_pts_y);
    

    /**
     * Helper function to generate a path out of the remaining path points
     * and using a spline going through the anchor points
     */ 
    vector<vector<double>> path_from_spline_old(double &car_prev_p_last_x, double &car_prev_p_last_y, 
                                                double &car_prev_p_last_yaw,
                                                 vector<vector<double>> &remaining_prev_path,
                                             vector<double> &spline_pts_x, vector<double> &spline_pts_y);

    /**
     * FLAGs:
     * collision_warning : this flag is set if the car in front is too close
     * lane_0_safe : no cars are detected in the left lane and change is safe
     * lane_1_safe : no cars are detected in the middle lane and change is safe
     * lane_2_safe : no cars are detected in the right lane and change is safe
     */
    bool collision_warning;
    bool lane_0_safe;
    bool lane_1_safe;
    bool lane_2_safe;

public:
    Planner(); 
    Planner(vector<double> map_waypoints_x,vector<double> map_waypoints_y, 
                vector<double> map_waypoints_s);
    virtual ~Planner();
    
    /**
     * Map Way Points
     */
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

    /**
     *  update the sensor fusion data
     */
    void sensor_fusion(Car car, vector<vector<double>> sensor_fusion_data_new);

    /**
     * Predicts what the other vehicles are going to do
     *  this function uses previous sensor data 
     */
    void prediction(vector<vector<double>> sensor_fusion_data_new);

    /**
     * Calculates the Trajectory
     * returns a vector co50ntaining the x and y coordinates of the generated trajectory
     * the car will slow down to avoid collision with the car in front of it
     */
    vector<vector<double>> trajectory(Car car, vector<vector<double>> remaining_prev_path);
    

    /**
     * Calculates the Trajectory
     * returns a vector containing the x and y coordinates of the generated trajectory
     * the car will slow down to avoid collision with the car in front of it.
     * This version uses s and d coordinates to avoid problems with x and y not being ordered
     * Generating the trajactory is done as follows:
     * - Step 1: 
     *   . take 2 points from the last trajectory. If there is no previous trajectory use car's position
     *     and use car heading to generate a past point
     *   . use the car's s and d coordinates as anchor points for the spline
     * - Step 2:
     *   . generate new anchor points for the spline by adding e.g 30m, 60m and 90m to the s value 
     *     of the last point and setting the d value to the desired line
     * - Step 3: 
     *   . create a spline  using the anchor points. using the s value should be 
     *   . generate s and d values of path points, by starting with s = last_pt_s + (v * dt)
     *     and d = spline(s)
     *   . convert the points to x,y cordinates. 
     */
    vector<vector<double>> trajectory_sd(Car car, vector<vector<double>> remaining_prev_path);
    

    /**
     * Calculate Trajectory given the change lane state
     * returns a vector containing the x and y coordinates of the generated trajectory
     * the car will slow down to avoid collision with the car in front of it
     */
    //vector<vector<double>> generate_change_l_tr(Car car, vector<vector<double>> remaining_prev_path);
    
    /**
     * Plan Vehicles behaviour
     * implements state transition
     */
    void behaviour(Car car);

};


#endif  // PLANNER_H