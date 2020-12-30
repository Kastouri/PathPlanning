#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using tk::spline;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // start at middle lane
  int lane = 1;

  // try to keep speed close to the limit
  int speed_rf = 22.352; // 50 mph = 22.352 metres per second 

  h.onMessage([&speed_rf, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /**
           *  Take data from senesor fusion  into account
           * check if a car is driving in the current lane
           * slow down if were getting too close
           * TODO: change lane
           */
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            double o_car_s = sensor_fusion[i][5];
            double o_car_d = sensor_fusion[i][6];
            int o_car_lane = o_car_d / 4;

            // if car is in the same lane as           
            
          }
          
          // remaining path size
          int prev_size = previous_path_x.size();

          /**
           *  set of anchor points for the spline
           * use point from the previous path to produce a smooth transition for the neu path
           */
          vector<double> spline_pts_x;
          vector<double> spline_pts_y;
          
          // number of remaining points of the prev. path
          int prev_path_size = previous_path_x.size();
          
          // car last position in the previous path

          // get last point's position and orientation from previous path (currently last pt in spline vec.)
          double car_prev_p_last_x = car_x;
          double car_prev_p_last_y = car_y;
          double car_prev_p_last_yaw = car_yaw;

          // use point from the previous path 
          int min_num_prev_pt = 2; // TODO: this only works with 2
          if (prev_path_size >= min_num_prev_pt) {  // use last 2 points as anchor points
            for (int i = min_num_prev_pt; i >= 1; --i ){
              // push min_num_prev_pt point to the splines anchor vector
              spline_pts_x.push_back(previous_path_x[prev_path_size - i]);
              spline_pts_y.push_back(previous_path_y[prev_path_size - i]);
            }
            car_prev_p_last_x = previous_path_x[prev_path_size - 1];
            car_prev_p_last_y = previous_path_y[prev_path_size - 1];
            car_prev_p_last_yaw = atan2(car_prev_p_last_y - previous_path_y[prev_path_size - 2],
                                          car_prev_p_last_x - previous_path_x[prev_path_size - 2]);
          } else { // no previous path, use cars position and orientation
            // get a previous point using the cars position and orientation
            double prev_point_dist = 1;
            double car_prev_x = car_x - cos(car_yaw) * prev_point_dist;
            double car_prev_y = car_y - sin(car_yaw) * prev_point_dist; 
            // push calculated previous point to the spline anchor vector
            spline_pts_x.push_back(car_prev_x);
            spline_pts_y.push_back(car_prev_y);
            // push the current car position to the spline anchor vector
            spline_pts_x.push_back(car_x);
            spline_pts_y.push_back(car_y);
          }

          /**
           * start from last point and create new points for the spline anchor vector
           */
          // transfrom last point in previous path to frenet
          vector<double> car_p_last_frenet = getFrenet(car_prev_p_last_x, car_prev_p_last_y, car_prev_p_last_yaw,
                                                      map_waypoints_x, map_waypoints_y);
          // add 3 new points to the spline
          for (int i = 0; i < 3; ++i) {
            double spline_s_n = car_p_last_frenet[0] + 30.0 * (i + 1);
            double spline_d_n = car_p_last_frenet[1]; 
            // transform back to x, y coordinates
            vector<double> spline_n = getXY(spline_s_n, spline_d_n, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
            spline_pts_x.push_back(spline_n[0]);
            spline_pts_y.push_back(spline_n[1]);
          }

          /**
           * Generating the new path
           *  start by adding new points to the splines Anchor
           *  generate new points using the spline
           */
           
          // add the remaining points from the previous path to the new path
          for(int i = 0; i < prev_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double car_spline_x_l = car_prev_p_last_x;
          double car_spline_y_l = car_prev_p_last_y;
          double car_spline_yaw_l = car_prev_p_last_yaw;
          
          double car_spline_x = car_prev_p_last_x;
          double car_spline_y = car_prev_p_last_y;
          double car_spline_yaw = car_prev_p_last_yaw;
          
          // generate new points using the spline

          spline s;
          for (int i = 0; i < spline_pts_x.size(); ++i){
            cout << "spline new x:  " << spline_pts_x[i] << endl;
          }
          s.set_points(spline_pts_x, spline_pts_y);
          cout << "set spline" << endl;

          
          for (int i = 0; i < 50 - prev_path_size; ++i) {
            // generate new point
            car_spline_x = car_spline_x_l +  cos(car_spline_yaw_l) * 0.02 * speed_rf; 
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
          cout << " generated " << 50 - prev_path_size << " points for the current path" << endl; 

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}