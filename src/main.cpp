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
#include <map>
#include <tuple>
// for convenience
using nlohmann::json;
using std::map;
using std::tuple;
using std::string;
using std::vector;

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
  //starting lane 
  int lane = 1;
  // finite state
  string state = "KL";
  map<string, int> lane_direction = {{"LCL", -1}, {"LCR", 1}, {"KL", 0}};
  //reference velocity
  double ref_vel = 0.0; //mph
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &state, &ref_vel, &lane_direction]
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
          std::cout << "current car speed is:" << car_speed << '\n';
          std::cout << "current car yaw is:" << car_yaw << '\n';
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
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
          //starting state of each cycle
          double pos_x;
          double pos_y;
          double angle;
          
          //define boundaries and interpolate points in between using spine function 
          vector<double> ptsx;
          vector<double> ptsy;
          //passing leftover previous path to next few points to enable smooth path
          int prev_size = previous_path_x.size();
          std::cout << "left over path size is:" << prev_size << '\n';
          double lane_speed;
          lane_speed = get_lane_speed(sensor_fusion, lane, end_path_s);
          std::cout << "current lane speed is:" << lane_speed << '\n';
          /*
          avoid collision
          */
          if (prev_size > 0){
            car_s = end_path_s;
          }
          bool too_close = false;
          //loop through detected vehicles
          for(int i = 0; i < sensor_fusion.size(); i++){
            // find out if the car is in my lane
            float d = sensor_fusion[i][6];
            if(d < 4*lane + 4 && d > 4*lane){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              //adjust for latency 
              check_car_s += (double)prev_size * .02 * check_speed;
              // if the car is in front of us under a certain threshold
              double warning_gap = 30.0;
              if(check_car_s > car_s && check_car_s - car_s < warning_gap){
                too_close = true;
                //switch lane
                int best_lane;
                string next_state;
                tie(best_lane, next_state) =  choose_best_lane(sensor_fusion, state, lane, lane_direction, prev_size, end_path_s);
                std::cout << "best lane is:" << best_lane << '\n';
                std::cout << "next state is:" << next_state << '\n';
                lane = best_lane;
                state = next_state;
              }
            }
          }
          if (too_close){
            //slow down
            ref_vel -= .224;
          }else if(ref_vel < 49.5){
            ref_vel += .224;
          }
          //if car has executed every step planned
          if (prev_size < 2){
            //current location
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            //previous location
            double prev_car_x = car_x - cos(angle);
            double prev_car_y = car_y - sin(angle);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(pos_x);
            ptsy.push_back(pos_y);
            ptsy.push_back(prev_car_y);
          }
          else{ //Previous path's end x and y values 
            pos_x = previous_path_x[prev_size-1];
            pos_y = previous_path_y[prev_size-1];
            double pos_x2 = previous_path_x[prev_size-2];
            double pos_y2 = previous_path_y[prev_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            ptsx.push_back(pos_x2);
            ptsx.push_back(pos_x);
            ptsy.push_back(pos_y2);
            ptsy.push_back(pos_y);
          }
          //set target point (boundary) at a far distance  (to ensure smooth transition)
          vector<double> next_wp30 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp60 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp90 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp30[0]);
          ptsx.push_back(next_wp60[0]);
          ptsx.push_back(next_wp90[0]);
          
          ptsy.push_back(next_wp30[1]);
          ptsy.push_back(next_wp60[1]);
          ptsy.push_back(next_wp90[1]);
          
          //transformation from global coordinates to current local vehicle coordinates
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - pos_x;
            double shift_y = ptsy[i] - pos_y;
            
            ptsx[i] = (shift_x * cos(0-angle) - shift_y* sin(0-angle));
            ptsy[i] = (shift_x * sin(0-angle) + shift_y* cos(0-angle));
          }
          
          //create a spline
          tk::spline s;
          s.set_points(ptsx, ptsy);
          //add leftover previous path points 
          for (int i=0; i<prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          } 
          // interpolate spline points between current position and target_position 
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          //decide how many points to interpolate in order to travel at reference velocity
          double N = (target_dist/(.02*ref_vel/2.24));
          double x_add_on = 0.0;
          for (int i=1; i< 50 - prev_size; i++){
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            //transform back to global coordinates
            double x_point_local = (x_point * cos(angle) - y_point * sin(angle));
            double y_point_local = (x_point * sin(angle) + y_point * cos(angle));
            x_point = x_point_local + pos_x;
            y_point = y_point_local + pos_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
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