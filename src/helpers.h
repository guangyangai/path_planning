#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <map>
#include <tuple>
#include <string>
#include <vector>

// for convenience
using std::map;
using std::tuple;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// available successor states
vector<string> successor_states(string prev_state, int lane){
  vector<string> states;
  states.push_back("KL");
  if(prev_state.compare("KL") ==0){
    if(lane != 0){
      states.push_back("LCL");
    }
    if(lane != 2){ 
      states.push_back("LCR");
    }
  }else if (prev_state.compare("LCL") ==0){
    if(lane != 0){
      states.push_back("LCL");
    }
  }else if (prev_state.compare("LCR") ==0) {
    if(lane != 2) {
      states.push_back("LCR");
    }
  }
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

// float calculate_cost(const Vehicle &vehicle, 
//                      const map<int, vector<Vehicle>> &predictions, 
//                      const vector<Vehicle> &trajectory) {
//   // Sum weighted cost functions to get total cost for trajectory.
//   map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, 
//                                                        predictions);
//   float cost = 0.0;

//   // Add additional cost functions here.
//   vector<std::function<float(const Vehicle &, const vector<Vehicle> &, 
//                              const map<int, vector<Vehicle>> &, 
//                              map<string, float> &)
//     >> cf_list = {goal_distance_cost, inefficiency_cost};
//   vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
//   for (int i = 0; i < cf_list.size(); ++i) {
//     float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
//                                                trajectory_data);
//     cost += new_cost;
//   }

//   return cost;
// }

tuple<int, string>  choose_best_lane(const vector<vector<double>> &sensor_fusion, string prev_state, int lane, map<string, int> lane_direction, int prev_size, double car_s){
  vector<string> states = successor_states(prev_state, lane);
  float cost;
  string next_state;
  int best_lane;
  double WEIGHT_CHANGE_LANE = 10;
  double minimum_cost = 999999999;
  vector<float> costs;
  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    cost = 0;
    std::cout << "lane change is:" << lane_direction[*it] << '\n';
    int new_lane = lane + lane_direction[*it];
    std::cout << "new lane is:" << lane_direction[*it] << '\n';
    //calculate cost to move to the new line 
    cost += abs(lane_direction[*it])*WEIGHT_CHANGE_LANE;
    for(int i = 0; i < sensor_fusion.size(); i++){
      float d = sensor_fusion[i][6];
      if(d < 4*new_lane + 4 && d > 4*new_lane){
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];
        check_car_s += (double)prev_size * .02 * check_speed;
        cost += (check_car_s - car_s) * (check_car_s - car_s);
      }
    }
    if (cost<minimum_cost){
      best_lane = new_lane;
      next_state = *it;
      minimum_cost = cost;
    }
  }
  //return best_lane;
  return std::make_tuple(best_lane, next_state);
}

#endif  // HELPERS_H