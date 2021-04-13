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
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

// find lane number of each car.
double findlane(double car_d) {
    int lane = 0;
    if (car_d > 0 && car_d < 4.0) {
        lane = 0;
    } else if (car_d > 4.0 && car_d < 8.0) {
        lane = 1;
    } else if (car_d > 8.0 && car_d < 12.0) {
        lane = 2;
    }
    return lane;
}

// unit conversion
double mph_to_ms(double mph) {
    return mph * (1600.0 / 3600.0);
}

double ms_to_mph(double ms) {
    return ms * (3600.0 / 1600.0);
}


double keep_lane_cost(bool car_ahead, int ego_lane, double front_dist, int collision_cost) {
    double cost=0;
    if (car_ahead) {
        cost = collision_cost;
    } else {
        if (ego_lane == 1) {// Cars should stay in the middle of the road
            cost = 50;
        } else {
            cost = 100;
        }
        if (front_dist >= 150) {
            cost += 0;
        } else {
            cost += 100;
        }
    }

    return cost;

}

double turn_right_cost(bool car_right, int ego_lane, double rightfront_dist, double rightback_dist,
                       int collision_cost, int right_turn_cost) {
    double cost=0;
    if (car_right || ego_lane == 2) { 

        cost = collision_cost;
    } else {
        if (rightfront_dist > 150 && rightback_dist > 30) {
            cost = 0.5 * right_turn_cost;
        } else {
            cost = right_turn_cost;
        }
    }

    return cost;
}

double turn_left_cost(bool car_left, int ego_lane, double leftfront_dist, double leftback_dist,
                      int collision_cost, int left_turn_cost) {
    double cost=0;
    if (car_left || ego_lane == 0) {
        cost = collision_cost;
    } else {
        if (leftfront_dist > 150 && leftback_dist > 30) {
            cost = 0.5 * left_turn_cost;
        } else {
            cost = left_turn_cost;
        }
    }

    return cost;
}

double slow_down(double slow_down_cost) {
    double cost=0;
    cost = slow_down_cost;
    return cost;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Start in lane 1, which is the center lane (0 is left and 2 is right)
  int lane = 1;
  // Start at zero velocity and gradually accelerate
  double ref_vel = 0.0; // mph

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane]
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
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // find lane of my car
          int ego_lane = findlane(car_d);

          // Initialization 
          
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          bool collision_avoidance = false;
          double front_dist = 999;
          double front_car_velocity = 0; 
          double rightfront_dist = 999;
          double rightback_dist = 999;
          double leftfront_dist = 999;
          double leftback_dist = 999;
          double closest_right_dist=999;
          double closest_left_dist=999;

               
          // Find ref_v to use
          for (int i = 0; i < sensor_fusion.size(); i++) {
            //double fusion_id = sensor_fusion[i][0];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double fusion_s = sensor_fusion[i][5];
            int fusion_d = sensor_fusion[i][6];
            double check_speed = sqrt(vx * vy + vy * vy);
            // distance between other cars and my car
            double dist=fusion_s-car_s;
            
            int other_cars_lane = findlane(fusion_d);

            // if other cars on the same lane as the ego car
            if (other_cars_lane == ego_lane) {
                if( (fusion_s > car_s) && (fusion_s - car_s < 40)){
                    car_ahead=true;
                    if (check_speed) {
                        front_car_velocity = ms_to_mph(check_speed);
                      }
                }
                if( (fusion_s > car_s )&& (fusion_s - car_s < 15)){ // Emergency collision avoidance
                    collision_avoidance=true;
                }

                if (dist > 0) {
                    front_dist = min(dist,front_dist); 
                }
            } else if (other_cars_lane + 1 == ego_lane) { 
                if( (car_s - 30 < fusion_s) && (car_s + 30 > fusion_s)){ 
                    car_left= true;
                }
                if (dist > 0) {
                    if (dist<leftfront_dist){
                        leftfront_dist=dist;
                        if(dist<10){
                            closest_left_dist=car_d-fusion_d;
                        }
                    }
                } else {
                    leftback_dist = min(abs(dist),leftback_dist);
                }

            } else if (other_cars_lane - 1 == ego_lane) {
                if( (car_s - 30 < fusion_s) && (car_s + 30 > fusion_s)){
                    car_right= true;
                }
                if (dist > 0) {
                    if (dist<rightfront_dist){
                        rightfront_dist=dist;
                        if(dist<10){
                            closest_right_dist=fusion_d-car_d;
                        }
                    }
                } else {
                    rightback_dist = min(abs(dist),rightback_dist);
                }
            }
          }

          // calculate cost and find the minimum cost.
          double KL_cost;
          double TR_cost;
          double TL_cost;
          double SD_cost;
          double collision_cost = 500;
          double right_turn_cost = 250;
          double left_turn_cost = 250;
          double slow_down_cost = 300;

          vector<double> costs;
          KL_cost = keep_lane_cost(car_ahead, ego_lane, front_dist, collision_cost);
          costs.push_back(KL_cost);
          TR_cost = turn_right_cost(car_right, ego_lane, rightfront_dist, rightback_dist,
                                          collision_cost, right_turn_cost);
          costs.push_back(TR_cost);
          TL_cost = turn_left_cost(car_left, ego_lane, leftfront_dist, leftback_dist,
                                        collision_cost, left_turn_cost);
          costs.push_back(TL_cost);
          SD_cost = slow_down(slow_down_cost);
          costs.push_back(SD_cost);

          //sorting
          int decision = 0;
          double min_cost = 999;

          for (int i = 0; i < costs.size(); ++i) {
              double cost = costs[i];
              if (cost < min_cost) {
                  min_cost = cost;
                  decision = i;
              }
          }

          double speed_diff = 0; 
          const double MAX_SPEED = 49.5; //mph
          const double MAX_ACC = .224; 


          if (decision == 0) {
            lane = ego_lane;
            if (ref_vel < MAX_SPEED) {// accelerate
                speed_diff += MAX_ACC;
            }
            if (closest_left_dist<3.5 ||closest_right_dist<3.5){ // slow down when other cars change lanes 
                speed_diff -= MAX_ACC;
            }
          } else if (decision == 1) {// turn right 
              lane = ego_lane + 1;
          } else if (decision == 2) {// turn left
              lane = ego_lane - 1;
          } else if (decision == 3) {// slow down
              lane = ego_lane;
            if (collision_avoidance){ // Emergency collision avoidance
                speed_diff -= MAX_ACC;
            }else{
                if (ref_vel - front_car_velocity>0) {// decelerate
                    speed_diff -= MAX_ACC;
                } else {
                    speed_diff -= 0.0;
                }
            }
          } 

          // Create a list of evenly spaced waypoints 30m apart
          // Interpolate those waypoints later with spline and fill it in with more points
          vector<double> ptsx;
          vector<double> ptsy;
        
          // Reference x, y, yaw states, either will be the starting point or end point of the previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
        
          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - 0.5 * cos(car_yaw);
            double prev_car_y = car_y - 0.5 * sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use the previous path's end point as starting reference
          else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);
            // Use the two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
        
          // Add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
        
          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
            ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
          }
        
          // Create a spline
          tk::spline s;
          // Set (x,y) points to the spline (i.e. fits a spline to those points)
          s.set_points(ptsx, ptsy);
          
          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // Calculate how to break up spline points so that we travel at desired velocity
          double target_x = 30.0; // 30.0 m is the distance horizon
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y); // this is the d in the diagram
          double x_add_on = 0.0; // Related to the transformation (starting at zero)
          // Fill up the rest of path planner after filling it with previous points, will always output 50 points
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            // Reduce speed if too close, add if no longer close
            ref_vel += speed_diff;
            if (ref_vel > MAX_SPEED) {
                ref_vel = MAX_SPEED;
            } else if (ref_vel < MAX_ACC) {
                ref_vel = MAX_ACC;
            }
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate x, y back to normal
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
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