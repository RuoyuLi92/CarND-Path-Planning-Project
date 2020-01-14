#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
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
  
  // the vehicle start in the middle lane
  int lane = 1;
  
  // Have a reference velocity to target
  double ref_vel = 0.0; //mph
  
  //Flag to switch lanes
  bool laneFlag = true;
  
  h.onMessage([&laneFlag, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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
		  
		  int prev_size = previous_path_x.size();
		  
		  // Pull the position of ego vehicle in future(end of the last planned trajectory)
		  if (prev_size > 0) {
			  car_s = end_path_s;
		  }
		  
		  bool too_close = false;
		  
		  // Find rev_v to use
		  for(int i = 0; i < sensor_fusion.size(); ++i) {
			  
			  // if the car is in my lane
			  float d = sensor_fusion[i][6];
			  if(d < (2 + 4*lane + 2) && d > (2 + 4*lane -2)) {
				  double vx = sensor_fusion[i][3];
				  double vy = sensor_fusion[i][4];
				  double check_speed = sqrt(vx*vx + vy*vy);
				  double check_car_s = sensor_fusion[i][5];
				  
				  check_car_s += ((double)prev_size * .02 * check_speed); // if using previous points 
				  // we can project the s value outwards in time, prev_path_x has points, its unit is [step]
				  // distance[m] = prev_size[step] * 0.02[s/step] * check_speed[m/s]
				  // this distance is predicted using the current speed  of the target car and assume we are
				  // following the trajectory in previous_path_x, so it shows us where the car is in the end of
				  // last planned trajectory
				  
				  // check s value greater than mine and s gap, 
				  if((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
					  
					  
					  
					  // Do some thing here, lower ref_vel will be set so we dont crash into the car infront of us
					  // Could also flag to change lanes.
					  //ref_vel = 29.5; //mph
					  too_close = true;
					  if(lane == 0) {
					  
						  if(safeLaneChange(1, sensor_fusion, car_s, prev_size)) {
							  lane = 1;
							  }
				        }
					  else if(lane == 1) {
						  if(laneFlag) {
							  if(safeLaneChange(0, sensor_fusion, car_s, prev_size)) {
								  lane = 0;
								}
							}
						  else { 
								if(safeLaneChange(2, sensor_fusion, car_s, prev_size)) {
									lane = 2;
							    }
						    }
						}
					  else {
							if(safeLaneChange(1, sensor_fusion, car_s, prev_size)) {
									lane = 1;
								}
							}
					  laneFlag = !laneFlag;
					  
					  // start from blindly turning left when the front vehicle is too slow and we are not at left lane
					  //if(lane > 0) {
						  //lane = 0;
					  //}
				  }
			  }
		  }
		  
		  if(too_close) {
			  ref_vel -= .224; //this .224 mph ends up 5 m/s^2
		  }
		  else if(ref_vel < 49.5) {
			  ref_vel += .224;
		  }
		  
					  
				  
				  
		  /* This following part is the actually trajectory generation part
		  
		  */		  
		  // Create a list of widley spaced (x,y) waypoints, evenly spaced at 30m
		  // Later we will interpolate tehse waypoints with a spline, and fill it
		  // with more points that control speed
		  
		  vector<double>ptsx;
		  vector<double>ptsy;
		  
		  // Keep tracking reference x,y,yaw states
		  // either we will reference the starting point where the car is or at the previous paths end point
		  
		  double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);
		  
		  // if previous size is all consumed,i.e. all planned points have been traversed then use the car as starting reference
		  
		  if(prev_size < 2) {
			  
			  // Use two points that make the path tangent to the car
			  double prev_car_x = car_x - cos(car_yaw);
			  double prev_car_y = car_y - sin(car_yaw);
			  
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);
			  
			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
		  }
		  // Use the previous path's end point as starting reference
		  else {
			  
			  // Redefines reference state as previous path end point
			  ref_x = previous_path_x[prev_size-1];
			  ref_y = previous_path_y[prev_size-1];
			  
			  double ref_x_prev = previous_path_x[prev_size-2];
			  double ref_y_prev = previous_path_y[prev_size-2];
			  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
			  
			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);
			  
			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
			  
			  
			  
		  }
		  
		  // In Frenet add evenly spaced points ahead of the starting reference
		  vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  
		  ptsx.push_back(next_wp0[0]);
		  ptsx.push_back(next_wp1[0]);
		  ptsx.push_back(next_wp2[0]);
		  
		  ptsy.push_back(next_wp0[1]);
		  ptsy.push_back(next_wp1[1]);
		  ptsy.push_back(next_wp2[1]);
		  
		  // Doing transformation on car's local coordinates, so that the reference (x,y) is at origin
		  // and the reference angle is 0 degree
		  
		  for (int i = 0; i < ptsx.size(); ++i) {
			  // Shift car reference angle to 0 degree
			  double shift_x = ptsx[i] - ref_x;
			  double shift_y = ptsy[i] - ref_y;
			  
			  ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
			  ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
		  
		  }
		  
		  // Create a spline
		  tk::spline s;
		  
		  // set (x,y) points to the spline
		  s.set_points(ptsx, ptsy);

		  
		  // Define the actual(x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  
		  // Start with all of the previous path_points from last time
		  // To avoid starting from strach every time, 
		  for (int i = 0; i<previous_path_x.size(); ++i) {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }
		  
		  // Calculate how to break up spline points so that we travel at our desired reference velocity
		  // N * 0.02 * vel = d := [piece] * [s/piece] * [m/s] = [m] use this to calculate N
		  // if we evenly break up 30 meters into N piece, then the car travel at desired velocity.
		  double target_x = 30.0;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
		  
		  double x_add_on = 0; // since we have done the local transformation we start from the origin
		  
		  // Fill up the rest of our path planner after filling it with previous paths, here we will
		  // always out put 50 points
		  for (int i = 1; i<= 50 - previous_path_x.size(); ++i) {
			  
			  
			  double N = (target_dist/(.02 * ref_vel/2.24)); //ref_vel/2.24 transform [mileph]2[meterps]
			  double x_point = x_add_on + target_x/N;
			  double y_point = s(x_point);
			  
			  x_add_on = x_point;
			  
			  double x_ref = x_point;
			  double y_ref = y_point;
			  
			  // We are now in local coordinate, so we transform back to world coordinate 
			  x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
			  y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
			  
			  x_point += ref_x;
			  y_point += ref_y;
			  
			  next_x_vals.push_back(x_point);
			  next_y_vals.push_back(y_point);
			  
		  }
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		  /* The version without line fitting on waypoints of highway map
		  // move the car with constant speed
		  // the car moves from "point" to "point", each 1 second we generate 50 points for
		  // the vehicle to traverse, this 0.5 has unit 0.5[m/step] 50 has unit 50[steps/second]
		  // this 50 [steps/second] also called a 50 pack planner
		  double dist_inc = 0.5;
		  for(int i = 0; i < 50; i++) {
			  double next_s = car_s + (i + 1) * dist_inc;
			  // the waypoints are set on the yellow line, and car is located 1.5 lanes from middle
			  // each lane is 4 meters wide
			  double next_d = 6; 
			  vector<double> vec_xy = getXY(next_s, next_d, map_waypoints_s, 
                     map_waypoints_x, 
                     map_waypoints_y);
			  //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
			  next_x_vals.push_back(vec_xy[0]);
			  //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
			  next_y_vals.push_back(vec_xy[1]);
			//std::cout << car_yaw;
		  }
		  
		  // end
		  */
		  		  
          json msgJson;
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