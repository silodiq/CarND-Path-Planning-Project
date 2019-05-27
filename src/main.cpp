#include <uWS/uWS.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


using nlohmann::json;
using std::string;
using std::vector;

const double up_limit_vel = 49.5;
const double up_down_step = 0.224;
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

  // start in lane 1
  int car_ref_lane = 1;
  // reference velocity to target,mph
  double car_ref_vel = 0; 
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&car_ref_vel,&car_ref_lane]
  (uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length,
                     uWS::OpCode opCode) {
	// "42" at the start of the message means there's a websocket message event.
	// The 4 signifies a websocket message
	// The 2 signifies a websocket event
	//auto sdata = string(data).substr(0, length);
	//cout << sdata << endl;
	if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_vel = j[1]["speed"];
          
		  // previous path data
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int previous_path_size = previous_path_x.size();
          
		  // previous path's end_s and end_d
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
         
		  // other cars' data on the same side
          auto other_cars = j[1]["sensor_fusion"];            
         
		  // if previous path is not empty
          if (previous_path_size > 0) {
            car_s = end_path_s;
          }            
         
		  // true: try to change lane or slow down
		  // false: just in lane 
          bool b_change_or_slow = false;
          
          bool b_cant_go_left = (car_ref_lane == 0);
          bool b_cant_go_right = (car_ref_lane == 2);
         
          short car_lane = ((short)floor(car_d/4));
          bool b_changing_lanes = (car_lane != car_ref_lane);

          // distance of leading vehicle ahead
          double min_dist_left = 9999.0;
          double min_dist_here = 9999.0;
          double min_dist_right = 9999.0;            
         
		  // find the unit normal vector of the position of the curve
          int other_waypoint_idx = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          double other_waypoint_dx = map_waypoints_dx[other_waypoint_idx];
          double other_waypoint_dy = map_waypoints_dy[other_waypoint_idx];            
          
          // go through all other cars
          for (int i = 0; i < other_cars.size(); i++) {
            
            double other_x = other_cars[i][1];
            double other_y = other_cars[i][2];
            float other_d = other_cars[i][6];
            double other_vx = other_cars[i][3];
            double other_vy = other_cars[i][4];
            double other_v = sqrt(other_vx*other_vx+other_vy*other_vy);
            double other_s = other_cars[i][5];
            // calculate vd by dot product with d vector
            double other_vd = other_vx*other_waypoint_dx + other_vy*other_waypoint_dy;
            // calculate vs by knowledge vs*vs + vd*vd = vx*vx + vy*vy
            double other_vs = sqrt(other_vx*other_vx + other_vy*other_vy - other_vd*other_vd);

            other_s += ((double)previous_path_size*.02*other_vs);
            // other_d += ((double)previous_path_size*.02*other_vd); 

            short other_lane = ((short)floor(other_d/4));

            // other car's status
            bool b_other_is_going_right = (other_vd > 2.0);
            bool b_other_is_going_left  = (other_vd < -2.0);
            short other_direction     = b_other_is_going_right ? 1 : (b_other_is_going_left ? -1 : 0);
            short other_merging_lane  = other_lane + other_direction;

            // other car is in my lane or lef or right?
            bool b_other_is_in_my_lane  = (other_lane == car_ref_lane) || (other_merging_lane == car_ref_lane);
            bool b_other_is_left  = (other_lane == car_ref_lane -1) || (other_merging_lane == car_ref_lane -1);
            bool b_other_is_right = (other_lane == car_ref_lane +1) || (other_merging_lane == car_ref_lane +1);

            // other car is ahead, behind or close ?
            double other_distance = other_s-car_s;
            bool b_other_is_ahead  = (other_distance > 0.0) && (other_distance < 30.0);
            bool b_other_is_close  = abs(other_s-car_s) < 10;
            bool b_other_is_behind = (other_distance < 0.0) && (other_distance > -15.0);
           
			// Update the lead distance
            if (other_distance > 0.0) {
              if (b_other_is_in_my_lane) {
				  if (other_distance < min_dist_here) {
					  min_dist_here = other_distance;
				  }
              } else if (b_other_is_left) {
				  if (other_distance < min_dist_left) {
					  min_dist_left = other_distance;
				  }
              } else if (b_other_is_right) {
				  if (other_distance < min_dist_right) {
					  min_dist_right = other_distance;
				  }
              }
            }

            // other car is slower?
            bool b_other_is_slower = other_v- car_ref_vel < 0;
            
			// is ther a car ahead of my and in the same lane
			b_change_or_slow = b_change_or_slow || (b_other_is_ahead && b_other_is_in_my_lane);
            
			// care about collision
            bool b_other_is_obstacle = ( (b_other_is_ahead && b_other_is_slower)
                                      || (b_other_is_behind && !b_other_is_slower)
                                      || b_other_is_close);
            
			// can't change lane when encounter collision dangerous with other cars
			b_cant_go_left = b_cant_go_left || (b_other_is_left && b_other_is_obstacle);
			b_cant_go_right = b_cant_go_right || (b_other_is_right && b_other_is_obstacle);
          }
          
		  // no need to change lane, if target lane distance is closer then current lane
		  b_cant_go_left = b_cant_go_left || (min_dist_left < min_dist_here);
		  b_cant_go_right = b_cant_go_right || (min_dist_right < min_dist_here);
          
          // if car too close is in front of car
          if (b_change_or_slow) {
            
			// already changing lane or can't change lane
            if (b_changing_lanes || (b_cant_go_left && b_cant_go_right) || car_ref_vel < 20) {
				// slow down instead (obeying max. accell. & velocity)
				if (car_ref_vel > 5) {
					car_ref_vel -= up_down_step;
				}
            } 
			// go left lane
			else if (!b_cant_go_left) {				
				car_ref_lane = (car_ref_lane - 1);
            } 
			// go right lane
			else if (!b_cant_go_right) {				
				car_ref_lane = (car_ref_lane + 1);
            }
          } 
		  // too slow, speed up
		  else if (car_ref_vel < up_limit_vel) {
			car_ref_vel += up_down_step;
          }          

		  // create a wide spacing (x,y) list of path points in
		  // 30m, and then interpolate these path points with splines
          // fill in more points
          vector<double> control_x;
          vector<double> control_y;        

		  // reference x,y, yaw state: the state we can actually control
		  // current cars;The current state or end of
		  // path of last round (keep smooth)
		  // track
          double car_ref_x = car_x;
          double car_ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (previous_path_size < 2) {
			// if the previous size is almost empty, use the car as a starting reference
			// but it is necessary to put a point behind it to make the path tangent to the car
            double car_prev_x = car_x - cos(car_yaw);
            double car_prev_y = car_y - sin(car_yaw);
            control_x.push_back(car_prev_x);
            control_x.push_back(car_x);
            control_y.push_back(car_prev_y);
            control_y.push_back(car_y);
          } 
		  else {
			// defines a vehicle that references the previous path of the 
			// country and the path tangent behind the point where the car is tangent
			car_ref_x = previous_path_x[previous_path_size-1];
			car_ref_y = previous_path_y[previous_path_size-1];
            double car_prev_x = previous_path_x[previous_path_size-2];
            double car_prev_y = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(car_ref_y - car_prev_y, car_ref_x - car_prev_x);
            
			// use two points to make the path tangent to the previous point
            // the end
            control_x.push_back(car_prev_x);
            control_x.push_back(car_ref_x);
            control_y.push_back(car_prev_y);
            control_y.push_back(car_ref_y);
          }
         
		  // In Frenet, points are added evenly in 30-meter intervals before the judge starts
          for (int spacing = 30; spacing <= 90; spacing += 30) {
            vector<double> controlpoint = getXY(car_s+spacing, (2+4* car_ref_lane),
                                                map_waypoints_s, map_waypoints_x, map_waypoints_y);
            control_x.push_back(controlpoint[0]);
            control_y.push_back(controlpoint[1]);
          }

		  // convert control points to car coordinates
          // reference location
          vector<double> control_x_car;
          vector<double> control_y_car;

          for (int i = 0; i < control_x.size(); i++) {
            // shift
            double x_shifted = control_x[i]- car_ref_x;
            double y_shifted = control_y[i]- car_ref_y;
            // rotate
            control_x_car.push_back(x_shifted*cos(0-ref_yaw)-y_shifted*sin(0-ref_yaw));
            control_y_car.push_back(x_shifted*sin(0-ref_yaw)+y_shifted*cos(0-ref_yaw));
          }

          // create a spline & set (x,y) points to the spline
          tk::spline spline_car;
          spline_car.set_points(control_x_car, control_y_car);

          // sample (x,y) points from spline
          vector<double> car_next_x;
          vector<double> car_next_y;

          // start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
			  car_next_x.push_back(previous_path_x[i]);
			  car_next_y.push_back(previous_path_y[i]);
          }         

		  // calculate the distance along the spline interpolation
          // so we move at the reference speed we want
          // the path will extend 30 meters in the x direction ahead
          double target_x_car = 30.0;
          // get y value for that distance
          double target_y_car = spline_car(target_x_car);
          // calculate euclidean distance travelled to that point
          double target_dist = sqrt(target_x_car*target_x_car + target_y_car*target_y_car);
          // calculate spacing of points needed to find reference velocity
          double num_points = target_dist / (.02*car_ref_vel /2.24);

          // fill up the rest of our path planner so we have 50 points
          double recent_x_car = 0;
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            double x_car = recent_x_car + target_x_car/num_points;
            double y_car = spline_car(x_car);
            recent_x_car = x_car;
            // transform car coordinates to world coordinates
            // rotate
            double x = (x_car*cos(ref_yaw)-y_car*sin(ref_yaw));
            double y = (x_car*sin(ref_yaw)+y_car*cos(ref_yaw));
            // shift
            x += car_ref_x;
            y += car_ref_y;
            // add to return value
			car_next_x.push_back(x);
			car_next_y.push_back(y);
          }

          json msgJson;
          msgJson["next_x"] = car_next_x;
          msgJson["next_y"] = car_next_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
	  // Manual driving
      } else {        
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });


  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code,
                         char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host,port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
