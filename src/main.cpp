#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
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

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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
  auto prev_vel = 0.0;
  auto current_lane = 1;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&current_lane,&prev_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	std::vector<double> previous_path_x = j[1]["previous_path_x"];
          	std::vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;


            // init
            auto total_lanes = 3;
            auto max_vel = 49.5 * 0.44704; // m/s
            auto min_vel = 0.0; // m/s
            auto acc = 5.0; // m/s^2


            // car ref pos refers to either:
            // - the car's position when there's no previous path
            // - the previous path's last prediction
            auto car_ref_x = car_x;
            auto car_ref_y = car_y;
            auto car_ref_yaw = car_yaw;
            auto car_ref_s = car_s;

            auto car_ref_x_prev = car_x - cos(car_ref_yaw);
            auto car_ref_y_prev = car_y - sin(car_ref_yaw);

            // previous path's last prediction
            auto size_prev_waypts = previous_path_x.size();
            if (size_prev_waypts > 0)
            {
              car_ref_x = previous_path_x[size_prev_waypts - 1];
              car_ref_y = previous_path_y[size_prev_waypts - 1];

              // use last two waypts to find heading
              car_ref_x_prev = previous_path_x[size_prev_waypts - 2];
              car_ref_y_prev = previous_path_y[size_prev_waypts - 2];
              car_ref_yaw = atan2(car_ref_y - car_ref_y_prev,
                                  car_ref_x - car_ref_x_prev);

              car_ref_s = end_path_s;
            }


            // ------------------ prediction of other vehicles
            auto veh_ahead = false;
            auto veh_left = false;
            auto veh_right = false;
            auto veh_ahead_v = 15.0; // m/s
            for (const auto& other_veh : sensor_fusion)
            {
              double other_veh_vx = other_veh[3];
              double other_veh_vy = other_veh[4];
              double other_veh_v = distance(other_veh_vx, other_veh_vy, 0, 0);
              double other_veh_s = other_veh[5];
              double other_veh_d = other_veh[6];

              // project other vehicle by ego car's previous prediction
              other_veh_s += previous_path_x.size() * 0.02 * other_veh_v;

              // TODO: consider s rollover

              // other vehicle is ahead by 30m and in the same lane
              if (other_veh_s > car_ref_s && (other_veh_s - car_ref_s) < 30 &&
                other_veh_d > current_lane*4 && other_veh_d < (current_lane+1)*4)
              {
                veh_ahead = true;
                veh_ahead_v = other_veh_v;
              }
              // other vehicle to the left and within 30m
              else if (other_veh_d > (current_lane-1)*4 && other_veh_d < (current_lane)*4 &&
                abs(other_veh_s - car_ref_s) < 30)
              {
                veh_left = true;
              }
              // other vehicle to the right and within 30m
              else if (other_veh_d > (current_lane+1)*4 && other_veh_d < (current_lane+2)*4 &&
                abs(other_veh_s - car_ref_s) < 30)
              {
                veh_right = true;
              }
            }

 


            // ------------------  behavioral planner

            // if no vehicle in front, keep lane
            auto target_lane = current_lane;

            // otherwise, let's explore our options
            if (veh_ahead)
            {
              // try to go left
              if (current_lane > 0 && !veh_left)
              {
                target_lane = current_lane - 1;
              }

              // try to go right
              else if (current_lane < total_lanes - 1 && !veh_right)
              {
                target_lane = current_lane + 1;
              }

              // keep lane and slow down to the speed of vehicle in front
              else
              {
                acc = -5.0;
                min_vel = veh_ahead_v - 0.5;
              }
            }

            // debug
            std::cout << "curr_lane=" << current_lane <<
                        " veh_f=" << veh_ahead << 
                        " veh_l=" << veh_left << 
                        " veh_r=" << veh_right << 
                        " trgt_lane=" << target_lane <<
                        " a=" << acc <<
                        " minv=" << min_vel <<
                        "\n";


            // ------------------  trajectory generation
            // compute remaining trajectory
            // use previous path waypoints and add remaining waypoints to sum to 50

            // spline
            std::vector<double> ptsx;
            std::vector<double> ptsy;

            // add previous two points
            ptsx.push_back(car_ref_x_prev);
            ptsy.push_back(car_ref_y_prev);
            ptsx.push_back(car_ref_x);
            ptsy.push_back(car_ref_y);

            
            // get waypts with 30m spacing from car frenet coordinates
            auto next_wp0 = getXY(car_ref_s+30, target_lane*4 + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            auto next_wp1 = getXY(car_ref_s+60, target_lane*4 + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            auto next_wp2 = getXY(car_ref_s+90, target_lane*4 + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // tranform waypts from global coordinate system to car coordinate system
            for (auto i = 0; i < ptsx.size(); ++i)
            {
              auto xdiff = ptsx[i] - car_ref_x;
              auto ydiff = ptsy[i] - car_ref_y;

              ptsx[i] = xdiff * cos(car_ref_yaw) + ydiff * sin(car_ref_yaw);
              ptsy[i] = ydiff * cos(car_ref_yaw) - xdiff * sin(car_ref_yaw);
            }
              

            // create spline
            tk::spline s;
            s.set_points(ptsx, ptsy);


            // add previous path waypoints
            vector<double> next_x_vals(previous_path_x);
            vector<double> next_y_vals(previous_path_y);

            auto target_x = 30.0;
            auto target_y = s(target_x);
            auto target_dist = distance(0,0,target_x, target_y);

            // use spline to get remaining waypts
            auto x_prev = 0.0;
            for (int i = 1; i <= 50 - size_prev_waypts; ++i)
            {
              // update velocity such that acc=+-5m/s^2
              auto ref_vel = prev_vel + acc * 0.02;
              // limit to max vel
              if (ref_vel > max_vel)
                ref_vel = max_vel;
              // limit to min vel
              if (acc < 0 && ref_vel < min_vel)
                ref_vel = min_vel;

              double N = target_dist / (0.02 * ref_vel);
              double x_car = x_prev + target_x/N;
              double y_car = s(x_car);

              x_prev = x_car;
              prev_vel = ref_vel;

              // tranform waypts from car coordinate system to global coordinate system
              auto xdiff = x_car;
              auto ydiff = y_car;
              auto x_global = xdiff * cos(-car_ref_yaw) + ydiff * sin(-car_ref_yaw);
              auto y_global = ydiff * cos(-car_ref_yaw) - xdiff * sin(-car_ref_yaw);
              
              x_global += car_ref_x;
              y_global += car_ref_y;

              next_x_vals.push_back(x_global);
              next_y_vals.push_back(y_global);              
            }
            

          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

            current_lane = target_lane;

            // debug
            std::cout << "x=" << car_x << 
                        " y=" << car_y << 
                        " s=" << car_s << 
                        " d=" << car_d << 
                        " yaw=" << car_yaw << 
                        " v=" << car_speed <<
                        " lane=" << current_lane <<
                        "\n"; 


          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
