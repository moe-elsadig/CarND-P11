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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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



  // start in which lane
  int lane = 1;

  // have a reference velocity to target
  double ref_vel = 0.; // MPH 


  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
            
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];


            int prev_size = previous_path_x.size();

          	json msgJson;

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // check for a previous path and update the car's current s-coordinate
            if(prev_size > 0){

              car_s = end_path_s;
            }

            //a boolean value activated when another vehicle is too close
            bool too_close = false;

            //an int to store the id of the car that activated the too close boolean
            int car_ahead = -1;

            //a value that stores whether it is safe to shift to the left or not
            bool safe_left = true;

            //a value that stores whether it is safe to shift to the right or not
            bool safe_right = true;

            //a value to store the current lane's movement speed
            double cur_lane_spd = 50.;

            //speed limit for the road
            double speed_limit = 49.5;

            //safety gap with car ahead
            double safety_gap_ahead = (car_speed/speed_limit) * 20.;

            //find ref_v to use
            //iterate throught the sensor_fusion parameter (the other vehicles nearby)
            for(int i = 0; i < sensor_fusion.size(); i++){

              //the lateral coordinate of the other vehicle to determine it's current lane
              float d = sensor_fusion[i][6];

              //check for car in same lane or about to change to the same lane
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              //estimate the location of the car in sd coordinates in the next future step
              check_car_s += ((double) prev_size*0.02*check_speed);

              double turn_gap = 4.; 

              //check whether the car is occupying the same lane or not
              if(d < (2+4*lane+2) && d > (2+4*lane-2)){

                //if the car is in the same lane check whether or not it is both ahead and within 30 meters
                if((check_car_s > car_s) && ((check_car_s-car_s) < safety_gap_ahead) && car_speed > check_speed){

                  //if the above condition is true:
                  //set the too_close flag and record the id and speed of the car
                  too_close = true;
                  car_ahead = i;
                  cur_lane_spd = check_speed;
                }
              //if it doesn't occupy the same exact lane check if it is next to or ahead of the car by 20m  
              //if this condition is true, check whether it is in the adjacent lane or entering the current turn to prevent unsafe turns.
              }else if((check_car_s > (car_s-turn_gap)) && ((check_car_s-(car_s-turn_gap)) < (safety_gap_ahead-turn_gap))){

                if(d < 2+4*lane-1 && d > 2+4*(lane-1)-3 && car_speed > check_speed){

                  //set the safe_left flag to false to prevent left turns
                  safe_left = false;
                }

                //for safety the two need to be mutually exclusive even though illogical

                if(d > 2+4*lane+1 && d < 2+4*(lane+1)+3 && car_speed > check_speed){


                  //set the safe_left flag to false to prevent right turns
                  safe_right = false;
                }
              }
            }

            //for debugging behavioud continuously output the status of the three main behaviour changing flags to the console
            cout << "\rToo_close: " << too_close
                 << "\tSafe_right: " << safe_right
                 << "\tSafe_left: " << safe_left
                 << "\tcar/lane_spd: " << car_speed
                 << "  /  " << cur_lane_spd << "      ";

            //based on the above calculations perform any needed behaviour changes
            //check whether a car is too close
            if(too_close){

              //if a car is close which lane should you change to? without a cost function at the moment
              //preference is given to going left
              if(safe_left && lane > 0){

                //decrease the value to turn left
                lane -= 1;

              }else if(safe_right && lane < 2){

                //increase the value to turn right
                lane += 1;

              }else{
                
                //if the car cannot avoid the car ahead decrease the speed to match it to avoid collision
                if(car_speed > cur_lane_spd){

                  double perc = (car_speed-cur_lane_spd)/cur_lane_spd;
                  ref_vel -= .448 * perc; 
                } 
              }
            
            //if there is no car obstructing the path, accelerate towards the speed limit
            }else if(ref_vel < speed_limit){
              
              double perc = (cur_lane_spd - car_speed)/cur_lane_spd;
              ref_vel += .448 * perc;
              // ref_vel += .448;
            }

            //initialise vectors to store the XY map points
            vector<double> ptsx;
            vector<double> ptsy;

            //origin for calculations, the car's current global x, y, and yaw values
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //initialise or extract points from the previous path to add to the x/y map points
            //check whether or not there have been less than two previous rounds of calculations
            if(prev_size < 2){

              //if there hasn't
              //estimate two values for both x and y and add them to the points vectors
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              //add the last two x values
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              //add the last two y values
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            
            }else{

              //if there are more than 2 previous readings
              //reassign the new reference x and y points the last points in the vectors previous path vectors
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              //init x and variables to store the points prior to the reference points in the previous path
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];

              //calculate the reference yaw angles from the two reference point sets
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              //add the x ref and prev_ref points to the xpoints
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              //add the y ref and prev_ref points to the ypoints
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            //get the XY coordinates for points that are 30, 60 and 90 meters ahead of the car in s coordinates
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            //add the x values from the 30, 60 and 90 forward estimations to the x_points
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            //add the y values from the 30, 60 and 90 forward estimations to the y_points
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            //iterate throught the all the points in ptsx and ptsy and shift from the map-xy coordinates to the car-xy coordinates
            for(int i=0; i<ptsx.size(); i++){

              //distances relative to the vehicle
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              //transform the points to the car's xy coordinates and replace them in the pts vectors
              ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }

            //initialise a spline element s
            tk::spline s;

            //set the points for this spline using our pts x and y calculated so far
            s.set_points(ptsx,ptsy);

            //initialise vectors to store the future set of x and y values
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            //iterate through the previous values of the path and add them to the future path vectors
            for(int i=0; i<previous_path_x.size(); i++){

              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }


            //after getting the spline up fro the next 90 m in s-coordinates
            //target a range to immediately follow and draw in the x-axes
            double target_x = 30.0;

            //use the spline to get the corresponding y value
            double target_y = s(target_x);

            //calculate the distance to the target
            double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
            
            //an add-on value for the x-coordinate
            double x_add_on = 0.;

            //the vectors for future x and y values should always have 50 elements, 
            //by checking on the previous path size and subtracting from 50
            for(int i=0; i <= 50-previous_path_x.size(); i++){

              //get the motion step size N by
              //dividing the distance by the calculation step size
              //step size is the step speed * velocity in m/s
              double N = (target_dist/(.02*ref_vel/2.24));

              //find the next x value along the track towards the target
              double x_point = x_add_on + (target_x)/N;

              //find its corresponding y value using the spline
              double y_point = s(x_point);

              //reinitialise x-addon to work on the next set of coordinates on the motion track
              x_add_on = x_point;

              //init variables to store the final track values to follow
              double x_ref = x_point;
              double y_ref = y_point;

              //transform the values back to the map coordinates
              x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

              //add the current position of the vehicle to the newly calculated marker position
              x_point += ref_x;
              y_point += ref_y;

              //push the values to the motion track vector
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            // DOing
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

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
