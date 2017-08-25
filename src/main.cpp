#include <iostream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <chrono>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "MPC.h"
#include "polynomial.h"

using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  
  return "";
}

int main() {
  
  uWS::Hub h;

  // Initializing the model predictive control object
  MPC mpc;
  
  // Initializing the polynomial object
  Polynomial poly;
  
  h.onMessage([&mpc, &poly](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    string sdata = string(data).substr(0, length);
    
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      
      string s = hasData(sdata);
      
      if (s != "") {
        
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          
          // j[1] is the data JSON object
          
          // Waypoints
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          // Current car state
          // Position in map coordinates
          double px = j[1]["x"];
          double py = j[1]["y"];
          
          // Orientation in radians
          double psi = j[1]["psi"];
          
          // Speed in MPH
          double v = j[1]["speed"];
          
          // Speed in m/s
          v *= 0.447;
          
          // Steering angle in radians
          double delta = j[1]["steering_angle"];
          
          // Aligning simulator steering convention with MPC
          delta *= -1.0;
          
          // Throttle
          double a = j[1]["throttle"];
          
          // 100 ms (0.1 s) actuation latency
          double latency = 0.1;
          
          // Car's length from the front to center of gravity
          double Lf = 2.67;
          
          // Predicting car's state after latency
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi += (v / Lf) * delta * latency;
          v += a * latency;
          
          // Transforming waypoints from map to car coordinates
          // x, y, and psi = 0 in car coordinates
          for (int i = 0; i < ptsx.size(); ++i) {
            // Shifting waypoints to car origin
            double shifted_x = ptsx[i] - px;
            double shifted_y = ptsy[i] - py;
            
            // Rotating waypoints to car's x-axis (heading)
            ptsx[i] = (shifted_x * cos(-psi) - shifted_y * sin(-psi));
            ptsy[i] = (shifted_x * sin(-psi) + shifted_y * cos(-psi));
          }
          
          // Converting vector<double> to Eigen::VectorXd
          Eigen::VectorXd ptsx_eigen(ptsx.size());
          Eigen::VectorXd ptsy_eigen(ptsy.size());
          
          for (int i = 0; i < ptsx.size(); ++i) {
            ptsx_eigen[i] = ptsx[i];
            ptsy_eigen[i] = ptsy[i];
          }
          
          // Fitting car coordinate waypoints to 3rd order polynomial
          auto coeffs = poly.polyfit(ptsx_eigen, ptsy_eigen, 3);
          
          // Calculating CTE
          // cte = f(x) - y; x = 0, y = 0
          double cte = poly.polyeval(coeffs, 0.0);
          
          // Calculating epsi
          // epsi = psi - arctan(f'(x)); x = 0, psi = 0
          double epsi = -atan(coeffs[1]);
          
          // Car state vector
          // x, y, and psi = 0 in car coordinates
          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v, cte, epsi;
          
          // Predicting the car's future state
          auto vars = mpc.Solve(state, coeffs);
          
          //Displaying the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          
          for (int i = 2; i < vars.size(); ++i) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }
          
          // Creating coordinate points for the reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double spacing = 5.0;
          int num_points = 10;
          
          for (int i = 1; i < num_points; ++i) {
            next_x_vals.push_back(spacing * i);
            next_y_vals.push_back(poly.polyeval(coeffs, spacing * i));
          }
          
          json msgJson;
          
          // Dividing steer_value by PI(25/180) rad to normalize
          // Negative to match simulator steering convention
          msgJson["steering_angle"] = -vars[0] / ((25.0 / 180.0) * M_PI);//deg2rad(25.0);
          
          msgJson["throttle"] = vars[1];
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          // Delaying the system by 100 ms (0.1 s) to mimic actuation latency
          this_thread::sleep_for(chrono::milliseconds(100));
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
        
      }
      
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      
    }
    
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    
    const std::string s = "<h1>Hello world!</h1>";
    
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      res->end(nullptr, 0);
    }
    
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
  
}
