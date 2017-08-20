#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  
  if (found_null != string::npos) {
    return "";
  }
  
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  
  return "";
}

// Fit a polynomial
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Evaluate a polynomial
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

int main() {
  
  uWS::Hub h;

  // Initializing the model predictive control object
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    string sdata = string(data).substr(0, length);
    
    cout << sdata << endl;
    
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
          
          // Car state
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          
          cout << px << " " << py << " " << psi << " " << v << endl;
          
          cout << endl;
          
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
          
//          double* ptrx = &ptsx[0];
//          Eigen::Map<Eigen::VectorXd> ptsx_eigen(ptrx, 6);
//          
//          double* ptry = &ptsy[0];
//          Eigen::Map<Eigen::VectorXd> ptsy_eigen(ptry, 6);
          
          // Converting vector<double> to Eigen::VectorXd
          Eigen::VectorXd ptsx_eigen(ptsx.size());
          Eigen::VectorXd ptsy_eigen(ptsy.size());
          
          for (int i = 0; i < ptsx.size(); ++i) {
            
            ptsx_eigen[i] = ptsx[i];
            ptsy_eigen[i] = ptsy[i];
            
          }
          
          // Fitting car coordinate waypoints to 3rd order polynomial
          auto coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);
          
          cout << coeffs << endl;
          
          // FIX TO BE SHORTEST DISTANCE TO POINT
          // Calculating CTE
          // cte = f(x) - y; x = 0, y = 0
          double cte = polyeval(coeffs, 0);
          
          // Calculating epsi
          // epsi = psi - arctan(f'(x)); x = 0, psi = 0
          double epsi = -atan(coeffs[1]);
          
          cout << cte << " " << epsi << endl;
          
          // Car state vector
          // x, y, and psi = 0 in car coordinates
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
          
          auto vars = mpc.Solve(state, coeffs);
          
          json msgJson;
          
          // Dividing steer_value by PI(25/180) rad to normalize and by -1 to match simulator steering convention
          msgJson["steering_angle"] = -vars[0]/(deg2rad(25.0));
          msgJson["throttle"] = vars[1];

          //Display the MPC predicted trajectory 
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
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          // Creating coordinate points for the reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double spacing = 2.5;
          int num_points = 25;
          
          for (int i = 1; i < num_points; ++i) {
            
            next_x_vals.push_back(spacing * i);
            next_y_vals.push_back(polyeval(coeffs, spacing * i));
            
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
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
