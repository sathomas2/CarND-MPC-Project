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

// for convenience
using json = nlohmann::json;

// For converting miles per hour to meters per second
double mph2mps(double x) { return x * 1609.34 / 3600.0; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          //double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];
          double steer_angle = j[1]["steering_angle"];
          
          //transform waypoints from global to vehicle coordinates
          //the rotation has to be performed aroudn the origin, so the translation must occur first
          // Vehicle coordinates are now [0,0]
          Eigen::VectorXd next_x_vals(ptsx.size());
          Eigen::VectorXd next_y_vals(ptsy.size());
          for (int i=0; i<ptsx.size(); ++i) {
            next_x_vals[i] = (cos(-1.0*psi) * (ptsx[i]-px)) - (sin(-1.0*psi) * (ptsy[i]-py));
            next_y_vals[i] = (sin(-1.0*psi) * (ptsx[i]-px)) + (cos(-1.0*psi) * (ptsy[i]-py));
          }
          
          // Fit polynomial to waypoints
          auto coeffs = polyfit(next_x_vals, next_y_vals, 2);
      
          // Calculate cross-track and orientation erros.
          // Also calculate new x and y coordinates, taking into account 100ms of lag (0.1s)
          // Convert velocity to meters per sec
          double v_mps = mph2mps(v);
          double lag_x = 0.1*v_mps*cos(steer_angle);
          double lag_y = 0.1*v_mps*sin(steer_angle);
          double cte = polyeval(coeffs, lag_x) - lag_y;
          double epsi = steer_angle - atan(coeffs[1] + 2*coeffs[2]*lag_x);

          cout << "CTE " << cte << endl;
          cout << "EPSI " << epsi << endl;
          cout << "steer angle " << steer_angle << endl;
          cout << endl;
          
          // reference velocity for mpc cost function, in meters per sec
          double ref_v = mph2mps(40);
          
          Eigen::VectorXd state(6);
          state << lag_x, lag_y, steer_angle, v_mps, cte, epsi;
          vector<double> actuators;
          mpc.Solve(state, coeffs, ref_v);
          
          
          json msgJson;
          msgJson["steering_angle"] = mpc.steer_angle;
          msgJson["throttle"] = mpc.throttle;

          //Display the MPC predicted trajectory with a green line
          msgJson["mpc_x"] = mpc.mpc_ptsx;
          msgJson["mpc_y"] = mpc.mpc_ptsy;
          
          // Produce waypoints to show in simulator with a yellow line
          vector<double> way_x_vals(15);
          vector<double> way_y_vals(15);
          for (int i=0; i<15; ++i) {
            way_x_vals[i] = static_cast<double>((i+1)*5);
            way_y_vals[i] = polyeval(coeffs, way_x_vals[i]);
          }
          msgJson["next_x"] = way_x_vals;
          msgJson["next_y"] = way_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be able to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
