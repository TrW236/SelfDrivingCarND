#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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

double check_bound(double var, double upper, double lower) {
  if (var > upper) var = upper;
  if (var < lower) var = lower;
  return var;
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
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];  // The global x positions of the waypoints.
          vector<double> ptsy = j[1]["ptsy"];  // The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
          double px = j[1]["x"];  // The global x position of the vehicle.
          double py = j[1]["y"];  //  The global y position of the vehicle.
          double psi = j[1]["psi"];  // The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions
          double v = j[1]["speed"];  //  The current velocity in mph.

            double t_lat = 0.1, Lf = 2.67;

            double delta = j[1]["steering_angle"];  // * `steering_angle` (float) - The current steering angle in **radians**.
            double throttle = j[1]["throttle"];   // * `throttle` (float) - The current throttle value [-1, 1]

            // state after 100 ms
            double x_lat = px +  v * CppAD::cos(psi) * t_lat;
            double y_lat = py + v * CppAD::sin(psi) * t_lat;
            double psi_lat = psi - v * delta / Lf * t_lat;
            double v_lat = v + throttle *t_lat;  // throttle = a

            for (unsigned int i = 0; i < ptsx.size(); ++i)
            {
                double shift_x = ptsx[i] - x_lat;
                double shift_y = ptsy[i] - y_lat; //py;

                ptsx[i] = (shift_x * cos(-psi_lat) - shift_y * sin(-psi_lat));
                ptsy[i] = (shift_x * sin(-psi_lat) + shift_y * cos(-psi_lat));
            }
            // at this moment the waypoints coordinates are changed according to the state_lat

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
            Eigen::Map<Eigen::VectorXd> eigen_ptsx(&ptsx[0], 6);
            Eigen::Map<Eigen::VectorXd> eigen_ptsy(&ptsy[0], 6);
            Eigen::VectorXd coeffs = polyfit(eigen_ptsx, eigen_ptsy, 3);  // coeffs are according to state_lat

            double epsi_lat = -atan(coeffs[1]);  //psi - atan(coeffs[1] + 2.0 * coeffs[2] * px + 3.0 * coeffs[3] * px * px)
            double cte_lat = polyeval(coeffs, 0.0);  //- 0.0 + v * CppAD::sin(epsi) * t_lat;
            // double epsi_lat = (psi - 0.0) - v * delta / Lf * t_lat;
            cout <<endl << "=================cte"<<cte_lat<<endl; 
            double ref_v;
            if (abs(cte_lat) < 1) ref_v = 100;
            else ref_v = 50;

          // first optimization result
          // double cte = polyeval(coeffs, 0.0);// polyeval(coeffs, px) - py;
          // Due to the sign starting at 0, the orientation error is -f'(x).

          Eigen::VectorXd state_lat(6);
            state_lat << 0.0, 0.0, 0.0, v_lat, cte_lat, epsi_lat;
          // state << 0.0,0.0,0.0,v,cte,epsi;//px, py, psi, v, cte, epsi;

          auto vars = mpc.Solve(state_lat, coeffs, ref_v);

          // double Lf = 2.67; //(deg2rad(25)*Lf)
          double steer_value = check_bound(vars[0], 1.0, -1.0)/deg2rad(25)/Lf;
          double throttle_value = check_bound(vars[1], 1.0, -1.0);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (unsigned int i = 2; i < vars.size(); ++i)
          {
            if (i%2 == 0)
            {
              mpc_x_vals.push_back(vars[i]);
            }
            else
            {
              mpc_y_vals.push_back(vars[i]);
            }
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = 2.5;
          int num_points = 25;

          for (int i = 0; i < num_points; ++i)
          {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
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
