#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <random>
#include <time.h>
#include <stdio.h>    
#include <stdlib.h>

// for convenience
using json = nlohmann::json;
#define PI 3.14159265359
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double fRand(double fMin, double fMax)
{
    //srand (time(NULL));
    double f = (double)rand() / RAND_MAX;  // (0,1)
    return fMin + f * (fMax - fMin);  // (fmin, fmax)
}

void checkPositive(double *value){
  if (*value<=0.0)
  {
    *value = 0.00001;
  }
}


int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.1, 0.0001, 6);
  pid.setPrevTotalCte(100000000000.0);
  pid.setPrevK(0.5, 0.0001, 0.001);

  PID pid_speed;
  pid_speed.Init(0.5, 0.001, 2);
  

  h.onMessage([&pid, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          pid.UpdateError(cte);
          steer_value = pid.OutputControl();
          // cout<<pid.p_error<<", "<<pid.i_error<<", "<<pid.d_error<<endl;
          if (steer_value > PI/4.0)
          {
            steer_value = PI/4.0;
          } else if (steer_value < - PI/4.0)
          {
            steer_value = -PI/4.0;
          }

          
          double target_speed;
          if (fabs(cte) > 0.5)
          {
            if (fabs(cte) > 1)
            {
              target_speed = 10;
            } else{
              target_speed = 30;
            }
          } else {
            target_speed = 80;
          }
      
          
           
          double speed_cte = speed - target_speed;
          pid_speed.UpdateError(speed_cte);
          double throttle_value  = pid_speed.OutputControl();

          if(throttle_value > 1){
              throttle_value = 1.0;
          }else if(throttle_value < -1){
              throttle_value = -1.0;
          }
          
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          bool tune_params = false;
          if (pid.count == 3500 && tune_params)  // trivial GA to tune params
          { 
            cout<<pid.Kp<<", "<<pid.Ki<<", "<<pid.Kd<<", total error: " << pid.total_cte<<", prev_total_cte: "<<pid.prev_total_cte<<endl;
            double std_p = 0.1;
            double std_i = 0.0001;
            double std_d = 1;
            
            double sel_kp, sel_ki, sel_kd; 
            if (pid.total_cte < pid.prev_total_cte)  // this time better result
            {
              pid.setPrevTotalCte(pid.total_cte);
              pid.setPrevK(pid.Kp, pid.Ki, pid.Kd);
              sel_kp = pid.Kp;
              sel_ki = pid.Ki;
              sel_kd = pid.Kd;
            
            } else {  // last time better result
              sel_kp = pid.prev_kp;
              sel_ki = pid.prev_ki;
              sel_kd = pid.prev_kd;
            }
            cout<<"selected: "<<sel_kp<<", "<<sel_ki<<", "<<sel_kd<<endl;

            double Kp = fRand(sel_kp-std_p, sel_kp+std_p);
            double Ki = fRand(sel_ki-std_i, sel_ki+std_i);
            double Kd = fRand(sel_kd-std_d, sel_kd+std_d);
            checkPositive(&Kp);
            checkPositive(&Ki);
            checkPositive(&Kd);
            pid.Init(Kp, Ki, Kd);
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


