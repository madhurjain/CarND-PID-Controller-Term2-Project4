#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

bool twiddleRun = false;

// for convenience
using json = nlohmann::json;

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

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  if(twiddleRun)
  {
    pid.Init(0, 0, 0);
  }
  else
  {
    pid.Init(0.12, 0.00025, 3.0);
  }

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value =  - pid.TotalError();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.4;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          // For Twiddle
          if (twiddleRun)
          {
            pid.newErr += pow(cte, 2);
            pid.stepCount += 1;
            if(fabs(cte) > 2)
            {
              std::string resetMsg = "42[\"reset\",{}]";
              ws.send(resetMsg.data(), resetMsg.length(), uWS::OpCode::TEXT);
            }
            if(pid.stepCount >= 500)
            {
              // move parameters to array
              double p[] = { pid.Kp, pid.Ki, pid.Kd };
              double sum_dp = pid.dp[0] + pid.dp[1] + pid.dp[2];
              std::cout << "\n=============================" << std::endl;
              std::cout << "Iteration: " << pid.iteration << std::endl;
              std::cout << "Tune Parameter: " << pid.tuneParameter << ", Current Value: " << p[pid.tuneParameter] << std::endl;
              switch(pid.nextState)
              {
                  case pid.FIRSTRUN:
                    pid.bestErr = pid.newErr;
                    std::cout << "First Run:: Best Err: " << pid.bestErr << ", CTE: " << cte << std::endl;
                    p[pid.tuneParameter] += pid.dp[pid.tuneParameter];
                    std::cout << "Parameter #" << pid.tuneParameter << " incremented by " << pid.dp[pid.tuneParameter]
                              << ", new value: " << p[pid.tuneParameter] << std::endl;
                    pid.nextState = pid.BASELINE;
                    break;
                  case pid.BASELINE:
                    std::cout << "Twiddle Run Baseline:: Best Err: " << pid.bestErr << ", New Err: " << pid.newErr << ", CTE: " << cte << std::endl;
                    // if we already meet the tolerance required, then we are done
                    if(fabs(sum_dp) > pid.tolerance)
                    {
              
                      if (pid.newErr < pid.bestErr)
                      {
                        pid.bestErr = pid.newErr;
                        pid.dp[pid.tuneParameter] *= 1.1;
                        std::cout << "New Err < Best Err for Parameter " << pid.tuneParameter << std::endl;
                        std::cout << "Best Err Updated. Moving to next parameter.." << std::endl;
                        // move to the next parameter and establish a baseline
                        pid.tuneParameter = (pid.tuneParameter + 1) % 3;
                        p[pid.tuneParameter] += pid.dp[pid.tuneParameter];
                        std::cout << "Parameter #" << pid.tuneParameter << " incremented by " << pid.dp[pid.tuneParameter]
                                  << ", new value: " << p[pid.tuneParameter] << std::endl;
                        pid.nextState = pid.BASELINE;
                      }
                      else
                      {
                        // reset p[i] to previous value and rerun for same parameter
                        p[pid.tuneParameter] -= pid.dp[pid.tuneParameter] * 2;
                        pid.nextState = pid.CORRECTION;
                      }
                    }
                    else
                    {
                      pid.nextState = pid.DONE;
                    }
                    break;
                  case pid.CORRECTION:
                    std::cout << "Twiddle Run Correction:: Best Err: " << pid.bestErr << ", New Err: " << pid.newErr << ", CTE: " << cte << std::endl;
                    if (pid.newErr < pid.bestErr)
                    {
                      pid.bestErr = pid.newErr;
                      pid.dp[pid.tuneParameter] *= 1.1;
                      std::cout << "New Err < Best Err for Parameter " << pid.tuneParameter << std::endl;
                      std::cout << "Best Err Updated" << std::endl;
                    }
                    else
                    {
                      p[pid.tuneParameter] += pid.dp[pid.tuneParameter];
                      std::cout << "Parameter #" << pid.tuneParameter << " incremented by " << pid.dp[pid.tuneParameter]
                                << ", new value: " << p[pid.tuneParameter] << std::endl;
                      pid.dp[pid.tuneParameter] *= 0.9;
                    }
                    // move to the next parameter and establish a baseline
                    pid.tuneParameter = (pid.tuneParameter + 1) % 3;
                    p[pid.tuneParameter] += pid.dp[pid.tuneParameter];
                    std::cout << "Parameter #" << pid.tuneParameter << " incremented by " << pid.dp[pid.tuneParameter]
                              << ", new value: " << p[pid.tuneParameter] << std::endl;
                    pid.nextState = pid.BASELINE;
                    break;
                  case pid.DONE:
                    twiddleRun = false;
                    std::cout << "Done Twiddle Run" << std::endl;
                    std::cout << "Kp: " << pid.Kp << ", Ki: " << pid.Ki << ", Kd: " << pid.Kd << std::endl;
                    break;
              } // switch(pid.nextState)
              pid.iteration += 1;
              // reset step count
              pid.stepCount = 0;
              // move back the array to parameters
              pid.Kp = p[0];
              pid.Ki = p[1];
              pid.Kd = p[2];
              std::cout << "Kp: " << pid.Kp << ", Ki: " << pid.Ki << ", Kd: " << pid.Kd << std::endl;
              
              pid.newErr = 0.0;
              std::string resetMsg = "42[\"reset\",{}]";
              ws.send(resetMsg.data(), resetMsg.length(), uWS::OpCode::TEXT);
            } // if pid.stepCount >= 500
          } // if twiddleRun
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
