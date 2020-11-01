#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * Initialize the pid variable.
   */
  /*double Kp_init = 0.25;
  double Ki_init = 0.0003;
  double Kd_init = 3.4;*/
  double Kp_init = 0.25;
  double Ki_init = 0.00031539;
  double Kd_init = 3.47695;
  pid.Init(Kp_init, Ki_init, Kd_init);
  int cycles = 0;
  bool twiddle = false;
  double besttwiddleerr = -1000;
  double bestKp;
  double bestKi;
  double bestKd;
  double dp = 0.01;
  double di = 0.00001;
  double dd = 0.05;
  double lastdir;
  int roundlen = 1440;
  int twiddleparam = 0;

  h.onMessage([&pid, &cycles, &twiddle, &besttwiddleerr, &dp, &di, &dd, &lastdir, &roundlen, &bestKp, &bestKi, &bestKd, &twiddleparam](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          steer_value = pid.CalcResponse(cte);
          steer_value = std::max(steer_value, -1.0);
          steer_value = std::min(steer_value, 1.0);

          double err = pid.GetTotalError();
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Total Error: " << err 
          //          << std::endl;
          
          cycles +=1;
          double throttle = 0.3;

          if (twiddle){
            if (cycles%roundlen==0){
              double lastKp = pid.GetKp();
              double lastKi = pid.GetKi();
              double lastKd = pid.GetKd();
              
              switch(twiddleparam){
                case 0:
                  {
                    double nextKp;
                    if (besttwiddleerr==-1000){
                      besttwiddleerr = err;
                      bestKp = lastKp;
                      nextKp = lastKp+dp;
                      lastdir = 1.0;
                    }
                    else{
                      if ((dp < 0.005)||(cycles==((roundlen*10)*(1+twiddleparam)))){
                        twiddleparam+=1;
                        lastdir = 1.0;
                        std::cout << " Found final parameter Kp!! " << std::endl;
                        if (err<besttwiddleerr){
                          besttwiddleerr = err;
                          bestKp = lastKp;
                        }
                        std::cout << " Total Error: " << besttwiddleerr << "; best Kp: " << bestKp << std::endl;
                        besttwiddleerr = -1000;
                        pid.Init(bestKp, lastKi, lastKd);
                        
                      }
                      else{
                        if (lastdir>0){
                          if (err<besttwiddleerr){
                            besttwiddleerr = err;
                            bestKp = lastKp;
                            dp *=1.1;
                            nextKp = lastKp+dp;
                            lastdir = 1.0;
                          }
                          else{
                            if (cycles==roundlen){
                              lastdir =-1.0;
                              nextKp = lastKp-dp;
                            }
                            else{
                              lastdir =-1.0;
                              nextKp = lastKp-(2*dp);
                            }
                          }
                        }
                        else{
                          if (err<besttwiddleerr){
                            besttwiddleerr = err;
                            bestKp = lastKp;
                            dp *=1.1;
                            nextKp = lastKp-dp;
                            lastdir = -1.0;
                          }
                          else{
                            nextKp = lastKp+(1.7*dp);
                            dp *=0.7;
                            lastdir = 1.0;
                          }
                        }
                      }
                    }

                    std::cout << " Last dir: " << lastdir << std::endl;
                    std::cout <<" last Kp: " << lastKp << std::endl;
                    std::cout << " next Kp: " << nextKp << std::endl;
                    std::cout << " dp: " << dp << std::endl;
                    std::cout << " this Error: " << err << std:: endl;
                    std::cout << " best Error: " << besttwiddleerr << std:: endl;

                    pid.Init(nextKp, lastKi, lastKd);
                    break;
                  }
                case 1:
                  {
                    double nextKi;
                    if (besttwiddleerr==-1000){
                      besttwiddleerr = err;
                      bestKi = lastKi;
                      nextKi = lastKi+di;
                      lastdir = 1.0;
                    }
                    else{
                      if ((di < 0.000005)||(cycles==((roundlen*10)*(1+twiddleparam)))){
                        twiddleparam+=1;
                        lastdir = 1.0;
                        std::cout << " Found final parameter Ki!! " << std::endl;
                        if (err<besttwiddleerr){
                          besttwiddleerr = err;
                          bestKi = lastKi;
                        }
                        std::cout << " Total Error: " << besttwiddleerr << "; best Ki: " << bestKi << std::endl;
                        besttwiddleerr = -1000;
                        pid.Init(lastKp, bestKi, lastKd);
                      }
                      else{
                        if (lastdir>0){
                          if (err<besttwiddleerr){
                            besttwiddleerr = err;
                            bestKi = lastKi;
                            di *=1.1;
                            nextKi = lastKi+di;
                            lastdir = 1.0;
                          }
                          else{
                            if (cycles==roundlen){
                              lastdir =-1.0;
                              nextKi = lastKi-di;
                            }
                            else{
                              lastdir =-1.0;
                              nextKi = lastKi-(2*di);
                            }
                          }
                        }
                        else{
                          if (err<besttwiddleerr){
                            besttwiddleerr = err;
                            bestKi = lastKi;
                            di *=1.1;
                            nextKi = lastKi-di;
                            lastdir = -1.0;
                          }
                          else{
                            nextKi = lastKi+(1.7*di);
                            di *=0.7;
                            lastdir = 1.0;
                          }
                        }
                      }
                    }

                    std::cout << " Last dir: " << lastdir << std::endl;
                    std::cout <<" last Ki: " << lastKi << std::endl;
                    std::cout << " next Ki: " << nextKi << std::endl;
                    std::cout << " di: " << di << std::endl;
                    std::cout << " this Error: " << err << std:: endl;
                    std::cout << " best Error: " << besttwiddleerr << std:: endl;

                    pid.Init(lastKp, nextKi, lastKd);
                    break;
                  }
                case 2:
                  {
                    double nextKd;
                    if (besttwiddleerr==-1000){
                      besttwiddleerr = err;
                      bestKd = lastKd;
                      nextKd = lastKd+dd;
                      lastdir = 1.0;
                    }
                    else{
                      if ((dd < 0.025)||(cycles==((roundlen*10)*(1+twiddleparam)))){
                        twiddle=false;
                        std::cout << " Found final parameter Kd!! " << std::endl;
                        if (err<besttwiddleerr){
                          besttwiddleerr = err;
                          bestKd = lastKd;
                        }
                        std::cout << " Total Error: " << besttwiddleerr << "; best Kd: " << bestKd << std::endl;
                        std::cout << "best Kp: " << bestKp << "; best Ki: " << bestKi << "; best Kd: " << bestKd << std::endl;
                        pid.Init(lastKp, lastKi, bestKd);
                        throttle = 0.0;
                      }
                      else{
                        if (lastdir>0){
                          if (err<besttwiddleerr){
                            besttwiddleerr = err;
                            bestKd = lastKd;
                            dd *=1.1;
                            nextKd = lastKd+dd;
                            lastdir = 1.0;
                          }
                          else{
                            if (cycles==roundlen){
                              lastdir =-1.0;
                              nextKd = lastKd-dd;
                            }
                            else{
                              lastdir =-1.0;
                              nextKd = lastKd-(2*dd);
                            }
                          }
                        }
                        else{
                          if (err<besttwiddleerr){
                            besttwiddleerr = err;
                            bestKd = lastKd;
                            dd *=1.1;
                            nextKd = lastKd-dd;
                            lastdir = -1.0;
                          }
                          else{
                            nextKd = lastKd+(1.7*dd);
                            dd *=0.7;
                            lastdir = 1.0;
                          }
                        }
                      }
                    }

                    std::cout << " Last dir: " << lastdir << std::endl;
                    std::cout <<" last Kd: " << lastKd << std::endl;
                    std::cout << " next Kd: " << nextKd << std::endl;
                    std::cout << " dd: " << dd << std::endl;
                    std::cout << " this Error: " << err << std:: endl;
                    std::cout << " best Error: " << besttwiddleerr << std:: endl;

                    pid.Init(lastKp, lastKi, nextKd);
                    break;
                  }
              }
            }
          }
          
          if (twiddle){
            if ((cycles%300==0)&&(cycles<30*roundlen)){
              std::cout << cycles << std::endl;
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
