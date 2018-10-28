#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

using namespace std;

const int N = 1600;

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
    
    double tol=0.02;
    int n_steps=0;
    double err = 0.0;
    double best_err = 0.0;
    int i_pid=0;
    int mv_negtive = 0;
    int mv_positive = 0;
    int rd_negtive = 0;
    int rd_positive = 0;
    int first_run = 1;
    
    vector<double> p = {0, 0, 0};
    vector<double> dp = {1, 1, 1};
    //pid.Init(p[0], p[1], p[2]);
    
    //pid.Init(0.77352, 0.0, 0.34343);
    pid.Init(0.25, 0.00005, 6.5);
    
    h.onMessage([&pid, &first_run, &i_pid, &mv_positive, &mv_negtive, &rd_positive, &rd_negtive, &p, &dp, &best_err, &err, &n_steps, &tol](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
                    
                    // regular run & count steps
                    pid.UpdateError(cte);
                    steer_value = pid.TotalError();
                    
                    if(steer_value > 1){ steer_value = 1; }
                    if(steer_value < -1){ steer_value = -1; }
                    /*
                    n_steps++;
                    //cout << n_steps << ",  ";
                    
                    // collect running errors
                    if(n_steps>=N){
                        err += cte*cte;
                    }
                    err /= N;
                    
                    // when reaching the maximum steps, pid tuning
                    if(n_steps==2*N){
                        
                        // run the first time: init error -> best_err
                        if(first_run){
                            best_err = err;
                            first_run = 0;
                            
                            mv_positive = 1;
                            rd_positive = 0;
                            
                            mv_negtive = 0;
                            rd_negtive = 0;
                            //cout << "first run" << endl;
                        }
                        
                        
                        // parameter tune
                        // if the sum of dps < tolerance, print out the optimal parameters
                        cout << "err: " << err << "  best_err: " << best_err << endl;
                        cout << " sum dp: " << dp[0]+dp[1]+dp[2] << " p: " << p[0] << " i: " << p[1] << " d: " << p[2] << endl;
                        
                        if((rd_positive || rd_negtive) && (err < best_err)){
                            
                            //cout << "read, err < best_err" << endl;
                            
                            best_err = err;
                            dp[i_pid] *= 1.1;
                            
                            i_pid++;
                            if(i_pid==3){ i_pid=0; }
                            
                            mv_positive = 1;
                            rd_positive = 0;
                            
                            mv_negtive = 0;
                            rd_negtive = 0;
                            
                            err = 0.0;
                            n_steps = 0;
                        }
                        
                        if(rd_positive && (err >= best_err)){
                            
                            //cout << "read +, err > best_err" << endl;
                            
                            mv_positive = 0;
                            rd_positive = 0;
                            
                            mv_negtive = 1;
                            rd_negtive = 0;
                        }
                        
                        if(rd_negtive && (err >= best_err)){
                            
                            
                            //cout << "read -, err > best_err" << endl;
                            
                            p[i_pid] += dp[i_pid];
                            dp[i_pid] *= 0.9;
                            
                            i_pid++;
                            if(i_pid==3){ i_pid=0; }
                            
                            mv_positive = 1;
                            rd_positive = 0;
                            
                            mv_negtive = 0;
                            rd_negtive = 0;
                            
                            err = 0.0;
                            n_steps = 0;
                        }
                        
                        // tuning to +,
                        if(mv_positive){
                            
                            //cout << "move +" << endl;
                            p[i_pid] += dp[i_pid];
                            
                            mv_positive = 0;
                            rd_positive = 1;
                            
                            mv_negtive = 0;
                            rd_negtive = 0;
                            
                            err = 0.0;
                            n_steps = 0;
                            pid.Init(p[0], p[1], p[2]);
                            std::string msg = "42[\"reset\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            
                        }
                        
                        // tuning to -,
                        if(mv_negtive){
                            
                            //cout << "move -" << endl;
                            p[i_pid] -= 2*dp[i_pid];
                            
                            mv_positive = 0;
                            rd_positive = 0;
                            
                            mv_negtive = 0;
                            rd_negtive = 1;
                            
                            err = 0.0;
                            n_steps = 0;
                            pid.Init(p[0], p[1], p[2]);
                            std::string msg = "42[\"reset\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }*/
                    /*
                    double thr=0;
                    if(abs(pid.d_error)>0.01){
                        thr = 0.3;
                    }else{
                        thr = 0.3;
                    }
                    cout << "pid.d_error " << pid.d_error << "thr " << thr << endl;
                     */
                    
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
