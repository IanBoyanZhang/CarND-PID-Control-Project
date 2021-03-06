#include <uWS/uWS.h>
#include <ctime>
#include "json.hpp"
#include "PID.h"
#include "SimplePI.h"

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

/**
 * Global variables for control twiddle loop
 */
unsigned int step_counter = 0;
unsigned int STEPS_THRESHOLD = 500;
double prev_time;
double curr_time;
double tol = 0.2;
bool use_twiddle = false;

// MPH from simulator
double MAX_SPEED = 80;
double target_speed = 30;

double best_mse_so_far = numeric_limits<double>::max();



// Initial Params combo
/*double _Kp = 0.2;
double _Ki = 0.3;
double _Kd = 0.004;*/

double _Kp = 0.15;
double _Ki = 0.25;
double _Kd = 0.004;

// Speed controller
double _Kp_s = 0.1;
double _Ki_s = 0.002;

void resetSimulator(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

double getTimeDiff(double prev_time, double curr_time) {
  return (curr_time - prev_time)/CLOCKS_PER_SEC;
}

int main(int argc, const char *argv[])
{
  uWS::Hub h;

  PID pid;

  SimplePI simplePI;
  /*************************************************************************
   * Initialize PID Coefficients
   *************************************************************************/

  if (argc != 8) {
    cout << "Now running with default parameters" << endl;
  } else {
    _Kp = strtod(argv[1], NULL);
    _Ki = strtod(argv[2], NULL);
    _Kd = strtod(argv[3], NULL);
/*    _Kp_s = strtod(argv[4], NULL);
    _Ki_s = strtod(argv[5], NULL);

    target_speed = strtod(argv[6], NULL);
    // True non zero
    use_twiddle = strtod(argv[7], NULL);
    */
  }
  pid.Init(_Kp, _Ki, _Kd);
  pid.InitPotentialChange(1, 1, 1);

  cout << "Kp: " << _Kp << endl;
  cout << "Ki: " << _Ki << endl;
  cout << "Kd: " << _Kd << endl;

  simplePI.Init(_Kp_s, _Ki_s);
  simplePI.SetDesired(target_speed);

/*
  cout << "Kp_s: " << _Kp_s << endl;
  cout << "Ki_s: " << _Ki_s << endl;*/

  prev_time = clock();

  h.onMessage([&pid, &simplePI](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value = 0;
          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          /*************************************************************************
           * PID steering control
           *************************************************************************/
          curr_time = clock();
          double dt = getTimeDiff(prev_time, curr_time);
/*          if (dt == 0) {
            dt = 0.1;
          }*/
          prev_time = curr_time;

          cout << "dt:" << dt << endl;

          pid.UpdateError(cte, dt);
          steer_value = pid.Control(1);
          pid.Next();
          /*************************************************************************
           * PI speed control control
           *************************************************************************/
          target_speed = MAX_SPEED * (1 - steer_value);
          simplePI.SetDesired(target_speed);
          double throttle_value = simplePI.Control(speed);
          cout << "Speed" << speed << endl;
          cout << "throttle_value" << throttle_value << endl;

          /*************************************************************************
           * Bang Bang Control
           * In Final submission we are using this
           *************************************************************************/
          if (fabs(steer_value) >= 0.40) {
            throttle_value = -  0.7 * fabs(steer_value);
          } else {
            throttle_value = 1;
          }
          /*************************************************************************
           * Steering Twiddle Loop
           *************************************************************************/
          if (use_twiddle && step_counter >= STEPS_THRESHOLD) {
            pid.Twiddle(tol, pid.GetMSE());
            vector<double> p_vector = pid.GetP();
            vector<double> dp_vector = pid.GetDp();
            if (!pid.ReachMinima()) {
              pid.Init(p_vector[0], p_vector[1], p_vector[2]);
              pid.InitPotentialChange(dp_vector[0], dp_vector[1], dp_vector[2]);
              step_counter = 0;
              // reset simulator
              resetSimulator(ws);
            } else {
              // Reach Minima
              std::cout << "Reach minima -> P" << p_vector[0] << std::endl;
              std::cout << "Reach minima -> P" << p_vector[1] << std::endl;
              std::cout << "Reach minima -> P" << p_vector[2] << std::endl;

              if (pid.GetMSE() >= best_mse_so_far) {
                // reset pid twiddle with other initial guess
                pid.Init(_Kp, _Ki, _Kd);
                pid.InitPotentialChange(0.9, 0.9, 0.9);
                resetSimulator(ws);
              } else {
                best_mse_so_far = pid.GetMSE();
              }
            }
          }

          step_counter += 1;

          cout << "MSE: " << pid.GetMSE() << endl;

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          // msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
