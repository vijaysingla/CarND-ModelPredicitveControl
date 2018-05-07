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

int main(int argc,const char *argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

dt =stod(argv[1]);
ref_v = stod(argv[2]);
w1 = stod(argv[3]);
w2 = stod(argv[4]);
w3 = stod(argv[5]);
w4 = stod(argv[6]);

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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // ref_x ,ref_y to store waypoints in local vehcile coordinate system

          vector<double> ref_x;
          vector<double>ref_y;
          /*
           * Transformation of global coordinates for reference points to
           * local vehicle coordinates
           */
          for (int i =0 ;i <ptsx.size();i++)
          {
        	  double rel_x = ptsx[i] -px;
			  double rel_y = ptsy[i] - py;
        	  ref_x.push_back(rel_x*cos(psi) +rel_y*sin(psi)) ;
        	  ref_y.push_back(-rel_x*sin(psi) +rel_y*cos(psi));
          }

          // Converting ref_x and ref_y vector into eigen vector as polyfit takes eigne vector as input
          /*
           * this code snipet to convert vector  to eigen vector  is taken from
           * https://stackoverflow.com/questions/17036818/initialise-eigenvector-with-stdvector
           */
          double* ptrx =&ref_x[0];
          double* ptry = &ref_y[0];
          Eigen::Map<Eigen::VectorXd>x_vals(ptrx,6);
          Eigen::Map<Eigen::VectorXd>y_vals(ptry,6);

          // fitting 3rd order polynomial between x and y points of trajectory
          Eigen::VectorXd coeffs = polyfit(x_vals,y_vals,3);

          double cte;
          double epsi;
          /*
           * cte =    py- (coeffs[0] + coeffs[1] * px+coeffs[2]*px*px + coeffs[3]*px*px*px)  ;
           * epsi = psi - atan((coeffs[1]+2*coeffs[2]*px+3*coeffs[3]*px*px));
           * For local coordinate system py,psi,px are zero
           * cte  = 0 -polyeval(coeffs,0)
           * epsi = 0 -atan(coeffs[1])
            */

          cte = 0 - polyeval(coeffs,0);
          epsi = 0 -atan(coeffs[1]);


          Eigen::VectorXd state(6);
          /*
           * In local vehicle coordinate system , px,py,psi are zero
           */
          state << 0,0,0,v,cte,epsi;

          auto vars = mpc.Solve(state, coeffs);
          // Multiplying by -1 as counterclockwise in simulator means right turn

          double steer_value = -1*vars[0];
          double throttle_value = vars[1];
          std::cout<<"steer: " <<steer_value << "throttle: "<<throttle_value <<std::endl;
          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.mpc_values  are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for (int t=0;t<9;t++)
          {
        	  mpc_x_vals.push_back(vars[t+2]);
        	  mpc_y_vals.push_back(vars[t+11]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          /*
           * Creating the reference points using poly fit coeffs for local coordinate system
           */
          for (int i =0 ; i <20; i++)
          {
        	  double x = i*1.5;
        	  double y;
        	  y= coeffs[0] +coeffs[1]*x + coeffs[2]*x*x +coeffs[3]*x*x*x;
        	  next_x_vals.push_back(x);
        	  next_y_vals.push_back(y);
          }

          //points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          std::cout<<"cte:" <<cte<<std::endl;

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
          this_thread::sleep_for(chrono::milliseconds(0));
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
