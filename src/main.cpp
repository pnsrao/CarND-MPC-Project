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

int latency_ms = 100;

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

int main(int argc, char* argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  if(argc > 1){
    latency_ms = atoi(argv[1]);
  }
  cout << "Latency value = "<<latency_ms<<"ms"<<endl;
  
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
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
	  //cout<<j[1]<<endl;
          vector<double> ptsxd = j[1]["ptsx"];
          vector<double> ptsyd = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
	  double delta = j[1]["steering_angle"];
	  double acceleration = j[1]["throttle"];
	  
	  /* Read values ptsxd={};ptsyd={};px = 94.03938;py=-139.0399;psi=0.2957008;v=40.4048;delta = -0.02211288;acceleration = 0.4205547;*/
	  
	  if(latency_ms > 0){
	    //Assume that the car continues to travel for 100ms with current parameters
	    // predict state in 100ms
	    double latency_sec = latency_ms/1000;
	    double Lf = 2.67;
	    px = px + v*cos(psi)*latency_sec;
	    py = py + v*sin(psi)*latency_sec;
	    psi = psi + v*deg2rad(25)*(delta/Lf)*latency_sec; //division by Lf 
	    v = v + acceleration*latency_sec;
	  }

	  for(int i = 0; i < ptsxd.size(); i++) {
	    //shift car reference angle to 90 degrees
	    double shift_x = ptsxd[i] - px;
	    double shift_y = ptsyd[i] - py;
	    ptsxd[i] = shift_x * cos(0-psi)-shift_y*sin(0-psi);
	    ptsyd[i] = shift_x * sin(0-psi)+shift_y*cos(0-psi);
	  }
	  /*for(int i = 0; i < ptsxd.size(); i++) {
	    cout<<ptsxd[i]<<",";
	  }
	  cout<<endl;
 	  for(int i = 0; i < ptsyd.size(); i++) {
	    cout<<ptsyd[i]<<",";
	  }
	  cout<<endl;*/
         /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
	  Eigen::VectorXd ptsx = Eigen::VectorXd::Map(ptsxd.data(),ptsxd.size());
	  Eigen::VectorXd ptsy = Eigen::VectorXd::Map(ptsyd.data(),ptsyd.size());
	  auto coeffs = polyfit(ptsx, ptsy, 3);
	  // The cross track error is calculated by evaluating at polynomial at x, f(x)
	  // and subtracting y.
	  double cte = polyeval(coeffs, 0);
	  // Due to the sign starting at 0, the orientation error is -f'(x).
	  // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
	  double epsi = -atan(coeffs[1]);
	  Eigen::VectorXd state(6);
	  state << 0.0, 0.0, 0.0, v, cte, epsi;
	  //cout << cte <<" "<<epsi<<endl;
	  auto vars = mpc.Solve(state, coeffs);
          double steer_value = vars[0]/deg2rad(25);
          double throttle_value = vars[1];
	  
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
	  for(int ii=2;ii<vars.size();ii+=2){
	    mpc_x_vals.push_back(vars[ii]);
	    mpc_y_vals.push_back(vars[ii+1]);
	  }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;// = ptsxd;
          vector<double> next_y_vals;// = ptsyd;
	  double polyinc = 2.5;
	  int num_pts = 25;
	  for(int ii=0;ii<num_pts;ii++){
	    double x_val = ii*polyinc - 2*polyinc;
	    double y_val = polyeval(coeffs,x_val);
	    next_x_vals.push_back(x_val);
	    next_y_vals.push_back(y_val);
	  }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msgJson.dump() << std::endl;
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
