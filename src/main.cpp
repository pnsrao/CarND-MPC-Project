#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

  // MPC is initialized here!
  MPC mpc;
  int iters = 0;
  ifstream inFile("../lake_track_waypoints.csv");
  string s;
  inFile >> s; // First line
  char c;
  double xval,yval;
  vector<double> ptsxd;
  vector<double> ptsyd;
  while(inFile >>xval>>c>>yval){
    ptsxd.push_back(xval);
    ptsyd.push_back(yval);
  }
  // For 
  // TODO Modify the structure j below
  double px = 0;
  double py = 0;
  double psi = 0;
  double v = 0;

  Eigen::VectorXd ptsx = Eigen::VectorXd::Map(ptsxd.data(),ptsxd.size());
  Eigen::VectorXd ptsy = Eigen::VectorXd::Map(ptsyd.data(),ptsyd.size());
  auto coeffs = polyfit(ptsx, ptsy, 3);
  for (size_t i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;
  
    // The cross track error is calculated by evaluating at polynomial at x, f(x)
    // and subtracting y.
    double cte = polyeval(coeffs, px) - py;
    // Due to the sign starting at 0, the orientation error is -f'(x).
    // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
    double epsi = psi - atan(coeffs[1]);
    Eigen::VectorXd state(6);
    state << px, py, psi, v, cte, epsi;
    auto vars = mpc.Solve(state, coeffs);
    double steer_value = vars[6]/deg2rad(25);
    double throttle_value = vars[7];
  }
}
