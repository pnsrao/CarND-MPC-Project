# The Model
The global kinematic model is used to describe vehicle motion and evolution of the state variables. 
Vehicle position, orientation angle, velocity, cross track error and angle errors are used as state variables. 
Steering angle and acceleration (throttle) are the actuators.
The equations of the model in the form of the solver constraints are described below:
~~~~
      // Recall the equations for the model:
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
~~~~
Note that the simulator interprets the steering actuator with the opposite sign to the convention in the class.

# Timestep Length and Elapsed Duration

The timestep duration (dt) should be small enough   for the model to accurately predict the motion of the car, 
Especially if the track curves a lot, the actuator values mustreact fast enough for the car to navigate the track.

The number of timesteps influences the computational complexity of the Solver. Larger N implies a bigger problem size for the solver 
and a larger computation time.  Too small a N implies that you do not see far enough into the track for a good solution.

I tried a value of 1 second for dt during initial debugging and the car was behaving erratically. I then settled on a 
value of 0.1s which seemed to work well.

Similarly I tried N=25 initially. While the car seemed to drive reasonably most of the time, there were occasions when the 
predicted path seemed to be completely incorrect, possibly because the solver did not finish in time. Decreasing it to N=10
seemed to work better.

# Polynomial Fitting and MPC Preprocessing
A third order polynomial was fitted to the waypoints.

Before this step, the way points were preprocssed to transform the global coordinates to car coordinates with the car 
positioned at (0,0) and oriented facing the positive X-direction. This helped significantly in calculating the CTE 
as well as the the angle error
~~~~
	  for(int i = 0; i < ptsxd.size(); i++) {
	    //shift car reference angle to 90 degrees
	    double shift_x = ptsxd[i] - px;
	    double shift_y = ptsyd[i] - py;
	    ptsxd[i] = shift_x * cos(0-psi)-shift_y*sin(0-psi);
	    ptsyd[i] = shift_x * sin(0-psi)+shift_y*cos(0-psi);
	  }
~~~~

# Model Predictive Control with Latency
In order to factor in the impact of latency, the kinematic model was used to predict the position, direction and 
velocity of the car 100ms ahead before starting the entire MPC procedure.
Since we know that there is a 100ms delay in applying the result, we instead use the predictes state 100ms ahead as the initial state 
provided to the MPC algorithm.
~~~~
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
~~~~
