/*******************************************
 ************ Reference State **************
 *******************************************/

State vector: cte and eψ
Both should be 0

double cost = 0;
for (int t = 0; t < n; t++) {
	Cost += pow(cte[t], 2);
	Cost += pow(epsi[t]t, 2);
}

/*******************************************
 ********** Dealing With Stopping **********
 *******************************************/

cost += pow(v[t] - 25, 2)

euclidean Distance
(Xdes, Ydes)

/*******************************************
 ****** Additional Cost Conststraints ******
 *******************************************/

Additional Cost Contstraints

for(int t = 0; t < N-1; t++) {
	cost += pow(delta[t], 2)
}

for (int t = 0; t < N-1; t++) {
  cost += pow(delta[t+1] - delta[t], 2)
  cost += pow(a[t+1] - a[t], 2)
}

/*******************************************
 *********** Length and Duration ***********
 *******************************************/

// The prediction horizon is the duration over which future predictions are made. 
// We’ll refer to this as T.

// T is the product of two other variables, N and dt.

// N is the number of timesteps in the horizon. dt is how much time elapses between actuations. 
// For example, if N were 20 and dt were 0.5, then T would be 10 seconds.

// N, dt, and T are hyperparameters you will need to tune for each model predictive controller 
// you build. However, there are some general guidelines. T should be as large as possible, 
// while dt should be as small as possible.

// These guidelines create tradeoffs.

//
// ******************* Horizon **********************
//

// In the case of driving a car, T should be a few seconds, at most. Beyond that horizon, 
// the environment will change enough that it won't make sense to predict any further into 
// the future.

//
// ************** Number of Timesteps ***************
//

// The goal of Model Predictive Control is to optimize the control inputs: [δ,a]. An optimizer 
// will tune these inputs until a low cost vector of control inputs is found. The length of this 
// vector is determined by N:

[δ1, a1,δ2,a2,...,δN−1,aN−1]

// Thus N determines the number of variables optimized by the MPC. This is also the major driver 
// of computational cost.

//
// ************** Timestep Duration ***************
//

// MPC attempts to approximate a continues reference trajectory by means of discrete paths between 
// actuations. Larger values of dt result in less frequent actuations, which makes it harder to 
// accurately approximate a continuous reference trajectory. This is sometimes called 
// "discretization error".

// A good approach to setting N, dt, and T is to first determine a reasonable range for T and 
// then tune dt and N appropriately, keeping the effect of each in mind.

/*******************************************
 ********* Putting It All Together *********
 *******************************************/

// Model Predictive Control (Loop)

// State
[X1, Y1, ψ1, V1, cte1, eψ1]

// Solver

// Model
Xt + 1 = Xt + Vt * cos(ψt) * dt
yt + 1 = yt + Vt * sin(ψt) * dt
ψt + 1 = ψt + ((Vt)/(Lf)) * δt * dt
Vt + 1 = Vt + at * dt
ctet + 1 = f(Xt) - yt + Vt * sin(eψt) * dt
eψt + 1 = ψt + ψdes t + ((Vt)/(Lf)) * δt * dt

// Constraints
δε[-25, 25]
aε[-1,1]

// Cost
J = sigma(cte t – cte_ref)^2 + (eψt – eψ_ref)^2 + …

// ->
[  δ1      a1  ]
[  δ2      a2  ]
[   .       .  ]
[ δN-1     aN-1]

/*******************************************
 **************** Latency ******************
 *******************************************/

// In a real car, an actuation command won't execute instantly - there will be a delay as the 
// command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

// This is a problem called "latency", and it's a difficult challenge for some controllers - like 
// a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because 
// we can model this latency in the system.

//
// ************** PID Controller ***************
//

// PID controllers will calculate the error with respect to the present state, but the actuation 
// will be performed when the vehicle is in a future (and likely different) state. This can 
// sometimes lead to instability.

// The PID controller could try to compute a control input based on a future error, but without 
// a vehicle model it's unlikely this will be accurate.

//
// ************** Model Predictive Control***************
//

// A contributing factor to latency is actuator dynamics. For example the time elapsed between 
// when you command a steering angle to when that angle is actually achieved. 
// This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. 
// One approach would be running a simulation using the vehicle model starting from the current 
// state for the duration of the latency. The resulting state from the simulation is the new 
// initial state for MPC.

// Thus, MPC can deal with latency much more effectively, by explicitly taking it into account, 
// than a PID controller.

// Next we'll get the hang of implementing MPC!

/*******************************************
 ************* Mind The Line ***************
 *******************************************/

// In this quiz you'll use MPC to follow the trajectory along a line.

// Steps:

// 1. Set N and dt.
// 2. Fit the polynomial to the waypoints.
// 3. Calculate initial cross track error and orientation error values.
// 4. Define the components of the cost function (state, actuators, etc). You may use the methods 
//    previously discussed or make up something, up to you!
// 5. Define the model constraints. These are the state update equations defined in the Vehicle 
//    Models module.

// Before you begin let's go over the libraries you'll use for this quiz and the following project.

//
// ************** Ipopt ***************
//

// Ipopt is the tool we'll be using to optimize the control inputs [δ1,a1,...,δN−1,aN−1]. 
// It's able to find locally optimal values (non-linear problem!) while keeping the constraints 
// set directly to the actuators and the constraints defined by the vehicle model. Ipopt requires 
// we give it the jacobians and hessians directly - it does not compute them for us. Hence, we 
// need to either manually compute them or have have a library do this for us. Luckily, there 
// is a library called CppAD which does exactly this.

//
// ************** CppAD ***************
//

// CppAD is a library we'll use for automatic differentiation. By using CppAD we don't have to 
// manually compute derivatives, which is tedious and prone to error.
// In order to use CppAD effectively, we have to use its types instead of regular double or 
// std::vector types.
// Additionally math functions must be called from CppAD. Here's an example of calling pow:

CppAD::pow(x, 2);
// instead of 
pow(x, 2);

// Luckily most elementary math operations are overloaded. So calling *, +, -, / will work as 
// intended as long as it's called on CppAD<double> instead of double. Most of this is done for 
// you and there are examples to draw from in the code we provide.

//
// ************** Code Structure ***************
//

// We've filled in most of the quiz starter code for you. The goal of this quiz is really just 
// about getting everything to work as intended.
// That said, it may be tricky to decipher some elements of the starter code, so we will walk 
// you through it.
// There are two main components in MPC.cpp:

//	1. 
			vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) method
//	2.
			FG_eval class

//
// ************** MPC::Solve ***************
//

// x0 is the initial state [x,y,ψ,v,cte,eψ], coeffs are the coefficients of the fitting polynomial. 
// The bulk of this method is setting up the vehicle model constraints (constraints) and variables 
// (vars) for Ipopt.

// Variables
double x = x0[0];
double y = x0[1];
double psi = x0[2];
double v = x0[3];
double cte = x0[4];
double epsi = x0[5];
...
// Set the initial variable values
vars[x_start] = x;
vars[y_start] = y;
vars[psi_start] = psi;
vars[v_start] = v;
vars[cte_start] = cte;
vars[epsi_start] = epsi;

// Note Ipopt expects all the constraints and variables as vectors. For example, suppose N is 5, 
// then the structure of vars a 38-element vector:

vars[0] ,...,vars[4]  -> [x1,....,x5]
vars[5] ,...,vars[9]  -> [y1,....,y5]
vars[10],...,vars[14] -> [ψ1,....,ψ5]
vars[15],...,vars[19] -> [v1,....,v5]
vars[20],...,vars[24] -> [cte1,....,cte5]
vars[25],...,vars[29] -> [eψ1,....,eψ5]
vars[30],...,vars[33] -> [δ1,....,δ4]
vars[34],...,vars[37] -> [a1,....,a4]

// We then set lower and upper bounds on the variables. Here we set the range of values δ 
// to [-25, 25] in radians:

for (int i = delta_start; i < a_start; i++) {
	vars_lowerbound[i] = -0.436332;
  vars_upperbound[i] = 0.436332;
}

// Constraints
// Next the we set the lower and upper bounds on the constraints.

// Consider, for example:

xt+1 = xt + vt ∗ cos(ψt) ∗ dt

// This expresses that xt+1 MUST be equal to xt + vt ∗ cos(ψt) ∗ dt. Put differently: 

xt+1 − (xt + vt ∗ cos(ψt) ∗ dt) = 0

// The equation above simplifies the upper and lower bounds of the constraint: both must be 0.
// This can be generalized to the other equations as well:

for (int i = 0; i < n_constraints; i++) {
  constraints_lowerbound[i] = 0;
  constraints_upperbound[i] = 0;
}

// FG_eval
// The FG_eval class has the constructor:

FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

// where coeffs are the coefficients of the fitted polynomial. coeffs will be used by the cross 
// track error and heading error equations.

// The FG_eval class has only one method:

void operator()(ADvector& fg, const ADvector& vars)

// vars is the vector of variables (from the previous section) and fg is the vector of constraints.
// One complication: fg[0] stores the cost value, so the fg vector is 1 element larger than it 
// was in MPC::Solve.
// Here in operator() you'll define the cost function and constraints. x is already completed:

for (int t = 1; t < N; t++) {
	AD<double> x1 = vars[x_start + t];

	AD<double> x0 = vars[x_start + t - 1];
	AD<double> psi0 = vars[psi_start + t - 1];
	AD<double> v0 = vars[v_start + t - 1];

	// Here's `x` to get you started.
	// The idea here is to constraint this value to be 0.
	//
	// NOTE: The use of `AD<double>` and use of `CppAD`!
	// This is also CppAD can compute derivatives and pass
	// these to the solver.

	// TODO: Setup the rest of the model constraints
	fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
}

// Note that we start the loop at t=1, because the values at t=0 are set to our initial 
// state - those values are not calculated by the solver.

// An FG_eval object is created in MPC::Solve:

FG_eval fg_eval(coeffs);

// This is then used by Ipopt to find the lowest cost trajectory:

// place to return solution
CppAD::ipopt::solve_result<Dvector> solution;

// solve the problem
CppAD::ipopt::solve<Dvector, FG_eval>(
	options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
	constraints_upperbound, fg_eval, solution);

// The filled in vars vector is stored as solution.x and the cost as solution.obj_value.

/*******************************************
 ******** Solution: Mind The Line **********
 *******************************************/

// Ok, let's walk through the solution. We'll go through each of the TODO tags.

//
// ************** N & dt ***************
//

// TODO: Set N and dt
size_t N = 25;
double dt = 0.05;

// Here we had to assign values to N and dt. It's likely you set these variables to slightly 
// different values. That's fine as long as the cross track error decreased to 0. It's a good 
// idea to play with different values here.

// For example, if we were to set N to 100, the simulation would run much slower. This is 
// because the solver would have to optimize 4 times as many control inputs. Ipopt, the solver, 
// permutes the control input values until it finds the lowest cost. If you were to open up Ipopt 
// and plot the x and y values as the solver mutates them, the plot would look like a worm moving 
// around trying to fit the shape of the reference trajectory.

//
// ************** Cost function ***************
//

void operator()(ADvector& fg, const ADvector& vars) {
	// The cost is stored is the first element of `fg`.
 	// Any additions to the cost should be added to `fg[0]`.
 	fg[0] = 0;

 	// Cost function
 	// TODO: Define the cost related the reference state and
 	// any anything you think may be beneficial.

 	// The part of the cost based on the reference state.
  for (int t = 0; t < N; t++) {
  	fg[0] += CppAD::pow(vars[cte_start + t], 2);
  	fg[0] += CppAD::pow(vars[epsi_start + t], 2);
  	fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
  }
  // Minimize the use of actuators.
  for (int t = 0; t < N - 1; t++) {
  	fg[0] += CppAD::pow(vars[delta_start + t], 2);
  	fg[0] += CppAD::pow(vars[a_start + t], 2);
  }
  // Minimize the value gap between sequential actuations.
  for (int t = 0; t < N - 2; t++) {
  	fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  	fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
  }

// There's a lot to unwind here.
// Let's start with the function arguments: fg and vars.
// The vector fg is where the cost function and vehicle model/constraints is defined. We'll go 
// the fg vector in more detail shortly.
// The other function argument is the vector vars. This vector contains all variables used by 
// the cost function and model:

[x,y,ψ,v,cte,eψ]

[δ,a]

// This is all one long vector, so if N is 25 then the indices are assigned as follows:

vars[0],   ..., vars[24]  -> x1,...,x25
vars[25],  ..., vars[49]  -> y1,...,y25
vars[50],  ..., vars[74]  -> ψ1,...,ψ25
vars[75],  ..., vars[99]  -> v1,...,v25
vars[100], ..., vars[124] -> cte1,...,cte25
vars[125], ..., vars[149] -> eψ1,...,eψ25
vars[150], ..., vars[173] -> δ1,...,δ24
vars[174], ..., vars[197] -> a1,...,a24

// Now let's focus on the actual cost function. Since 0 is the index at whichIpopt expects fg to 
// store the cost value, we sum all the components of the cost and store them at index 0.

// In each iteration through the loop, we sum three components to reach the aggregate cost: our 
// cross-track error, our heading error, and our velocity error.

// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
	fg[0] += CppAD::pow(vars[cte_start + t] , 2);
	fg[0] += CppAD::pow(vars[epsi_start + t], 2);
	fg[0] += CppAD::pow(vars[v_start + t], 2);
}

// We've already taken care of the main objective - to minimize our cross track, heading, and 
// velocity errors. A further enhancement is to constrain erratic control inputs.

// For example, if we're making a turn, we'd like the turn to be smooth, not sharp. Additionally, 
// the vehicle velocity should not change too radically.

// Minimize change-rate.
for (int t = 0; t < N - 1; t++) {
	fg[0] += CppAD::pow(vars[delta_start + t], 2);
	fg[0] += CppAD::pow(vars[a_start + t], 2);
}

// The goal of this final loop is to make control decisions more consistent, or smoother. 
// The next control input should be similar to the current one.

// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
	fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}

//
// ************** Initialization & constraints ***************
//

// We initialize the model to the initial state. Recall fg[0] is reserved for the cost value, 
// so the other indices are bumped up by 1.

fg[1 + x_start] = vars[x_start];
fg[1 + y_start] = vars[y_start];
fg[1 + psi_start] = vars[psi_start];
fg[1 + v_start] = vars[v_start];
fg[1 + cte_start] = vars[cte_start];
fg[1 + epsi_start] = vars[epsi_start];

// All the other constraints based on the vehicle model:

xt+1 = xt + vt ∗ cos(ψt) ∗ dt
yt+1 = yt + vt ∗ sin(ψt) ∗ dt
ψ​t+1 = ψt + vt/Lf ∗ δt ∗ dt
vt+1 = vt + at ∗ dt
ctet+1 = f(x​t) − yt + (vt ∗ sin(eψt) ∗ dt)
eψt+1 = ψt − ψdest + (vt/Lf ∗ δt ∗ dt)

// Let's look how to model ψ. Based on the above equations, we need to constrain the value of ψ 
// at time t+1:

ψt+1 = ψt + vt/Lf ∗ δt ∗ dt

// We do that by setting a value within fg to the difference of ps1 and the above formula.

// Previously, we have set the corresponding constraints_lowerbound and the constraints_upperbound 
// values to 0. That means the solver will force this value of fg to always be 0.

for (int t = 1; t < N ; t++) {
  // psi, v, delta at time t
  AD<double> psi0 = vars[psi_start + t - 1];
  AD<double> v0 = vars[v_start + t - 1];
  AD<double> delta0 = vars[delta_start + t - 1];

  // psi at time t+1
  AD<double> psi1 = vars[psi_start + t];

  // how psi changes
  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
}

// The oddest line above is probably fg[1 + psi_start + t].

// fg[0] stores the cost value, so there's always an offset of 1. So fg[1 + psi_start] is where 
// we store the initial value of ψ. Finally, fg[1 + psi_start + t] is reserved for the tth of N 
// values of ψ that the solver computes.

// Coding up the other parts of the model is similar.

for (int t = 1; t < N; t++) {
  // The state at time t+1 .
  AD<double> x1 = vars[x_start + t];
  AD<double> y1 = vars[y_start + t];
  AD<double> psi1 = vars[psi_start + t];
  AD<double> v1 = vars[v_start + t];
  AD<double> cte1 = vars[cte_start + t];
  AD<double> epsi1 = vars[epsi_start + t];

  // The state at time t.
  AD<double> x0 = vars[x_start + t - 1];
  AD<double> y0 = vars[y_start + t - 1];
  AD<double> psi0 = vars[psi_start + t - 1];
  AD<double> v0 = vars[v_start + t - 1];
  AD<double> cte0 = vars[cte_start + t - 1];
  AD<double> epsi0 = vars[epsi_start + t - 1];

  // Only consider the actuation at time t.
  AD<double> delta0 = vars[delta_start + t - 1];
  AD<double> a0 = vars[a_start + t - 1];

  AD<double> f0 = coeffs[0] + coeffs[1] * x0;
  AD<double> psides0 = CppAD::atan(coeffs[1]);

  // Here's `x` to get you started.
  // The idea here is to constraint this value to be 0.
  //
  // Recall the equations for the model:
  // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
  // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
  // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
  // v_[t] = v[t-1] + a[t-1] * dt
  // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
  // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
  fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
}

//
// ************** Fitting a polynomial to the waypoints ***************
//

// TODO: fit a polynomial to the above x and y coordinates
auto coeffs = polyfit(ptsx, ptsy, 1);

// The x and y coordinates are contained in the ptsx and ptsy vectors. Since these are 2-element 
// vectors a 1-degree polynomial (straight line) is sufficient.

//
// ************** Calculating the cross track and orientation error ***************
//

double x = -1;
double y = 10;
double psi = 0;
double v = 10;
// TODO: calculate the cross track error
double cte = polyeval(coeffs, x) - y;
// TODO: calculate the orientation error
double epsi = psi - atan(coeffs[1]);

// The cross track error is calculated by evaluating at polynomial at x (-1) and subtracting y.

// Recall orientation error is calculated as follows eψ=ψ−ψdes, where ψdes is can be calculated 
// as arctan(f′(x)).

f(x)  = a0 + a1 ∗ x 
f′(x) = a1

// hence the solution double epsi = psi - atan(coeffs[1]);

/*******************************************
 ************** Tuning MPC *****************
 *******************************************/

// The steering starts out at a min value -25 degrees and then jumps, causing a spike in the graph, 
// to 25 degrees. The second spike from 25 degrees to 0 degrees is more gradual but still sudden.

// While this experiment works out fine in our sample program, on an actual road the vehicle would 
// likely steer off the road and possibly crash. It would be a very unpleasant ride at best!

// An approach to solving this problem is tuning a part of the cost function affecting steering.

for (int i = 0; i < N - 2; i++) {
  // Tune this part!
  fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}

// Multiplying that part by a value > 1 will influence the solver into keeping sequential steering 
// values closer together.

fg[0] += 100 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);

fg[0] += 500 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);