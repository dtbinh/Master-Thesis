

% update the rigid body Coriolis matrix

C_RB1=tau_RB_cor(veh,v_d_1');


C_RB2=tau_RB_cor(veh,v_d_2');


% after the restoring force term

% remember the structure of the 

% function g = gvect(W,B,theta,phi,r_g,r_b)

% actually the only parameter that should be changed is the fifth parameter
% veh.G_b
% The center of mass CG will inluence by different geometric configurations



G1=gvect(veh.G,veh.B,pitch_d_1,roll_d_1,veh.G_b,veh.B_b);

G2=gvect(veh.G,veh.B,pitch_d_2,roll_d_2,veh.G_b,veh.B_b);


%% now let us calculate the new desired input 

% The desired input has four components tau_d

tau_d_1=C_RB1+C_A1+D1+G1;

tau_d_2=C_RB2+C_A2+D2+G2;


%% Desired input for each thruster and for each trim trajectory

%% Optimization for the control input signal 

% formulate the optimization problem as bounded least square optimization problem  

% small angle approximation: 0.2*57.3=11.46 degree


l_bound_input=[-35;-35;-35;-35;-35;-35];

% the upper bound is the maximal thrust the thruster can generate

u_bound_input=[40;40;40;40;40;40];

% our consideration is always to minimize the difference between the
% desired generalised force and the real generalised force under the
% current configuration

% minimization problem: unbounded least squares

% In this case, we have a unique configuration matrix for each trim
% trajectory segment, however, actually it is not the case.

% We have seven trim trajectory which means that we should have seven
% configuration matrices 


% note the difference between the following optimization problem and the
% desired input optimization 

% in the desired input optimization phase, we want to find the control
% input which minimize the difference between the desired generalised force
% and the generated actuator control input concurrently

% more precisely speaking, one optimal control input for all trim
% trajectories

% for the following optimization, one optimal control input for one trim
% trajectory segment

% first trim trajectory
cvx_begin
variable u_d_1(6)
minimize(norm(B_input*u_d_1-tau_d_1))
subject to
l_bound_input<= u_d_1 <= u_bound_input
cvx_end

% second trim trajectory
cvx_begin
variable u_d_2(6)
minimize(norm(B_input*u_d_2-tau_d_2))
subject to
l_bound_input<= u_d_2 <= u_bound_input
cvx_end


