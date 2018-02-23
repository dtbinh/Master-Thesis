

% update the rigid body Coriolis matrix

C_RB1=tau_RB_cor(veh,v_d_1');


C_RB2=tau_RB_cor(veh,v_d_2');


C_RB3=tau_RB_cor(veh,v_d_3');


C_RB4=tau_RB_cor(veh,v_d_4');


C_RB5=tau_RB_cor(veh,v_d_5');


C_RB6=tau_RB_cor(veh,v_d_6');


C_RB7=tau_RB_cor(veh,v_d_7');

% after the restoring force term

% remember the structure of the 

% function g = gvect(W,B,theta,phi,r_g,r_b)

% actually the only parameter that should be changed is the fifth parameter
% veh.G_b
% The center of mass CG will inluence by different geometric configurations



G1=gvect(veh.G,veh.B,pitch_d_1,roll_d_1,veh.G_b,veh.B_b);

G2=gvect(veh.G,veh.B,pitch_d_2,roll_d_2,veh.G_b,veh.B_b);

G3=gvect(veh.G,veh.B,pitch_d_3,roll_d_3,veh.G_b,veh.B_b);

G4=gvect(veh.G,veh.B,pitch_d_4,roll_d_4,veh.G_b,veh.B_b);

G5=gvect(veh.G,veh.B,pitch_d_5,roll_d_5,veh.G_b,veh.B_b);

G6=gvect(veh.G,veh.B,pitch_d_6,roll_d_6,veh.G_b,veh.B_b);

G7=gvect(veh.G,veh.B,pitch_d_7,roll_d_7,veh.G_b,veh.B_b);

%% now let us calculate the new desired input 

% The desired input has four components tau_d

tau_d_1=C_RB1+C_A1+D1+G1;

tau_d_2=C_RB2+C_A2+D2+G2;

tau_d_3=C_RB3+C_A3+D3+G3;

tau_d_4=C_RB4+C_A4+D4+G4;

tau_d_5=C_RB5+C_A5+D5+G5;

tau_d_6=C_RB6+C_A6+D6+G6;

tau_d_7=C_RB7+C_A7+D7+G7;

%% Desired input for each thruster and for each trim trajectory

%% Optimization for the control input signal 

% formulate the optimization problem as bounded least square optimization problem  

% small angle approximation: 0.2*57.3=11.46 degree


l_bound_input=[-35;-35;-1;-1;-1;-1];
%l_bound_input=[-1000;-1000;-1000;-1000;-1000;-1000];

% the upper bound is the maximal thrust the thruster can generate

u_bound_input=[40;40;1;1;1;1];
%u_bound_input=[1000;1000;1000;1000;1000;1000];
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
minimize(norm(B_input_1*u_d_1-tau_d_1))
subject to
l_bound_input<= u_d_1 <= u_bound_input
cvx_end

% second trim trajectory
cvx_begin
variable u_d_2(6)
minimize(norm(B_input_2*u_d_2-tau_d_2))
subject to
l_bound_input<= u_d_2 <= u_bound_input
cvx_end

% third trim trajectory
cvx_begin
variable u_d_3(6)
minimize(norm(B_input_3*u_d_3-tau_d_3))
subject to
l_bound_input<= u_d_3 <= u_bound_input
cvx_end

% fourth trim trajectory
cvx_begin
variable u_d_4(6)
minimize(norm(B_input_4*u_d_4-tau_d_4))
subject to
l_bound_input<= u_d_4 <= u_bound_input
cvx_end

% fifth trim trajectory
cvx_begin
variable u_d_5(6)
minimize(norm(B_input_5*u_d_5-tau_d_5))
subject to
l_bound_input<= u_d_5 <= u_bound_input
cvx_end

% sixth trim trajectory
cvx_begin
variable u_d_6(6)
minimize(norm(B_input_6*u_d_6-tau_d_6))
subject to
l_bound_input<= u_d_6 <= u_bound_input
cvx_end

% seven trim trajectory
cvx_begin
variable u_d_7(6)
minimize(norm(B_input_7*u_d_7-tau_d_7))
subject to
l_bound_input<= u_d_7 <= u_bound_input
cvx_end
