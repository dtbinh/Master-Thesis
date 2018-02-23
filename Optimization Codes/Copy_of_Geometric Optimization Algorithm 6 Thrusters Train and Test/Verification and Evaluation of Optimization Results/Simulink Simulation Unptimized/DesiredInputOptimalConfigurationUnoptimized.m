%% in this file we want to build the initial dynamics and the intial desired input to verify the system is optimized

%% Calculate the initial desired input for all trim trajectory segments

r_g_init=CenterofMass({m_H,r_H},{m_T1,r_T1_init},{m_T2,r_T2_init},{m_T3,r_T3_init},{m_T4,r_T4_init},{m_T5,r_T5_init},{m_T6,r_T6_init});

% Note that the Moment of Inertia changes too

% However we assume that the Moment of Inertia is only determined by the
% hull, hence it is fixed during the second optimization phase 

Mrb_init=RigidBodyInertiaMatrix(veh.m,veh.iota,r_g_init);

% assign the intial rigid body mass matrix back to the global vehicle
% variable veh in order to calculate the following initial Coriolis matrix

veh.Mrb=Mrb_init;

% update the rigid body Coriolis matrix

C_RB1_init=tau_RB_cor(veh,v_d_1');


C_RB2_init=tau_RB_cor(veh,v_d_2');


C_RB3_init=tau_RB_cor(veh,v_d_3');


C_RB4_init=tau_RB_cor(veh,v_d_4');


C_RB5_init=tau_RB_cor(veh,v_d_5');


C_RB6_init=tau_RB_cor(veh,v_d_6');


C_RB7_init=tau_RB_cor(veh,v_d_7');

% after the restoring force term

% remember the structure of the 

% function g = gvect(W,B,theta,phi,r_g,r_b)

% actually the only parameter that should be changed is the fifth parameter
% veh.G_b
% The center of mass CG will inluence by different geometric configurations



G1_init=gvect(veh.G,veh.B,pitch_d_1,roll_d_1,r_g_init,veh.B_b);

G2_init=gvect(veh.G,veh.B,pitch_d_2,roll_d_2,r_g_init,veh.B_b);

G3_init=gvect(veh.G,veh.B,pitch_d_3,roll_d_3,r_g_init,veh.B_b);

G4_init=gvect(veh.G,veh.B,pitch_d_4,roll_d_4,r_g_init,veh.B_b);

G5_init=gvect(veh.G,veh.B,pitch_d_5,roll_d_5,r_g_init,veh.B_b);

G6_init=gvect(veh.G,veh.B,pitch_d_6,roll_d_6,r_g_init,veh.B_b);

G7_init=gvect(veh.G,veh.B,pitch_d_7,roll_d_7,r_g_init,veh.B_b);

%% now let us calculate the new desired input 

% The desired input has four components tau_d

tau_d_1_init=C_RB1_init+C_A1+D1+G1_init;

tau_d_2_init=C_RB2_init+C_A2+D2+G2_init;

tau_d_3_init=C_RB3_init+C_A3+D3+G3_init;

tau_d_4_init=C_RB4_init+C_A4+D4+G4_init;

tau_d_5_init=C_RB5_init+C_A5+D5+G5_init;

tau_d_6_init=C_RB6_init+C_A6+D6+G6_init;

tau_d_7_init=C_RB7_init+C_A7+D7+G7_init;

%% Desired input for each thruster and for each trim trajectory

% avoid misunderstanding from the desired control input in the optimization phase  

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
variable u_T_d_1_init(6)
minimize(norm(B_input*u_T_d_1_init-tau_d_1_init))
subject to
l_bound_input<= u_T_d_1_init <= u_bound_input
cvx_end

% second trim trajectory
cvx_begin
variable u_T_d_2_init(6)
minimize(norm(B_input*u_T_d_2_init-tau_d_2_init))
subject to
l_bound_input<= u_T_d_2_init <= u_bound_input
cvx_end

% third trim trajectory
cvx_begin
variable u_T_d_3_init(6)
minimize(norm(B_input*u_T_d_3_init-tau_d_3_init))
subject to
l_bound_input<= u_T_d_3_init <= u_bound_input
cvx_end

% fourth trim trajectory
cvx_begin
variable u_T_d_4_init(6)
minimize(norm(B_input*u_T_d_4_init-tau_d_4_init))
subject to
l_bound_input<= u_T_d_4_init <= u_bound_input
cvx_end

% fifth trim trajectory
cvx_begin
variable u_T_d_5_init(6)
minimize(norm(B_input*u_T_d_5_init-tau_d_5_init))
subject to
l_bound_input<= u_T_d_5_init <= u_bound_input
cvx_end

% sixth trim trajectory
cvx_begin
variable u_T_d_6_init(6)
minimize(norm(B_input*u_T_d_6_init-tau_d_6_init))
subject to
l_bound_input<= u_T_d_6_init <= u_bound_input
cvx_end

% seven trim trajectory
cvx_begin
variable u_T_d_7_init(6)
minimize(norm(B_input*u_T_d_7_init-tau_d_7_init))
subject to
l_bound_input<= u_T_d_7_init <= u_bound_input
cvx_end

