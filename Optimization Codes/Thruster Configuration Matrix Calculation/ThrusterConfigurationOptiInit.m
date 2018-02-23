%% Initialization for the optimization 

% Note that for the trim trajectory segment, the six velocities and the six inputs are constant
% For trim trjactory, roll angle phi and pitch angle theta are constant

%% calculate the gravity and buoyancy force

% W= veh.m*veh.g;
% B=veh.rho*veh.g*veh.V;
% 
% % evaluate the center of mass and center of buoyancy
% r_g=veh.G_b';
% r_b=veh.B_b';



%% the first trimming trajectory

% we should calculate the linearization points 

% velocity evaluation

u_l_1=linear_velocity_x_1(1);
v_l_1=linear_velocity_y_1(1);
w_l_1=linear_velocity_z_1(1);

p_l_1=angular_velocity_roll_1(1);
q_l_1=angular_velocity_pitch_1(1);
r_l_1=angular_velocity_yaw_1(1);

% input evaluation

% X_l=tau_d_1(1,1,1);
% Y_l=tau_d_1(2,1,1);
% Z_l=tau_d_1(3,1,1);
% K_l=tau_d_1(4,1,1);
% M_l=tau_d_1(5,1,1);
% N_l=tau_d_1(6,1,1);



% kinematic variable(Euler angle) evaluation

phi_l_1=roll_angle_1;

theta_l_1=pitch_angle_1;

psi_l_1=yaw_angle_1;

% evaluate the linearized state space matrix  

A_v_v_1=A_v_v_l(u_l_1,v_l_1,w_l_1,p_l_1,q_l_1,r_l_1);

A_v_w_1=A_v_w_l(u_l_1,v_l_1,w_l_1,p_l_1,q_l_1,r_l_1);

% B_v_1=eval(B_v_l); 
% The input matrix B_v_l should be evaluated in the optimization phase

A_w_v_1=eval(A_w_v_l);

A_w_w_1=eval(A_w_w_l);

% B_w_1=eval(B_w_l);
% The input matrix B_w_l should be evaluated in the optimization phase 
% because the configuration matrix is always different for different
% actuator configuration 

%% the second trimming trajectory

u_l_2=linear_velocity_x_2(1);
v_l_2=linear_velocity_y_2(1);
w_l_2=linear_velocity_z_2(1);

p_l_2=angular_velocity_roll_2(1);
q_l_2=angular_velocity_pitch_2(1);
r_l_2=angular_velocity_yaw_2(1);

% X_l=tau_d_2(1,1,1);
% Y_l=tau_d_2(2,1,1);
% Z_l=tau_d_2(3,1,1);
% K_l=tau_d_2(4,1,1);
% M_l=tau_d_2(5,1,1);
% N_l=tau_d_2(6,1,1);

% evaluate the linearized state space matrix

A_v_v_2=A_v_v_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);

A_v_w_2=A_v_w_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);

% B_v_2=eval(B_v_l);

A_w_v_2=A_v_v_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);

A_w_w_2=A_w_w_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);

% B_w_2=eval(B_w_l);

%% the third trimming trajectory

u_l_3=linear_velocity_x_3(1);
v_l_3=linear_velocity_y_3(1);
w_l_3=linear_velocity_z_3(1);

p_l_3=angular_velocity_roll_3(1);
q_l_3=angular_velocity_pitch_3(1);
r_l_3=angular_velocity_yaw_3(1);

% X_l=tau_d_3(1,1,1);
% Y_l=tau_d_3(2,1,1);
% Z_l=tau_d_3(3,1,1);
% K_l=tau_d_3(4,1,1);
% M_l=tau_d_3(5,1,1);
% N_l=tau_d_3(6,1,1);


A_v_v_3=A_v_v_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);

A_v_w_3=A_v_w_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);

% B_v_3=eval(B_v_l);

A_w_v_3=A_w_v_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);

A_w_w_3=A_w_w_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);

% B_w_3=eval(B_w_l);

%% the fourth trimming trajectory

u_l_4=linear_velocity_x_4(1);
v_l_4=linear_velocity_y_4(1);
w_l_4=linear_velocity_z_4(1);

p_l_4=angular_velocity_roll_4(1);
q_l_4=angular_velocity_pitch_4(1);
r_l_4=angular_velocity_yaw_4(1);

% X_l=tau_d_4(1,1,1);
% Y_l=tau_d_4(2,1,1);
% Z_l=tau_d_4(3,1,1);
% K_l=tau_d_4(4,1,1);
% M_l=tau_d_4(5,1,1);
% N_l=tau_d_4(6,1,1);


A_v_v_4=A_v_v_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);

A_v_w_4=A_v_w_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);

% B_v_4=eval(B_v_l);

A_w_v_4=A_w_v_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);
A_w_w_4=A_w_w_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);

% B_w_4=eval(B_w_l);

%% the fifth trimming trajectory

u_l_5=linear_velocity_x_5(1);
v_l_5=linear_velocity_y_5(1);
w_l_5=linear_velocity_z_5(1);

p_l_5=angular_velocity_roll_5(1);
q_l_5=angular_velocity_pitch_5(1);
r_l_5=angular_velocity_yaw_5(1);

% X_l=tau_d_5(1,1,1);
% Y_l=tau_d_5(2,1,1);
% Z_l=tau_d_5(3,1,1);
% K_l=tau_d_5(4,1,1);
% M_l=tau_d_5(5,1,1);
% N_l=tau_d_5(6,1,1);


A_v_v_5=A_v_v_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);

A_v_w_5=A_v_w_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);

% B_v_5=eval(B_v_l);

A_w_v_5=A_w_v_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);

A_w_w_5=A_w_w_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);

% B_w_5=eval(B_w_l);


%% the sixth trimming trajectory

u_l_6=linear_velocity_x_6(1);
v_l_6=linear_velocity_y_6(1);
w_l_6=linear_velocity_z_6(1);

p_l_6=angular_velocity_roll_6(1);
q_l_6=angular_velocity_pitch_6(1);
r_l_6=angular_velocity_yaw_6(1);

% X_l=tau_d_6(1,1,1);
% Y_l=tau_d_6(2,1,1);
% Z_l=tau_d_6(3,1,1);
% K_l=tau_d_6(4,1,1);
% M_l=tau_d_6(5,1,1);
% N_l=tau_d_6(6,1,1);


A_v_v_6=A_v_v_l(u_l_6,v_l_6,w_l_6,p_l_6,q_l_6,r_l_6);

A_v_w_6=A_v_w_l(u_l_6,v_l_6,w_l_6,p_l_6,q_l_6,r_l_6);

% B_v_6=eval(B_v_l);

A_w_v_6=A_w_v_l(u_l_6,v_l_6,w_l_6,p_l_6,q_l_6,r_l_6);

A_w_w_6=A_w_w_l();

% B_w_6=eval(B_w_l);


%% the seventh trimming trajectory

% assign the value of dynamics states from the trim 

u_l_7=linear_velocity_x_7(1);
v_l_7=linear_velocity_y_7(1);
w_l_7=linear_velocity_z_7(1);

p_l_7=angular_velocity_roll_7(1);
q_l_7=angular_velocity_pitch_7(1);
r_l_7=angular_velocity_yaw_7(1);

% X_l=tau_d_7(1,1,1);
% Y_l=tau_d_7(2,1,1);
% Z_l=tau_d_7(3,1,1);
% K_l=tau_d_7(4,1,1);
% M_l=tau_d_7(5,1,1);
% N_l=tau_d_7(6,1,1);


A_v_v_7=A_v_v_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);

A_v_w_7=A_v_w_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);

% B_v_7=eval(B_v_l);

A_w_v_7=A_w_v_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);

A_w_w_7=A_w_w_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);

% B_w_7=eval(B_w_l);

