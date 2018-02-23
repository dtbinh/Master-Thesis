%% in this file we take the configuration of the current actuators and build the error dynamics 
%  and the design the controller

%% The whole complete error dynamics including 12 state variables
% state variables:
% nu:(u,v,w),omega:(p,q,r),p:(x,y,z),lambda:(phi,theta,psi)
% Dimension: 
% A_E: 12 x 12
% B_E: 12 x 6


%% build the optimal error dynamics

% % debug
% 
% b_T1_optimized=veh.b_T1;
% b_T2_optimized=veh.b_T2;
% b_T3_optimized=veh.b_T3;
% b_T4_optimized=veh.b_T4;
% b_T5_optimized=veh.b_T5;
% b_T6_optimized=veh.b_T6;

%% Calculate the optimal desired input for all trim trajectory segments

r_g_optimized=CenterofMass({m_H,r_H},{m_T1,r_T1_optimized},{m_T2,r_T2_optimized},{m_T3,r_T3_optimized},{m_T4,r_T4_optimized},{m_T5,r_T5_optimized},{m_T6,r_T6_optimized});

% update the center of mass in the global variable veh

veh.G_b=r_g_optimized;

% now let us update the rigid body inertia Matrix since the third parameter
% r_G is updated with the new one r_g_new

% Note that the Moment of Inertia changes too

% However we assume that the Moment of Inertia is only determined by the
% hull, hence it is fixed during the second optimization phase 

Mrb_optimized=RigidBodyInertiaMatrix(veh.m,veh.iota,r_g_optimized);

% then we update the veh.Mrb by Mrb_new

veh.Mrb=Mrb_optimized;
 
%% F_kin

F_lambda=-inv(Mrb_optimized+veh.Ma)*g;

% divide the kinematic part in dynamic equation into two parts (linear velocity part and angular velocity part)

F_v_lambda=F_lambda(1:3,:);

F_w_lambda=F_lambda(4:6,:);

%% F_dyn

F=-inv(Mrb_optimized+veh.Ma)*(CRB+CA+Dg);

% divide the dynamic part in dynamic equation into two parts (linear velocity part and angular velocity part) 

F_v=F(1:3,:);

F_w=F(4:6,:);


% the optimal configuration from the result 

T_S_1_optimized={r_T1_optimized,b_T1_optimized,d_T1_optimized};
T_S_2_optimized={r_T2_optimized,b_T2_optimized,d_T2_optimized}; 
T_S_3_optimized={r_T3_optimized,b_T3_optimized,d_T3_optimized};
T_S_4_optimized={r_T4_optimized,b_T4_optimized,d_T4_optimized};
T_S_5_optimized={r_T5_optimized,b_T5_optimized,d_T5_optimized};
T_S_6_optimized={r_T6_optimized,b_T6_optimized,d_T6_optimized};

% use the optimal configuration to build the optimal dynamics

B_T=ThrusterConfigurationMatrix(T_S_1_optimized,T_S_2_optimized,T_S_3_optimized,T_S_4_optimized,T_S_5_optimized,T_S_6_optimized);

% put all thruster configuration matrices with different spin directions 

B_input=B_T;

% we update BH at first 

BH=inv(Mrb_optimized+veh.Ma)*B_input*[T1;T2;T3;T4;T5;T6];

BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 T3 T4 T5 T6]);


B_w_l=jacobian(BH_w,[T1 T2 T3 T4 T5 T6]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);


%% the first trimming trajectory

% stack all constant velocities into compact vectors
nu_C_1=[linear_velocity_x_1(1);linear_velocity_y_1(1);linear_velocity_z_1(1)];
omega_C_1=[angular_velocity_roll_1(1);angular_velocity_pitch_1(1);angular_velocity_yaw_1(1)];

% error dynamics for position
A_p_v_1=eye(3);
A_p_w_1=zeros(3);
A_p_p_1=-vp(omega_C_1);
A_p_lambda_1=-vp(nu_C_1);

% error dynamics for orientation
A_lambda_v_1=zeros(3);
A_lambda_w_1=eye(3);
A_lambda_p_1=zeros(3);
A_lambda_lambda_1=-vp(omega_C_1);

% error dynamics for linear velocities

A_v_v_1=A_v_v_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

A_v_w_1=A_v_w_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

B_v=B_v_l;

A_v_p_1=zeros(3);

A_v_lambda_1=zeros(3);


% error dynamics for angular velocities

A_w_v_1=A_w_v_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

A_w_w_1=A_w_w_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

B_w=B_w_l; 

A_w_p_1=zeros(3);

A_w_lambda_1=zeros(3);

% formulate the error dynamics
A_E_1=[A_v_v_1 A_v_w_1 A_v_p_1 A_v_lambda_1;A_w_v_1 A_w_w_1 A_w_p_1 A_w_lambda_1;A_p_v_1 A_p_w_1 A_p_p_1 A_p_lambda_1;A_lambda_v_1 A_lambda_w_1 A_lambda_p_1 A_lambda_lambda_1];
B_E_1=[B_v_1;B_w_1;zeros(6)];

A_E_1=double(A_E_1);
B_E_1=double(B_E_1);

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.001 0.001 0.001 0.001 0.001 0.001]);
[K1,P1,E1]=lqr(A_E_1,B_E_1,Q,R);

%% the second trimming trajectory

% stack all constant velocities into compact vectors
nu_C_2=[linear_velocity_x_2(1);linear_velocity_y_2(1);linear_velocity_z_2(1)];
omega_C_2=[angular_velocity_roll_2(1);angular_velocity_pitch_2(1);angular_velocity_yaw_2(1)];

% error dynamics for position
A_p_v_2=eye(3);
A_p_w_2=zeros(3);
A_p_p_2=-vp(omega_C_2);
A_p_lambda_2=-vp(nu_C_2);

% error dynamics for orientation
A_lambda_v_2=zeros(3);
A_lambda_w_2=eye(3);
A_lambda_p_2=zeros(3);
A_lambda_lambda_2=-vp(omega_C_2);

% error dynamics for linear velocities

A_v_v_2=A_v_v_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

A_v_w_2=A_v_w_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

B_v_2=B_v_l;

A_v_p_2=zeros(3);

A_v_lambda_2=zeros(3);


% error dynamics for angular velocities

A_w_v_2=A_w_v_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

A_w_w_2=A_w_w_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

B_w_2=B_w_l;

A_w_p_2=zeros(3);

A_w_lambda_2=zeros(3);

% formulate the error dynamics
A_E_2=[A_v_v_2 A_v_w_2 A_v_p_2 A_v_lambda_2;A_w_v_2 A_w_w_2 A_w_p_2 A_w_lambda_2;A_p_v_2 A_p_w_2 A_p_p_2 A_p_lambda_2;A_lambda_v_2 A_lambda_w_2 A_lambda_p_2 A_lambda_lambda_2];
B_E_2=[B_v_2;B_w_2;zeros(6)];

A_E_2=double(A_E_2);
B_E_2=double(B_E_2);

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.0001 0.0001 0.0001 0.0001 0.0001 0.0001]);
[K2,P2,E2]=lqr(A_E_2,B_E_2,Q,R);



