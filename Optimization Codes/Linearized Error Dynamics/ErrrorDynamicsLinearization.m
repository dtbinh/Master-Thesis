%% The whole complete error dynamics including 12 state variables
% state variables:
% nu:(u,v,w),omega:(p,q,r),p:(x,y,z),lambda:(phi,theta,psi)
% Dimension: 
% A_E: 12 x 12
% B_E: 12 x 6

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
% parameters already defined in "LinearizationFormulasEvaluation"
% A_v_v
% A_v_w
% B_v
A_v_p_1=zeros(3);
A_v_lambda_1=zeros(3);


% error dynamics for angular velocities
% parameterd already defined in "LinearizationFormulasEvaluation"
% A_w_v
% A_w_w
% 
A_w_p_1=zeros(3);
A_w_lambda_1=zeros(3);

% formulate the error dynamics
A_E_1=[A_v_v_1 A_v_w_1 A_v_p_1 A_v_lambda_1;A_w_v_1 A_w_w_1 A_w_p_1 A_w_lambda_1;A_p_v_1 A_p_w_1 A_p_p_1 A_p_lambda_1;A_lambda_v_1 A_lambda_w_1 A_lambda_p_1 A_lambda_lambda_1];
B_E_1=[B_v_1;B_w_1;zeros(6)];

Q=diag([1000 1000 1000 1000 1000 1000 1 1 1 1 1 1]);
R=diag([0.1 0.1 0.1 0.1 0.1 0.1]);
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
% parameters already defined in "LinearizationFormulasEvaluation"
% A_v_v_2
% A_v_w_2
% B_v_2
A_v_p_2=zeros(3);
A_v_lambda_2=zeros(3);


% error dynamics for angular velocities
% parameterd already defined in "LinearizationFormulasEvaluation"
% A_w_v_2
% A_w_w_2
% w_2
A_w_p_2=zeros(3);
A_w_lambda_2=zeros(3);

% formulate the error dynamics
A_E_2=[A_v_v_2 A_v_w_2 A_v_p_2 A_v_lambda_2;A_w_v_2 A_w_w_2 A_w_p_2 A_w_lambda_2;A_p_v_2 A_p_w_2 A_p_p_2 A_p_lambda_2;A_lambda_v_2 A_lambda_w_2 A_lambda_p_2 A_lambda_lambda_2];
B_E_2=[B_v_2;B_w_2;zeros(6)];

Q=diag([1000 1000 1000 1000 1000 1000 1 1 1 1 1 1]);
R=diag([0.1 0.1 0.1 0.1 0.1 0.1]);
[K2,P2,E2]=lqr(A_E_2,B_E_2,Q,R);

%% the third trimming trajectory

% stack all constant velocities into compact vectors
nu_C_3=[linear_velocity_x_3(1);linear_velocity_y_3(1);linear_velocity_z_3(1)];
omega_C_3=[angular_velocity_roll_3(1);angular_velocity_pitch_3(1);angular_velocity_yaw_3(1)];

% error dynamics for position
A_p_v_3=eye(3);
A_p_w_3=zeros(3);
A_p_p_3=-vp(omega_C_3);
A_p_lambda_3=-vp(nu_C_3);

% error dynamics for orientation
A_lambda_v_3=zeros(3);
A_lambda_w_3=eye(3);
A_lambda_p_3=zeros(3);
A_lambda_lambda_3=-vp(omega_C_3);

% error dynamics for linear velocities
% parameters already defined in "LinearizationFormulasEvaluation"
% A_v_v_3
% A_v_w_3
% B_v_3
A_v_p_3=zeros(3);
A_v_lambda_3=zeros(3);


% error dynamics for angular velocities
% parameterd already defined in "LinearizationFormulasEvaluation"
% A_w_v_3
% A_w_w_3
% B_w_3
A_w_p_3=zeros(3);
A_w_lambda_3=zeros(3);

% formulate the error dynamics
A_E_3=[A_v_v_3 A_v_w_3 A_v_p_3 A_v_lambda_3;A_w_v_3 A_w_w_3 A_w_p_3 A_w_lambda_3;A_p_v_3 A_p_w_3 A_p_p_3 A_p_lambda_3;A_lambda_v_3 A_lambda_w_3 A_lambda_p_3 A_lambda_lambda_3];
B_E_3=[B_v_3;B_w_3;zeros(6)];

Q=diag([1000 1000 300 300 300 300 1 1 1 1 1 1]);
R=diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]);
[K3,P3,E3]=lqr(A_E_3,B_E_3,Q,R);

%% the fourth trimming trajectory

% stack all constant velocities into compact vectors
nu_C_4=[linear_velocity_x_4(1);linear_velocity_y_4(1);linear_velocity_z_4(1)];
omega_C_4=[angular_velocity_roll_4(1);angular_velocity_pitch_4(1);angular_velocity_yaw_4(1)];

% error dynamics for position
A_p_v_4=eye(3);
A_p_w_4=zeros(3);
A_p_p_4=-vp(omega_C_4);
A_p_lambda_4=-vp(nu_C_4);

% error dynamics for orientation
A_lambda_v_4=zeros(3);
A_lambda_w_4=eye(3);
A_lambda_p_4=zeros(3);
A_lambda_lambda_4=-vp(omega_C_4);

% error dynamics for linear velocities
% parameters already defined in "LinearizationFormulasEvaluation"
% A_v_v_4
% A_v_w_4
% B_v_4
A_v_p_4=zeros(3);
A_v_lambda_4=zeros(3);


% error dynamics for angular velocities
% parameterd already defined in "LinearizationFormulasEvaluation"
% A_w_v_4
% A_w_w_4
% B_w_4
A_w_p_4=zeros(3);
A_w_lambda_4=zeros(3);

% formulate the error dynamics
A_E_4=[A_v_v_4 A_v_w_4 A_v_p_4 A_v_lambda_4;A_w_v_4 A_w_w_4 A_w_p_4 A_w_lambda_4;A_p_v_4 A_p_w_4 A_p_p_4 A_p_lambda_4;A_lambda_v_4 A_lambda_w_4 A_lambda_p_4 A_lambda_lambda_4];
B_E_4=[B_v_4;B_w_4;zeros(6)];

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.001 0.001 0.001 0.001 0.001 0.001]);
[K4,P4,E4]=lqr(A_E_4,B_E_4,Q,R);

%% the fifth trimming trajectory

% stack all constant velocities into compact vectors
nu_C_5=[linear_velocity_x_5(1);linear_velocity_y_5(1);linear_velocity_z_5(1)];
omega_C_5=[angular_velocity_roll_5(1);angular_velocity_pitch_5(1);angular_velocity_yaw_5(1)];

% error dynamics for position
A_p_v_5=eye(3);
A_p_w_5=zeros(3);
A_p_p_5=-vp(omega_C_5);
A_p_lambda_5=-vp(nu_C_5);

% error dynamics for orientation
A_lambda_v_5=zeros(3);
A_lambda_w_5=eye(3);
A_lambda_p_5=zeros(3);
A_lambda_lambda_5=-vp(omega_C_5);

% error dynamics for linear velocities
% parameters already defined in "LinearizationFormulasEvaluation"
% A_v_v_5
% A_v_w_5
% B_v_5
A_v_p_5=zeros(3);
A_v_lambda_5=zeros(3);


% error dynamics for angular velocities
% parameterd already defined in "LinearizationFormulasEvaluation"
% A_w_v_5
% A_w_w_5
% B_w_5
A_w_p_5=zeros(3);
A_w_lambda_5=zeros(3);

% formulate the error dynamics
A_E_5=[A_v_v_5 A_v_w_5 A_v_p_5 A_v_lambda_5;A_w_v_5 A_w_w_5 A_w_p_5 A_w_lambda_5;A_p_v_5 A_p_w_5 A_p_p_5 A_p_lambda_5;A_lambda_v_5 A_lambda_w_5 A_lambda_p_5 A_lambda_lambda_5];
B_E_5=[B_v_5;B_w_5;zeros(6)];

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.001 0.001 0.001 0.001 0.001 0.001]);
[K5,P5,E5]=lqr(A_E_5,B_E_5,Q,R);

%% the sixth trimming trajectory

% stack all constant velocities into compact vectors
nu_C_6=[linear_velocity_x_6(1);linear_velocity_y_6(1);linear_velocity_z_6(1)];
omega_C_6=[angular_velocity_roll_6(1);angular_velocity_pitch_6(1);angular_velocity_yaw_6(1)];

% error dynamics for position
A_p_v_6=eye(3);
A_p_w_6=zeros(3);
A_p_p_6=-vp(omega_C_6);
A_p_lambda_6=-vp(nu_C_6);

% error dynamics for orientation
A_lambda_v_6=zeros(3);
A_lambda_w_6=eye(3);
A_lambda_p_6=zeros(3);
A_lambda_lambda_6=-vp(omega_C_6);

% error dynamics for linear velocities
% parameters already defined in "LinearizationFormulasEvaluation"
% A_v_v_6
% A_v_w_6
% B_v_6
A_v_p_6=zeros(3);
A_v_lambda_6=zeros(3);


% error dynamics for angular velocities
% parameterd already defined in "LinearizationFormulasEvaluation"
% A_w_v_6
% A_w_w_6
% B_w_6
A_w_p_6=zeros(3);
A_w_lambda_6=zeros(3);

% formulate the error dynamics
A_E_6=[A_v_v_6 A_v_w_6 A_v_p_6 A_v_lambda_6;A_w_v_6 A_w_w_6 A_w_p_6 A_w_lambda_6;A_p_v_6 A_p_w_6 A_p_p_6 A_p_lambda_6;A_lambda_v_6 A_lambda_w_6 A_lambda_p_6 A_lambda_lambda_6];
B_E_6=[B_v_6;B_w_6;zeros(6)];

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.001 0.001 0.001 0.001 0.001 0.001]);
[K6,P6,E6]=lqr(A_E_6,B_E_6,Q,R);

%% the seventh trimming trajectory

% stack all constant velocities into compact vectors
nu_C_7=[linear_velocity_x_7(1);linear_velocity_y_7(1);linear_velocity_z_7(1)];
omega_C_7=[angular_velocity_roll_7(1);angular_velocity_pitch_7(1);angular_velocity_yaw_7(1)];

% error dynamics for position
A_p_v_7=eye(3);
A_p_w_7=zeros(3);
A_p_p_7=-vp(omega_C_7);
A_p_lambda_7=-vp(nu_C_7);

% error dynamics for orientation
A_lambda_v_7=zeros(3);
A_lambda_w_7=eye(3);
A_lambda_p_7=zeros(3);
A_lambda_lambda_7=-vp(omega_C_7);

% error dynamics for linear velocities
% parameters already defined in "LinearizationFormulasEvaluation"
% A_v_v_7
% A_v_w_7
% B_v_7
A_v_p_7=zeros(3);
A_v_lambda_7=zeros(3);


% error dynamics for angular velocities
% parameterd already defined in "LinearizationFormulasEvaluation"
% A_w_v_7
% A_w_w_7
% B_w_7
A_w_p_7=zeros(3);
A_w_lambda_7=zeros(3);

% formulate the error dynamics
A_E_7=[A_v_v_7 A_v_w_7 A_v_p_7 A_v_lambda_7;A_w_v_7 A_w_w_7 A_w_p_7 A_w_lambda_7;A_p_v_7 A_p_w_7 A_p_p_7 A_p_lambda_7;A_lambda_v_7 A_lambda_w_7 A_lambda_p_7 A_lambda_lambda_7];
B_E_7=[B_v_7;B_w_7;zeros(6)];

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.001 0.001 0.001 0.001 0.001 0.001]);
[K7,P7,E7]=lqr(A_E_7,B_E_7,Q,R);
