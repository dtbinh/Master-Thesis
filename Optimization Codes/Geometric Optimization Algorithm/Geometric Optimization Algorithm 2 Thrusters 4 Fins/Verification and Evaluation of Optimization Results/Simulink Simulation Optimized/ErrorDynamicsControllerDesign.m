%% firstly let calculate the geometric center of all the fins in the body frame 

r_F1_optimized=[x_F1_optimized;0.5*(d_H+a_F1)*sin(gamma_F1_optimized);0.5*(d_H+a_F1)*cos(gamma_F1_optimized)];
r_F2_optimized=[x_F2_optimized;0.5*(d_H+a_F2)*sin(gamma_F2_optimized);0.5*(d_H+a_F2)*cos(gamma_F2_optimized)];
r_F3_optimized=[x_F3_optimized;0.5*(d_H+a_F3)*sin(gamma_F3_optimized);0.5*(d_H+a_F3)*cos(gamma_F3_optimized)];
r_F4_optimized=[x_F4_optimized;0.5*(d_H+a_F4)*sin(gamma_F4_optimized);0.5*(d_H+a_F4)*cos(gamma_F4_optimized)];

%% Calculate the optimal desired input for all trim trajectory segments

r_g_optimized=CenterofMass({m_H,r_H},{m_T1,r_T1_optimized},{m_T2,r_T2_optimized},{m_F1,r_F1_optimized},{m_F2,r_F2_optimized},{m_F3,r_F3_optimized},{m_F4,r_F4_optimized});

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



%% the first trimming trajectory

% the optimal configuration from the result 

T_S_1_optimized={r_T1_optimized,b_T1_optimized,d_T1_optimized};
T_S_2_optimized={r_T2_optimized,b_T2_optimized,d_T2_optimized}; 

% for different trim trajectory we have different parameterized fins

Fin1_optimized={C_L,[a_F1,b_F1],u_ind_1,[x_F1_optimized,gamma_F1_optimized]};

Fin2_optimized={C_L,[a_F2,b_F2],u_ind_1,[x_F2_optimized,gamma_F2_optimized]};

Fin3_optimized={C_L,[a_F3,b_F3],u_ind_1,[x_F3_optimized,gamma_F3_optimized]};

Fin4_optimized={C_L,[a_F4,b_F4],u_ind_1,[x_F4_optimized,gamma_F4_optimized]};


% use the optimal configuration to build the optimal dynamics

B_T=ThrusterConfigurationMatrix(T_S_1_optimized,T_S_2_optimized);

B_F=FinConfigurationMatrix(Fin1_optimized,Fin2_optimized,Fin3_optimized,Fin4_optimized);

% put all thruster configuration matrices with different spin directions 

B_input_1=horzcat(B_T,B_F);

% we update BH at first 

BH=inv(Mrb_optimized+veh.Ma)*B_input_1*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);




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

% let us evaluate the A_v_lambda

A_v_lambda_1=double(A_v_lambda_l(roll_angle_1(1),pitch_angle_1(1),yaw_angle_1(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));

% A_v_lambda_1=zeros(3);


% error dynamics for angular velocities

A_w_v_1=A_w_v_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

A_w_w_1=A_w_w_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

B_w=B_w_l; 

A_w_p_1=zeros(3);

% A_w_lambda_1=zeros(3);

A_w_lambda_1=double(A_w_lambda_l(roll_angle_1(1),pitch_angle_1(1),yaw_angle_1(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));



% formulate the error dynamics
A_E_1=[A_v_v_1 A_v_w_1 A_v_p_1 A_v_lambda_1;A_w_v_1 A_w_w_1 A_w_p_1 A_w_lambda_1;A_p_v_1 A_p_w_1 A_p_p_1 A_p_lambda_1;A_lambda_v_1 A_lambda_w_1 A_lambda_p_1 A_lambda_lambda_1];
B_E_1=[B_v_1;B_w_1;zeros(6)];

A_E_1=double(A_E_1);
B_E_1=double(B_E_1);

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.0001 0.0001 0.0001 0.0001 0.0001 0.0001]);
[K1,P1,E1]=lqr(A_E_1,B_E_1,Q,R);





%% the second trimming trajectory

% for different trim trajectory we have different parameterized fins

Fin1_optimized={C_L,[a_F1,b_F1],u_ind_3,[x_F1_optimized,gamma_F1_optimized]};

Fin2_optimized={C_L,[a_F2,b_F2],u_ind_3,[x_F2_optimized,gamma_F2_optimized]};

Fin3_optimized={C_L,[a_F3,b_F3],u_ind_3,[x_F3_optimized,gamma_F3_optimized]};

Fin4_optimized={C_L,[a_F4,b_F4],u_ind_3,[x_F4_optimized,gamma_F4_optimized]};


B_F=FinConfigurationMatrix(Fin1_optimized,Fin2_optimized,Fin3_optimized,Fin4_optimized);

% put all thruster configuration matrices with different spin directions 

B_input_2=horzcat(B_T,B_F);



BH=inv(Mrb_optimized+veh.Ma)*B_input*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

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

% A_v_lambda_2=zeros(3);

A_v_lambda_2=double(A_v_lambda_l(roll_angle_2(1),pitch_angle_2(1),yaw_angle_2(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));


% error dynamics for angular velocities

A_w_v_2=A_w_v_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

A_w_w_2=A_w_w_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

B_w_2=B_w_l;

A_w_p_2=zeros(3);

% A_w_lambda_2=zeros(3);

A_w_lambda_2=double(A_w_lambda_l(roll_angle_2(1),pitch_angle_2(1),yaw_angle_2(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));

% formulate the error dynamics
A_E_2=[A_v_v_2 A_v_w_2 A_v_p_2 A_v_lambda_2;A_w_v_2 A_w_w_2 A_w_p_2 A_w_lambda_2;A_p_v_2 A_p_w_2 A_p_p_2 A_p_lambda_2;A_lambda_v_2 A_lambda_w_2 A_lambda_p_2 A_lambda_lambda_2];
B_E_2=[B_v_2;B_w_2;zeros(6)];

A_E_2=double(A_E_2);
B_E_2=double(B_E_2);

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]);
[K2,P2,E2]=lqr(A_E_2,B_E_2,Q,R);

%% the third trimming trajectory

% for different trim trajectory we have different parameterized fins

Fin1_optimized={C_L,[a_F1,b_F1],u_ind_3,[x_F1_optimized,gamma_F1_optimized]};

Fin2_optimized={C_L,[a_F2,b_F2],u_ind_3,[x_F2_optimized,gamma_F2_optimized]};

Fin3_optimized={C_L,[a_F3,b_F3],u_ind_3,[x_F3_optimized,gamma_F3_optimized]};

Fin4_optimized={C_L,[a_F4,b_F4],u_ind_3,[x_F4_optimized,gamma_F4_optimized]};

B_F=FinConfigurationMatrix(Fin1_optimized,Fin2_optimized,Fin3_optimized,Fin4_optimized);

% put all thruster configuration matrices with different spin directions 

B_input_3=horzcat(B_T,B_F);



BH=inv(Mrb_optimized+veh.Ma)*B_input_3*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

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

A_v_v_3=A_v_v_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));

A_v_w_3=A_v_w_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));

B_v_3=B_v_l;

A_v_p_3=zeros(3);
% A_v_lambda_3=zeros(3);
A_v_lambda_3=double(A_v_lambda_l(roll_angle_3(1),pitch_angle_3(1),yaw_angle_3(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));


% error dynamics for angular velocities

A_w_v_3=A_w_v_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));

A_w_w_3=A_w_w_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));

B_w_3=B_w_l;

A_w_p_3=zeros(3);
% A_w_lambda_3=zeros(3);

A_w_lambda_3=double(A_w_lambda_l(roll_angle_3(1),pitch_angle_3(1),yaw_angle_3(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));


% formulate the error dynamics
A_E_3=[A_v_v_3 A_v_w_3 A_v_p_3 A_v_lambda_3;A_w_v_3 A_w_w_3 A_w_p_3 A_w_lambda_3;A_p_v_3 A_p_w_3 A_p_p_3 A_p_lambda_3;A_lambda_v_3 A_lambda_w_3 A_lambda_p_3 A_lambda_lambda_3];
B_E_3=[B_v_3;B_w_3;zeros(6)];

A_E_3=double(A_E_3);
B_E_3=double(B_E_3);

Q=diag([1000 1000 1000 1 1 1 1 1 1 1 1 1]);
R=diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]);
[K3,P3,E3]=lqr(A_E_3,B_E_3,Q,R);

%% the fourth trimming trajectory

Fin1_optimized={C_L,[a_F1,b_F1],u_ind_4,[x_F1_optimized,gamma_F1_optimized]};

Fin2_optimized={C_L,[a_F2,b_F2],u_ind_4,[x_F2_optimized,gamma_F2_optimized]};

Fin3_optimized={C_L,[a_F3,b_F3],u_ind_4,[x_F3_optimized,gamma_F3_optimized]};

Fin4_optimized={C_L,[a_F4,b_F4],u_ind_4,[x_F4_optimized,gamma_F4_optimized]};

B_F=FinConfigurationMatrix(Fin1_optimized,Fin2_optimized,Fin3_optimized,Fin4_optimized);

% put all thruster configuration matrices with different spin directions 

B_input_4=horzcat(B_T,B_F);



BH=inv(Mrb_optimized+veh.Ma)*B_input_4*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

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

A_v_v_4=A_v_v_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));

A_v_w_4=A_v_w_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));

B_v_4=B_v_l;

A_v_p_4=zeros(3);
% A_v_lambda_4=zeros(3);
A_v_lambda_4=double(A_v_lambda_l(roll_angle_4(1),pitch_angle_4(1),yaw_angle_4(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));




% error dynamics for angular velocities

A_w_v_4=A_w_v_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));

A_w_w_4=A_w_w_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));

B_w_4=B_w_l;

A_w_p_4=zeros(3);
% A_w_lambda_4=zeros(3);

A_w_lambda_4=double(A_w_lambda_l(roll_angle_4(1),pitch_angle_4(1),yaw_angle_4(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));


% formulate the error dynamics
A_E_4=[A_v_v_4 A_v_w_4 A_v_p_4 A_v_lambda_4;A_w_v_4 A_w_w_4 A_w_p_4 A_w_lambda_4;A_p_v_4 A_p_w_4 A_p_p_4 A_p_lambda_4;A_lambda_v_4 A_lambda_w_4 A_lambda_p_4 A_lambda_lambda_4];
B_E_4=[B_v_4;B_w_4;zeros(6)];

A_E_4=double(A_E_4);
B_E_4=double(B_E_4);

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]);
[K4,P4,E4]=lqr(A_E_4,B_E_4,Q,R);

%% the fifth trimming trajectory

% for different trim trajectory we have different parameterized fins

Fin1_optimized={C_L,[a_F1,b_F1],u_ind_5,[x_F1_optimized,gamma_F1_optimized]};

Fin2_optimized={C_L,[a_F2,b_F2],u_ind_5,[x_F2_optimized,gamma_F2_optimized]};

Fin3_optimized={C_L,[a_F3,b_F3],u_ind_5,[x_F3_optimized,gamma_F3_optimized]};

Fin4_optimized={C_L,[a_F4,b_F4],u_ind_5,[x_F4_optimized,gamma_F4_optimized]};

B_F=FinConfigurationMatrix(Fin1_optimized,Fin2_optimized,Fin3_optimized,Fin4_optimized);

% put all thruster configuration matrices with different spin directions 

B_input_5=horzcat(B_T,B_F);



BH=inv(Mrb_optimized+veh.Ma)*B_input_5*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);


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

A_v_v_5=A_v_v_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));

A_v_w_5=A_v_w_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));

B_v_5=B_v_l;

A_v_p_5=zeros(3);
% A_v_lambda_5=zeros(3);

A_v_lambda_5=double(A_v_lambda_l(roll_angle_5(1),pitch_angle_5(1),yaw_angle_5(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));



% error dynamics for angular velocities

A_w_v_5=A_w_v_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));

A_w_w_5=A_w_w_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));

B_w_5=B_w_l;

A_w_p_5=zeros(3);
% A_w_lambda_5=zeros(3);

A_w_lambda_5=double(A_w_lambda_l(roll_angle_5(1),pitch_angle_5(1),yaw_angle_5(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));


% formulate the error dynamics
A_E_5=[A_v_v_5 A_v_w_5 A_v_p_5 A_v_lambda_5;A_w_v_5 A_w_w_5 A_w_p_5 A_w_lambda_5;A_p_v_5 A_p_w_5 A_p_p_5 A_p_lambda_5;A_lambda_v_5 A_lambda_w_5 A_lambda_p_5 A_lambda_lambda_5];
B_E_5=[B_v_5;B_w_5;zeros(6)];

A_E_5=double(A_E_5);
B_E_5=double(B_E_5);


Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]);
[K5,P5,E5]=lqr(A_E_5,B_E_5,Q,R);

%% the sixth trimming trajectory

% for different trim trajectory we have different parameterized fins

Fin1_optimized={C_L,[a_F1,b_F1],u_ind_6,[x_F1_optimized,gamma_F1_optimized]};

Fin2_optimized={C_L,[a_F2,b_F2],u_ind_6,[x_F2_optimized,gamma_F2_optimized]};

Fin3_optimized={C_L,[a_F3,b_F3],u_ind_6,[x_F3_optimized,gamma_F3_optimized]};

Fin4_optimized={C_L,[a_F4,b_F4],u_ind_6,[x_F4_optimized,gamma_F4_optimized]};

B_F=FinConfigurationMatrix(Fin1_optimized,Fin2_optimized,Fin3_optimized,Fin4_optimized);

% put all thruster configuration matrices with different spin directions 

B_input_6=horzcat(B_T,B_F);



BH=inv(Mrb_optimized+veh.Ma)*B_input_6*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

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

A_v_v_6=A_v_v_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));

A_v_w_6=A_v_w_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));

B_v_6=B_v_l;

A_v_p_6=zeros(3);
% A_v_lambda_6=zeros(3);


A_v_lambda_6=double(A_v_lambda_l(roll_angle_6(1),pitch_angle_6(1),yaw_angle_6(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));

% error dynamics for angular velocities

A_w_v_6=A_w_v_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));

A_w_w_6=A_w_w_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));

B_w_6=B_w_l;

A_w_p_6=zeros(3);
% A_w_lambda_6=zeros(3);
A_w_lambda_6=double(A_w_lambda_l(roll_angle_6(1),pitch_angle_6(1),yaw_angle_6(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));


% formulate the error dynamics
A_E_6=[A_v_v_6 A_v_w_6 A_v_p_6 A_v_lambda_6;A_w_v_6 A_w_w_6 A_w_p_6 A_w_lambda_6;A_p_v_6 A_p_w_6 A_p_p_6 A_p_lambda_6;A_lambda_v_6 A_lambda_w_6 A_lambda_p_6 A_lambda_lambda_6];
B_E_6=[B_v_6;B_w_6;zeros(6)];

A_E_6=double(A_E_6);
B_E_6=double(B_E_6);

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]);
[K6,P6,E6]=lqr(A_E_6,B_E_6,Q,R);

%% the seventh trimming trajectory

% for different trim trajectory we have different parameterized fins

Fin1_optimized={C_L,[a_F1,b_F1],u_ind_7,[x_F1_optimized,gamma_F1_optimized]};

Fin2_optimized={C_L,[a_F2,b_F2],u_ind_7,[x_F2_optimized,gamma_F2_optimized]};

Fin3_optimized={C_L,[a_F3,b_F3],u_ind_7,[x_F3_optimized,gamma_F3_optimized]};

Fin4_optimized={C_L,[a_F4,b_F4],u_ind_7,[x_F4_optimized,gamma_F4_optimized]};

B_F=FinConfigurationMatrix(Fin1_optimized,Fin2_optimized,Fin3_optimized,Fin4_optimized);

% put all thruster configuration matrices with different spin directions 

B_input_7=horzcat(B_T,B_F);



BH=inv(Mrb_optimized+veh.Ma)*B_input_7*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

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

A_v_v_7=A_v_v_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));

A_v_w_7=A_v_w_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));

B_v_7=B_v_l;

A_v_p_7=zeros(3);
% A_v_lambda_7=zeros(3);

A_v_lambda_7=double(A_v_lambda_l(roll_angle_7(1),pitch_angle_7(1),yaw_angle_7(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));



% error dynamics for angular velocities

A_w_v_7=A_w_v_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));

A_w_w_7=A_w_w_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));

B_w_7=B_w_l;

A_w_p_7=zeros(3);
% A_w_lambda_7=zeros(3);

A_w_lambda_7=double(A_w_lambda_l(roll_angle_7(1),pitch_angle_7(1),yaw_angle_7(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),r_g_optimized(1),r_g_optimized(2),r_g_optimized(3)));


% formulate the error dynamics
A_E_7=[A_v_v_7 A_v_w_7 A_v_p_7 A_v_lambda_7;A_w_v_7 A_w_w_7 A_w_p_7 A_w_lambda_7;A_p_v_7 A_p_w_7 A_p_p_7 A_p_lambda_7;A_lambda_v_7 A_lambda_w_7 A_lambda_p_7 A_lambda_lambda_7];
B_E_7=[B_v_7;B_w_7;zeros(6)];

A_E_7=double(A_E_7);
B_E_7=double(B_E_7);

Q=diag([1000 1000 1000 300 300 300 1 1 1 1 1 1]);
R=diag([0.0001 0.0001 0.0001 0.0001 0.0001 0.0001]);
[K7,P7,E7]=lqr(A_E_7,B_E_7,Q,R);

