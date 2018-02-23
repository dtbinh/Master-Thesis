%% note that the dynamics for each trim trajectory we should update the trim trajectory

% 
%%-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Preprocessing 

% at the first step, we only determine the spin direction of all thrusters
% we have 2^n_thruster configurations

%% for all trim trajectories 

% we want to find the best spin direction group,i.e, the spin direction
% group with the smallest condition number 
% but we should iterate within all trim trajectory segments 

bestConditionNumber=inf;


% First loop of optimization to determine the spin directions 

% In the preprocessing phase, the fin configuration matrix does not change 

% because we only change the value of the spin direction, at least in this
% pre-optimization phase, thus in this phase, we should intialize the fin
% configuration firstly for each trim trajectory segment


% spin direction index is used to the amount of different spin
% configurations 
spin_i=1;

for b_T1=-1:2:1
  for b_T2=-1:2:1


 
T_S_1={veh.r_T1,b_T1,veh.d_T1};
T_S_2={veh.r_T2,b_T2,veh.d_T2}; 

                   
% stack spin directions of all thrusters into one vector 

b_T=[b_T1;b_T2]; 

% put all different spin direction configuration into

% a cell group

b_T_c{spin_i}=b_T;


%% since we have fins and the fin configuratio matrix is 
% We change the Dynamics when we update the spin direction of all thrusters

%%  put the current spin direction of all thrusters into the mapping matrix
% for each thruster 

% after the configuration is fixed
% we should compute the current error dynmaics for each trim trajectory
% segments 

% u_l, v_l, w_l, p_l, q_l, r_l are already specified in the file 
% they are calculated in the initialization file
% "ThrusterConfigurationOptiInit.m"

% for the linearized error dynamics 

% The dynamic part is linearized mathematically using jacobian 

%% evaluate the system matrix 

% first trim trajectory

% update the configuration parameter for both thrusters and fins

% stack the thruster parameters in to a cell


% 1. Trim Trajectory

% build fin configuration matrix 

% Fin1_Trim1={C_L,[a_F1,b_F1],u_ind_1,[x_F1,gamma_F1]};
% 
% Fin2_Trim1={C_L,[a_F2,b_F2],u_ind_1,[x_F2,gamma_F2]};
% 
% Fin3_Trim1={C_L,[a_F3,b_F3],u_ind_1,[x_F3,gamma_F3]};
% 
% Fin4_Trim1={C_L,[a_F4,b_F4],u_ind_1,[x_F4,gamma_F4]};


Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

DynamicsUpdate;

A_v_v_1=A_v_v_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

A_v_w_1=A_v_w_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));


A_w_v_1=A_w_v_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));

A_w_w_1=A_w_w_l(v_d_1(1),v_d_1(2),v_d_1(3),v_d_1(4),v_d_1(5),v_d_1(6));


% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_1=[v_d_1(1),v_d_1(2),v_d_1(3)];

omega_C_1=[v_d_1(4),v_d_1(5),v_d_1(6)];

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

A_v_p_1=zeros(3);

% A_v_lambda_1=zeros(3);

A_v_lambda_1=double(A_v_lambda_l(roll_angle_1(1),pitch_angle_1(1),yaw_angle_1(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));



% error dynamics for angular velocities

A_w_p_1=zeros(3);

% A_w_lambda_1=zeros(3);

A_w_lambda_1=double(A_w_lambda_l(roll_angle_1(1),pitch_angle_1(1),yaw_angle_1(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% The input matrix should will not be changed 

B_v_1=double(B_v_l);

B_w_1=double(B_w_l);


% formulate the error dynamics

A_E_1=[A_v_v_1 A_v_w_1 A_v_p_1 A_v_lambda_1;A_w_v_1 A_w_w_1 A_w_p_1 A_w_lambda_1;A_p_v_1 A_p_w_1 A_p_p_1 A_p_lambda_1;A_lambda_v_1 A_lambda_w_1 A_lambda_p_1 A_lambda_lambda_1];

B_E_1=[B_v_1;B_w_1;zeros(6)];


% A_E_c: System Matrix Clusters for the first trim tra

% B_E_c: Error Input Matrix Clusters for the first trim trajectory:  

B_E_c_1{spin_i}=B_E_1;

% build the controllability matrix

Co_1=ctrb(A_E_1,B_E_1);

% Co_c: Controllability matrix cluster

Co_c_1{spin_i}=Co_1;


%% second trim trajectory


% 2. Trim Trajectory

% build fin configuration matrix 
% 
% Fin1_Trim2={C_L,[a_F1,b_F1],u_ind_2,[x_F1,gamma_F1]};
% 
% Fin2_Trim2={C_L,[a_F2,b_F2],u_ind_2,[x_F2,gamma_F2]};
% 
% Fin3_Trim2={C_L,[a_F3,b_F3],u_ind_2,[x_F3,gamma_F3]};
% 
% Fin4_Trim2={C_L,[a_F4,b_F4],u_ind_2,[x_F4,gamma_F4]};

Fin1={C_L,[a_F1,b_F1],u_ind_2,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_2,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_2,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_2,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

DynamicsUpdate;

A_v_v_2=A_v_v_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

A_v_w_2=A_v_w_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));



A_w_v_2=A_w_v_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));

A_w_w_2=A_w_w_l(v_d_2(1),v_d_2(2),v_d_2(3),v_d_2(4),v_d_2(5),v_d_2(6));



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_2=[v_d_2(1),v_d_2(2),v_d_2(3)];

omega_C_2=[v_d_2(4),v_d_2(5),v_d_2(6)];

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

A_v_p_2=zeros(3);

% A_v_lambda_2=zeros(3);

A_v_lambda_2=double(A_v_lambda_l(roll_angle_2(1),pitch_angle_2(1),yaw_angle_2(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));



% error dynamics for angular velocities

A_w_p_2=zeros(3);

% A_w_lambda_2=zeros(3);

A_w_lambda_2=double(A_w_lambda_l(roll_angle_2(1),pitch_angle_2(1),yaw_angle_2(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% The input matrix should will not be changed 

B_v_2=double(B_v_l);

B_w_2=double(B_w_l);

% formulate the error dynamics

A_E_2=[A_v_v_2 A_v_w_2 A_v_p_2 A_v_lambda_2;A_w_v_2 A_w_w_2 A_w_p_2 A_w_lambda_2;A_p_v_2 A_p_w_2 A_p_p_2 A_p_lambda_2;A_lambda_v_2 A_lambda_w_2 A_lambda_p_2 A_lambda_lambda_2];

B_E_2=[B_v_2;B_w_2;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_2{spin_i}=B_E_2;

% build the controllability matrix

Co_2=ctrb(A_E_2,B_E_2);

% Co_c: Controllability matrix cluster

Co_c_2{spin_i}=Co_2;


%% third trim trajectory

% 3. Trim Trajectory

% build fin configuration matrix 

% Fin1_Trim3={C_L,[a_F1,b_F1],u_ind_3,[x_F1,gamma_F1]};
% 
% Fin2_Trim3={C_L,[a_F2,b_F2],u_ind_3,[x_F2,gamma_F2]};
% 
% Fin3_Trim3={C_L,[a_F3,b_F3],u_ind_3,[x_F3,gamma_F3]};
% 
% Fin4_Trim3={C_L,[a_F4,b_F4],u_ind_3,[x_F4,gamma_F4]};


Fin1={C_L,[a_F1,b_F1],u_ind_3,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_3,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_3,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_3,[veh.x_F4,veh.gamma_F4]};



ConfigurationUpdate;

DynamicsUpdate;


A_v_v_3=A_v_v_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));


A_v_w_3=A_v_w_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));



A_w_v_3=A_w_v_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));


A_w_w_3=A_w_w_l(v_d_3(1),v_d_3(2),v_d_3(3),v_d_3(4),v_d_3(5),v_d_3(6));



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_3=[v_d_3(1),v_d_3(2),v_d_3(3)];

omega_C_3=[v_d_3(4),v_d_3(5),v_d_3(6)];

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

A_v_p_3=zeros(3);

% A_v_lambda_3=zeros(3);

A_v_lambda_3=double(A_v_lambda_l(roll_angle_3(1),pitch_angle_3(1),yaw_angle_3(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% error dynamics for angular velocities

A_w_p_3=zeros(3);

% A_w_lambda_3=zeros(3);

A_w_lambda_3=double(A_w_lambda_l(roll_angle_3(1),pitch_angle_3(1),yaw_angle_3(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));



% The input matrix

B_v_3=double(B_v_l);

B_w_3=double(B_w_l);

% formulate the error dynamics

A_E_3=[A_v_v_3 A_v_w_3 A_v_p_3 A_v_lambda_3;A_w_v_3 A_w_w_3 A_w_p_3 A_w_lambda_3;A_p_v_3 A_p_w_3 A_p_p_3 A_p_lambda_3;A_lambda_v_3 A_lambda_w_3 A_lambda_p_3 A_lambda_lambda_3];

B_E_3=[B_v_3;B_w_3;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_3{spin_i}=B_E_3;

% build the controllability matrix

Co_3=ctrb(A_E_3,B_E_3);

% Co_c: Controllability matrix cluster

Co_c_3{spin_i}=Co_3;


%% fourth trim trajectory

% 4. Trim Trajectory

% build fin configuration matrix 

% Fin1_Trim4={C_L,[a_F1,b_F1],u_ind_4,[x_F1,gamma_F1]};
% 
% Fin2_Trim4={C_L,[a_F2,b_F2],u_ind_4,[x_F2,gamma_F2]};
% 
% Fin3_Trim4={C_L,[a_F3,b_F3],u_ind_4,[x_F3,gamma_F3]};
% 
% Fin4_Trim4={C_L,[a_F4,b_F4],u_ind_4,[x_F4,gamma_F4]};


Fin1={C_L,[a_F1,b_F1],u_ind_4,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_4,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_4,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_4,[veh.x_F4,veh.gamma_F4]};

ConfigurationUpdate;

DynamicsUpdate;


A_v_v_4=A_v_v_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));

A_v_w_4=A_v_w_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));



A_w_v_4=A_w_v_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));

A_w_w_4=A_w_w_l(v_d_4(1),v_d_4(2),v_d_4(3),v_d_4(4),v_d_4(5),v_d_4(6));



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_4=[v_d_4(1),v_d_4(2),v_d_4(3)];

omega_C_4=[v_d_4(4),v_d_4(5),v_d_4(6)];


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

A_v_p_4=zeros(3);

% A_v_lambda_4=zeros(3);

A_v_lambda_4=double(A_v_lambda_l(roll_angle_4(1),pitch_angle_4(1),yaw_angle_4(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));



% error dynamics for angular velocities

A_w_p_4=zeros(3);

% A_w_lambda_4=zeros(3);

A_w_lambda_4=double(A_w_lambda_l(roll_angle_4(1),pitch_angle_4(1),yaw_angle_4(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% The input matrix

B_v_4=double(B_v_l);

B_w_4=double(B_w_l);


% formulate the error dynamics

A_E_4=[A_v_v_4 A_v_w_4 A_v_p_4 A_v_lambda_4;A_w_v_4 A_w_w_4 A_w_p_4 A_w_lambda_4;A_p_v_4 A_p_w_4 A_p_p_4 A_p_lambda_4;A_lambda_v_4 A_lambda_w_4 A_lambda_p_4 A_lambda_lambda_4];

B_E_4=[B_v_4;B_w_4;zeros(6)];


% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_4{spin_i}=B_E_4;

% build the controllability matrix

Co_4=ctrb(A_E_4,B_E_4);

% Co_c: Controllability matrix cluster

Co_c_4{spin_i}=Co_4;


%%  fifth trim trajectory

% 
% Fin1_Trim5={C_L,[a_F1,b_F1],u_ind_3,[x_F1,gamma_F1]};
% 
% Fin2_Trim5={C_L,[a_F2,b_F2],u_ind_3,[x_F2,gamma_F2]};
% 
% Fin3_Trim5={C_L,[a_F3,b_F3],u_ind_3,[x_F3,gamma_F3]};
% 
% Fin4_Trim5={C_L,[a_F4,b_F4],u_ind_3,[x_F4,gamma_F4]};

Fin1={C_L,[a_F1,b_F1],u_ind_3,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_3,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_3,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_3,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

DynamicsUpdate;

A_v_v_5=A_v_v_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));

A_v_w_5=A_v_w_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));


A_w_v_5=A_w_v_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));

A_w_w_5=A_w_w_l(v_d_5(1),v_d_5(2),v_d_5(3),v_d_5(4),v_d_5(5),v_d_5(6));


% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_5=[v_d_5(1),v_d_5(2),v_d_5(3)];

omega_C_5=[v_d_5(4),v_d_5(5),v_d_5(6)];

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

A_v_p_5=zeros(3);

% A_v_lambda_5=zeros(3);

A_v_lambda_5=double(A_v_lambda_l(roll_angle_5(1),pitch_angle_5(1),yaw_angle_5(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));



% error dynamics for angular velocities

A_w_p_5=zeros(3);

% A_w_lambda_5=zeros(3);

A_w_lambda_5=double(A_w_lambda_l(roll_angle_5(1),pitch_angle_5(1),yaw_angle_5(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% formulate the error dynamics

A_E_5=[A_v_v_5 A_v_w_5 A_v_p_5 A_v_lambda_5;A_w_v_5 A_w_w_5 A_w_p_5 A_w_lambda_5;A_p_v_5 A_p_w_5 A_p_p_5 A_p_lambda_5;A_lambda_v_5 A_lambda_w_5 A_lambda_p_5 A_lambda_lambda_5];

% The input matrix

B_v_5=double(B_v_l);

B_w_5=double(B_w_l);

B_E_5=[B_v_5;B_w_5;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_5{spin_i}=B_E_5;

% build the controllability matrix

Co_5=ctrb(A_E_5,B_E_5);

% Co_c: Controllability matrix cluster

Co_c_5{spin_i}=Co_5;

%% sixth trim trajectory


% Fin1_Trim6={C_L,[a_F1,b_F1],u_ind_3,[x_F1,gamma_F1]};
% 
% Fin2_Trim6={C_L,[a_F2,b_F2],u_ind_3,[x_F2,gamma_F2]};
% 
% Fin3_Trim6={C_L,[a_F3,b_F3],u_ind_3,[x_F3,gamma_F3]};
% 
% Fin4_Trim6={C_L,[a_F4,b_F4],u_ind_3,[x_F4,gamma_F4]};

Fin1={C_L,[a_F1,b_F1],u_ind_3,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_3,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_3,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_3,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

DynamicsUpdate;

A_v_v_6=A_v_v_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));

A_v_w_6=A_v_w_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));



A_w_v_6=A_w_v_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));

A_w_w_6=A_w_w_l(v_d_6(1),v_d_6(2),v_d_6(3),v_d_6(4),v_d_6(5),v_d_6(6));



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_6=[v_d_6(1),v_d_6(2),v_d_6(3)];

omega_C_6=[v_d_6(4),v_d_6(5),v_d_6(6)];

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

A_v_p_6=zeros(3);

% A_v_lambda_6=zeros(3); 

A_v_lambda_6=double(A_v_lambda_l(roll_angle_6(1),pitch_angle_6(1),yaw_angle_6(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));



% error dynamics for angular velocities

A_w_p_6=zeros(3);

% A_w_lambda_6=zeros(3);
A_w_lambda_6=double(A_w_lambda_l(roll_angle_6(1),pitch_angle_6(1),yaw_angle_6(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% formulate the error dynamics
A_E_6=[A_v_v_6 A_v_w_6 A_v_p_6 A_v_lambda_6;A_w_v_6 A_w_w_6 A_w_p_6 A_w_lambda_6;A_p_v_6 A_p_w_6 A_p_p_6 A_p_lambda_6;A_lambda_v_6 A_lambda_w_6 A_lambda_p_6 A_lambda_lambda_6];

B_v_6=double(B_v_l);

B_w_6=double(B_w_l);

B_E_6=[B_v_6;B_w_6;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_6{spin_i}=B_E_6;

% build the controllability matrix

Co_6=ctrb(A_E_6,B_E_6);

% Co_c: Controllability matrix cluster

Co_c_6{spin_i}=Co_6;

%% seventh trim trajectory

% Fin1_Trim7={C_L,[a_F1,b_F1],u_ind_3,[x_F1,gamma_F1]};
% 
% Fin2_Trim7={C_L,[a_F2,b_F2],u_ind_3,[x_F2,gamma_F2]};
% 
% Fin3_Trim7={C_L,[a_F3,b_F3],u_ind_3,[x_F3,gamma_F3]};
% 
% Fin4_Trim7={C_L,[a_F4,b_F4],u_ind_3,[x_F4,gamma_F4]};

Fin1={C_L,[a_F1,b_F1],u_ind_3,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_3,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_3,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_3,[veh.x_F4,veh.gamma_F4]};



ConfigurationUpdate;

DynamicsUpdate;


A_v_v_7=A_v_v_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));

A_v_w_7=A_v_w_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));



A_w_v_7=A_w_v_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));

A_w_w_7=A_w_w_l(v_d_7(1),v_d_7(2),v_d_7(3),v_d_7(4),v_d_7(5),v_d_7(6));
% 
% 
% 
% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_7=[v_d_7(1),v_d_7(2),v_d_7(3)];

omega_C_7=[v_d_7(4),v_d_7(5),v_d_7(6)];

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

A_v_p_7=zeros(3);

% A_v_lambda_7=zeros(3);

A_v_lambda_7=double(A_v_lambda_l(roll_angle_7(1),pitch_angle_7(1),yaw_angle_7(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));



% error dynamics for angular velocities

A_w_p_7=zeros(3);

% A_w_lambda_7=zeros(3);

A_w_lambda_7=double(A_w_lambda_l(roll_angle_7(1),pitch_angle_7(1),yaw_angle_7(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% formulate the error dynamics

A_E_7=[A_v_v_7 A_v_w_7 A_v_p_7 A_v_lambda_7;A_w_v_7 A_w_w_7 A_w_p_7 A_w_lambda_7;A_p_v_7 A_p_w_7 A_p_p_7 A_p_lambda_7;A_lambda_v_7 A_lambda_w_7 A_lambda_p_7 A_lambda_lambda_7];

B_v_7=double(B_v_l);

B_w_7=double(B_w_l);

B_E_7=[B_v_7;B_w_7;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_7{spin_i}=B_E_7;

% build the controllability matrix

Co_7=ctrb(A_E_7,B_E_7);

% Co_c: Controllability matrix cluster

Co_c_7{spin_i}=Co_7;

% index for different spin directions: 
spin_i=spin_i+1;
    end
end

%% after the iteration of all trim trajectory segments,

% chose the direction with the smallest condition number for each trim
% trajectory segment 

% since here we have 7 trim trajectories


% for each trim trajectories we have a cluster of condition number Co_c_i:
% in which the controllability matrix should be stored 

% we should do a transformation at first

% for each spin direction, it has seven elements

% put them into different groups

% Co_b_c_1

% Co_b_c_2

% Co_b_c_3

% Co_b_c_4


% % for the first spin direction
% 
% Co_b_c_1{1}=Co_c_1{1};
% Co_b_c_1{2}=Co_c_2{1};
% Co_b_c_1{3}=Co_c_3{1};
% Co_b_c_1{4}=Co_c_4{1};
% Co_b_c_1{5}=Co_c_5{1};
% Co_b_c_1{6}=Co_c_6{1};
% Co_b_c_1{7}=Co_c_7{1};
% 
% % for the second spin direction
% 
% Co_b_c_2{1}=Co_c_1{2};
% Co_b_c_2{2}=Co_c_2{2};
% Co_b_c_2{3}=Co_c_3{2};
% Co_b_c_2{4}=Co_c_4{2};
% Co_b_c_2{5}=Co_c_5{2};
% Co_b_c_2{6}=Co_c_6{2};
% Co_b_c_2{7}=Co_c_7{2};
% 
% % for the third spin direction 
% 
% Co_b_c_3{1}=Co_c_1{3};
% Co_b_c_3{2}=Co_c_2{3};
% Co_b_c_3{3}=Co_c_3{3};
% Co_b_c_3{4}=Co_c_4{3};
% Co_b_c_3{5}=Co_c_5{3};
% Co_b_c_3{6}=Co_c_6{3};
% Co_b_c_3{7}=Co_c_7{3}; 
% 
% % for the fourth spin direction 
% 
% Co_b_c_4{1}=Co_c_1{4};
% Co_b_c_4{2}=Co_c_2{4};
% Co_b_c_4{3}=Co_c_3{4};
% Co_b_c_4{4}=Co_c_4{4};
% Co_b_c_4{5}=Co_c_5{4};
% Co_b_c_4{6}=Co_c_6{4};
% Co_b_c_4{7}=Co_c_7{4}; 
% 
% % for the fifth spin direction 
% 
% Co_b_c_5{1}=Co_c_1{5};
% Co_b_c_5{2}=Co_c_2{5};
% Co_b_c_5{3}=Co_c_3{5};
% Co_b_c_5{4}=Co_c_4{5};
% Co_b_c_5{5}=Co_c_5{5};
% Co_b_c_5{6}=Co_c_6{5};
% Co_b_c_5{7}=Co_c_7{5}; 
% 
% % for the sixth spin direction 
% 
% Co_b_c_6{1}=Co_c_1{6};
% Co_b_c_6{2}=Co_c_2{6};
% Co_b_c_6{3}=Co_c_3{6};
% Co_b_c_6{4}=Co_c_4{6};
% Co_b_c_6{5}=Co_c_5{6};
% Co_b_c_6{6}=Co_c_6{6};
% Co_b_c_6{7}=Co_c_7{6}; 
% 
% % for the seventh spin direction 
% 
% Co_b_c_7{1}=Co_c_1{7};
% Co_b_c_7{2}=Co_c_2{7};
% Co_b_c_7{3}=Co_c_3{7};
% Co_b_c_7{4}=Co_c_4{7};
% Co_b_c_7{5}=Co_c_5{7};
% Co_b_c_7{6}=Co_c_6{7};
% Co_b_c_7{7}=Co_c_7{7}; 

%% using the following methods to chose the best configuration
cond_c=zeros(4,1);
num_cond_c=zeros(4,1);

for i_spin=1:4
    if rank(Co_c_1{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_1{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
    if rank(Co_c_2{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_2{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
    if rank(Co_c_3{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_3{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
    if rank(Co_c_4{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_4{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
    if rank(Co_c_5{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_5{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
    if rank(Co_c_6{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_6{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
    if rank(Co_c_7{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_7{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
end


cond_c_av=zeros(4,1);

for j_spin=1:4
    cond_c_av(j_spin)=cond_c(j_spin)/num_cond_c(j_spin);
end

[minval,min_index]=min(cond_c_av);

BestSpinDirection=b_T_c{min_index};
min_d=[];

for i=1:4
    if cond_c_av(i)==minval
        min_d=[min_d;cond_c_av(i)];
    end
end

%% assign the optimal spin direction to the global variable veh

veh.b_T1=BestSpinDirection(1);
veh.b_T2=BestSpinDirection(2);


% Define the following variable for simualtion

b_T1_optimized=BestSpinDirection(1);
b_T2_optimized=BestSpinDirection(2);



% now for each spin direciton configuraition

% for the cluster of each configuration we should first check the
% controllability 

% then we calculate the average of all controllable controllability matrix


% The first spin configuration 
% 
% cond_b_1=0; 
% 
% num_cond_b_1=0;
% 
% n_trimtraj=7;
% 
% % sum of all conditon 
% 
% for i=1:n_trimtraj
%     if rank(Co_b_c_1{i})==12
%             cond_b_1=cond(Co_b_c_1{i})+cond_b_1; 
%             num_cond_b_1=num_cond_b_1+1;
%              
%     end
% end
% 
% cond_b_1_av=cond_b_1/num_cond_b_1; 
% 
% 
% % The second spin configuration 
% 
% cond_b_2=0; 
% 
% num_cond_b_2=0;
% 
% for i=1:n_trimtraj
%     if rank(Co_b_c_2{i})==12
%             cond_b_2=cond(Co_b_c_2{i})+cond_b_2; 
%             num_cond_b_2=num_cond_b_2+1;
%              
%     end
% end
% 
% cond_b_2_av=cond_b_2/num_cond_b_2; 
% 
% 
% % The third spin configuration
% 
% cond_b_3=0; 
% 
% num_cond_b_3=0;
% 
% for i=1:n_trimtraj
%     if rank(Co_b_c_3{i})==12
%             cond_b_3=cond(Co_b_c_3{i})+cond_b_3; 
%             num_cond_b_3=num_cond_b_3+1;
%              
%     end
% end
% 
% cond_b_3_av=cond_b_3/num_cond_b_3; 
% 
% 
% % The fourth spin configuration
% 
% cond_b_4=0; 
% 
% num_cond_b_4=0;
% 
% for i=1:n_trimtraj
%     if rank(Co_b_c_4{i})==12
%             cond_b_4=cond(Co_b_c_4{i})+cond_b_4; 
%             num_cond_b_4=num_cond_b_4+1;
%              
%     end
% end
% 
% cond_b_4_av=cond_b_4/num_cond_b_4; 
% 
% % The fourth spin configuration
% 
% cond_b_5=0; 
% 
% num_cond_b_5=0;
% 
% for i=1:n_trimtraj
%     if rank(Co_b_c_5{i})==12
%             cond_b_5=cond(Co_b_c_5{i})+cond_b_5; 
%             num_cond_b_5=num_cond_b_5+1;
%              
%     end
% end
% 
% cond_b_5_av=cond_b_5/num_cond_b_5; 
% 
% % The sixth spin configuration
% 
% cond_b_6=0; 
% 
% num_cond_b_6=0;
% 
% for i=1:n_trimtraj
%     if rank(Co_b_c_6{i})==12
%             cond_b_6=cond(Co_b_c_6{i})+cond_b_6; 
%             num_cond_b_6=num_cond_b_6+1;
%              
%     end
% end
% 
% cond_b_6_av=cond_b_6/num_cond_b_6; 
% 
% % The sixth spin configuration
% 
% cond_b_7=0; 
% 
% num_cond_b_7=0;
% 
% for i=1:n_trimtraj
%     if rank(Co_b_c_7{i})==12
%             cond_b_7=cond(Co_b_c_7{i})+cond_b_7; 
%             num_cond_b_7=num_cond_b_7+1;
%              
%     end
% end
% 
% cond_b_7_av=cond_b_7/num_cond_b_7; 



% now let us compare all the average condition for each spin direction
% configuration

% [minval spin_index]=min([cond_b_1_av,cond_b_2_av,cond_b_3_av,cond_b_4_av,cond_b_5_av,cond_b_6_av,cond_b_7_av]);



%%-------------------------------------------------------------------------------------------------------------------------------------------------------
% main loop for optimization

% use the special trim trajectory segments 

% we have seven trim trajecotories 

% for each trim trajectory segment, we have a linearised system
% thus, theorectically we should check the controllability matrix for each
% trim trajectory, then we have totally n_trimtraj*n_thrusters
% Controllability matices


% first let us specify the desired generalised control input


% note that the foumula for calculating the desired input for each trim
% trajectory segmemt

% tau_d must be updated every time we change the geometic configurations since it will influence 
% the rigid body Coriolis matrix, since the geometric centre is changed 

% the dynamic states
% tau_d=C_RB(x_dyn,d_H,r_T,r_F)*x_dyn+C_A(x_dyn,d_H)*x_dyn+D(d_H,x_dyn)*x_dyn+g(x_kin,d_H,r_T,r_F)
% C_A and D actually should be affected by the geometric position as well.
% However, we assume that C_A and D is only determined by the hull geometry
% which is determined in the first optimization phase 

% Update the Coriolis matrix : note that the added mass Coriolis matrix and the drag matrix are not updated   




