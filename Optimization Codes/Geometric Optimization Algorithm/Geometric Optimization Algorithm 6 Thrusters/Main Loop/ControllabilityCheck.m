% ControllabilityCheck

% In this function we check the controllability after we perform one
% iteration: 
% 1. we perform the optimization for the control input: the linearization
%    point changes
% 2. we optimize the postion of actuators
% 3. we optimize the spin direction of each thursuter

% build the new dynamcis
% check the controllability

% use the global variable Ctrl to denote the controllability of the current
% system dynamics, when Ctrl=1, controllable, Ctrl=0, not controllable

% use the same method as in the preprocessing phase, 

% update the actuator configuration 

% remember that we have already already put the optimal value of all
% geometric variables into the global variable, the following part can be
% omitted 

T_S_1={veh.r_T1,veh.b_T1,veh.d_T1};
T_S_2={veh.r_T2,veh.b_T2,veh.d_T2}; 
T_S_3={veh.r_T3,veh.b_T3,veh.d_T3};
T_S_4={veh.r_T4,veh.b_T4,veh.d_T4};
T_S_5={veh.r_T5,veh.b_T5,veh.d_T5};
T_S_6={veh.r_T6,veh.b_T6,veh.d_T6};


%% update the error dynamics 

% update the current configuration

ConfigurationUpdate;

% update the dynamics 

DynamicsUpdate;

% we need to check the conver

% now we should build the error dynamcis for each trim trajectory

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

A_v_lambda_1=zeros(3);


% error dynamics for angular velocities

A_w_p_1=zeros(3);

A_w_lambda_1=zeros(3);


% The input matrix should will not be changed 

B_v_1=double(B_v_l);

B_w_1=double(B_w_l);


% formulate the error dynamics

A_E_1=[A_v_v_1 A_v_w_1 A_v_p_1 A_v_lambda_1;A_w_v_1 A_w_w_1 A_w_p_1 A_w_lambda_1;A_p_v_1 A_p_w_1 A_p_p_1 A_p_lambda_1;A_lambda_v_1 A_lambda_w_1 A_lambda_p_1 A_lambda_lambda_1];

B_E_1=[B_v_1;B_w_1;zeros(6)];

% store all system matrices and input matrices into a cluster



% Build the error dynamics controllability matrix 

Ctr_E_1=ctrb(A_E_1,B_E_1);

% store the matrix in cell of matrices

Ctr_E_1_c{index_loop}=Ctr_E_1;

% The current configuration must ensure all configurations are
% controllable, once it is not controllable, it will be 

if(rank(ctrb(A_E_1,B_E_1))==12)
    CtrlCheck=1;
else
    CtrlCheck=0;
end


% Calculate the condition number for the error dynamics

Cond_E_1=cond(Ctr_E_1);

% Cluster

Cond_E_1_c{index_loop}=Cond_E_1;

% the second trim trajectory 

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

A_v_lambda_2=zeros(3);


% error dynamics for angular velocities

A_w_p_2=zeros(3);

A_w_lambda_2=zeros(3);

% The input matrix should will not be changed 

B_v_2=double(B_v_l);

B_w_2=double(B_w_l);

% formulate the error dynamics

A_E_2=[A_v_v_2 A_v_w_2 A_v_p_2 A_v_lambda_2;A_w_v_2 A_w_w_2 A_w_p_2 A_w_lambda_2;A_p_v_2 A_p_w_2 A_p_p_2 A_p_lambda_2;A_lambda_v_2 A_lambda_w_2 A_lambda_p_2 A_lambda_lambda_2];

B_E_2=[B_v_2;B_w_2;zeros(6)];

% Build the error dynamics controllability matrix 

Ctr_E_2=ctrb(A_E_2,B_E_2);

% store the matrix in cell of matrices

Ctr_E_2_c{index_loop}=Ctr_E_2;

if(rank(ctrb(A_E_2,B_E_2))==12)
    CtrlCheck=1;
else
    CtrlCheck=0;
end

% Calculate the condition number for the error dynamics

Cond_E_2=cond(Ctr_E_2);

% Cluster

Cond_E_2_c{index_loop}=Cond_E_2;

% third trim trajectory

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

A_v_lambda_3=zeros(3);


% error dynamics for angular velocities

A_w_p_3=zeros(3);

A_w_lambda_3=zeros(3);


% The input matrix

B_v_3=double(B_v_l);

B_w_3=double(B_w_l);

% formulate the error dynamics

A_E_3=[A_v_v_3 A_v_w_3 A_v_p_3 A_v_lambda_3;A_w_v_3 A_w_w_3 A_w_p_3 A_w_lambda_3;A_p_v_3 A_p_w_3 A_p_p_3 A_p_lambda_3;A_lambda_v_3 A_lambda_w_3 A_lambda_p_3 A_lambda_lambda_3];

B_E_3=[B_v_3;B_w_3;zeros(6)];

% Build the error dynamics controllability matrix 

Ctr_E_3=ctrb(A_E_3,B_E_3);

% store the matrix in cell of matrices

Ctr_E_3_c{index_loop}=Ctr_E_3;

if(rank(ctrb(A_E_3,B_E_3))==12)
    CtrlCheck=1;
else
    CtrlCheck=0;
end

% Calculate the condition number for the error dynamics

Cond_E_3=cond(Ctr_E_3);

% Cluster

Cond_E_3_c{index_loop}=Cond_E_3;


% The forth trim trajecory

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

A_v_lambda_4=zeros(3);


% error dynamics for angular velocities

A_w_p_4=zeros(3);

A_w_lambda_4=zeros(3);

% The input matrix

B_v_4=double(B_v_l);

B_w_4=double(B_w_l);


% formulate the error dynamics

A_E_4=[A_v_v_4 A_v_w_4 A_v_p_4 A_v_lambda_4;A_w_v_4 A_w_w_4 A_w_p_4 A_w_lambda_4;A_p_v_4 A_p_w_4 A_p_p_4 A_p_lambda_4;A_lambda_v_4 A_lambda_w_4 A_lambda_p_4 A_lambda_lambda_4];

B_E_4=[B_v_4;B_w_4;zeros(6)];

% Build the error dynamics controllability matrix 

Ctr_E_4=ctrb(A_E_4,B_E_4);

% store the matrix in cell of matrices

Ctr_E_4_c{index_loop}=Ctr_E_4;

if(rank(ctrb(A_E_4,B_E_4))==12)
    CtrlCheck=1;
else
    CtrlCheck=0;
end

% Calculate the condition number for the error dynamics

Cond_E_4=cond(Ctr_E_4);

% Cluster

Cond_E_4_c{index_loop}=Cond_E_4;

% fifth trim trajectory


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

A_v_lambda_5=zeros(3);


% error dynamics for angular velocities

A_w_p_5=zeros(3);

A_w_lambda_5=zeros(3);

% formulate the error dynamics

A_E_5=[A_v_v_5 A_v_w_5 A_v_p_5 A_v_lambda_5;A_w_v_5 A_w_w_5 A_w_p_5 A_w_lambda_5;A_p_v_5 A_p_w_5 A_p_p_5 A_p_lambda_5;A_lambda_v_5 A_lambda_w_5 A_lambda_p_5 A_lambda_lambda_5];

% The input matrix

B_v_5=double(B_v_l);

B_w_5=double(B_w_l);

B_E_5=[B_v_5;B_w_5;zeros(6)];

% Build the error dynamics controllability matrix 

Ctr_E_5=ctrb(A_E_5,B_E_5);

% store the matrix in cell of matrices

Ctr_E_5_c{index_loop}=Ctr_E_5;

if(rank(ctrb(A_E_5,B_E_5))==12)
    CtrlCheck=1;
else
    CtrlCheck=0;
end

% Calculate the condition number for the error dynamics

Cond_E_5=cond(Ctr_E_5);

% Cluster

Cond_E_5_c{index_loop}=Cond_E_5;

% sixth trim trajectory

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

A_v_lambda_6=zeros(3); 


% error dynamics for angular velocities

A_w_p_6=zeros(3);

A_w_lambda_6=zeros(3);

% formulate the error dynamics

A_E_6=[A_v_v_6 A_v_w_6 A_v_p_6 A_v_lambda_6;A_w_v_6 A_w_w_6 A_w_p_6 A_w_lambda_6;A_p_v_6 A_p_w_6 A_p_p_6 A_p_lambda_6;A_lambda_v_6 A_lambda_w_6 A_lambda_p_6 A_lambda_lambda_6];

B_v_6=double(B_v_l);

B_w_6=double(B_w_l);

B_E_6=[B_v_6;B_w_6;zeros(6)];

% 

% Build the error dynamics controllability matrix 

Ctr_E_6=ctrb(A_E_6,B_E_6);

% store the matrix in cell of matrices

Ctr_E_6_c{index_loop}=Ctr_E_6;

if(rank(ctrb(A_E_6,B_E_6))==12)
    CtrlCheck=1;
else
    CtrlCheck=0;
end
% Calculate the condition number for the error dynamics

Cond_E_6=cond(Ctr_E_6);

% Cluster

Cond_E_6_c{index_loop}=Cond_E_6;


% seventh trim trajectory

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

A_v_lambda_7=zeros(3);


% error dynamics for angular velocities

A_w_p_7=zeros(3);

A_w_lambda_7=zeros(3);

% formulate the error dynamics

A_E_7=[A_v_v_7 A_v_w_7 A_v_p_7 A_v_lambda_7;A_w_v_7 A_w_w_7 A_w_p_7 A_w_lambda_7;A_p_v_7 A_p_w_7 A_p_p_7 A_p_lambda_7;A_lambda_v_7 A_lambda_w_7 A_lambda_p_7 A_lambda_lambda_7];

B_v_7=double(B_v_l);

B_w_7=double(B_w_l);

B_E_7=[B_v_7;B_w_7;zeros(6)];

% Build the error dynamics controllability matrix 

Ctr_E_7=ctrb(A_E_7,B_E_7);

% store the matrix in cell of matrices

Ctr_E_7_c{index_loop}=Ctr_E_7;

% determine the controllability matrix

if(rank(ctrb(A_E_7,B_E_7))==12)
    CtrlCheck=1;
else
    CtrlCheck=0;
end

% Calculate the condition number for the error dynamics

Cond_E_7=cond(Ctr_E_7);

% Cluster

Cond_E_7_c{index_loop}=Cond_E_7;























