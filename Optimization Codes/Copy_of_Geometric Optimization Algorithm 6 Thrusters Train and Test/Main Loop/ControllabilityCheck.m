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











