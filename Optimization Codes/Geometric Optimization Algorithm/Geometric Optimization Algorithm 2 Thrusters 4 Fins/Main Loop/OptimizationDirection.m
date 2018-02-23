%%  Yongyu Chen, formulate the optimization of the direction vector as convex optimization 

% in this file, we use QCQP to get the initial value of the direction optimization problem

% peform the initial guess for each thruster

% formulate the optimization of the direction vector as qcqp optimization
% problem, the optimization options are unchangable for all direction 
% optimization problems

% use sequential quadratic programming 

options_diropt_sqp = optimoptions('fmincon','Display','iter','Algorithm','sqp');


% read the constant geometric variable from the global variable veh, the
% spin direction comes from the preprocessing phase

b_T1=veh.b_T1;
b_T2=veh.b_T2;

% torque-force ratio

lambda_T1=veh.lambda1;
lambda_T2=veh.lambda2;



% read the postion vector from the the global vector veh

r_T1=veh.r_T1;
r_T2=veh.r_T2;


%% what we optimized in this case is the 



% these parameters will be updated in the following sequential optimization

%%

% calculate the current direction mapping matrix 

% the direction mapping matrix depends on geometric parameters from
% previous optimization phase, they will not change in this optimization
% phase

% for different trim trajectory the desired inout is different m trim
% trajectories

% for each thruster, the geometric variable is different, n thrusters

% Thus we have m X n direction mapping matrix

% first trim

B_dir_Trim1_1=DirObjFormulation(u_d_1(1),b_T1,lambda_T1,r_T1);

B_dir_Trim1_2=DirObjFormulation(u_d_1(2),b_T2,lambda_T2,r_T2);



% second trim 

B_dir_Trim2_1=DirObjFormulation(u_d_2(1),b_T1,lambda_T1,r_T1);

B_dir_Trim2_2=DirObjFormulation(u_d_2(2),b_T2,lambda_T2,r_T2);



% third trim 

B_dir_Trim3_1=DirObjFormulation(u_d_3(1),b_T1,lambda_T1,r_T1);

B_dir_Trim3_2=DirObjFormulation(u_d_3(2),b_T2,lambda_T2,r_T2);



% fourth trim 

B_dir_Trim4_1=DirObjFormulation(u_d_4(1),b_T1,lambda_T1,r_T1);

B_dir_Trim4_2=DirObjFormulation(u_d_4(2),b_T2,lambda_T2,r_T2);



% fifth trim 

B_dir_Trim5_1=DirObjFormulation(u_d_5(1),b_T1,lambda_T1,r_T1);

B_dir_Trim5_2=DirObjFormulation(u_d_5(2),b_T2,lambda_T2,r_T2);



% sixth trim 

B_dir_Trim6_1=DirObjFormulation(u_d_6(1),b_T1,lambda_T1,r_T1);

B_dir_Trim6_2=DirObjFormulation(u_d_6(2),b_T2,lambda_T2,r_T2);


% seventh trim 

B_dir_Trim7_1=DirObjFormulation(u_d_7(1),b_T1,lambda_T1,r_T1);

B_dir_Trim7_2=DirObjFormulation(u_d_7(2),b_T2,lambda_T2,r_T2);





% when we optimize the first direction vector, the other direction vectors
% are kept constantly

% optimization of all thrusters in sequence 

%% Thruster 1




% sum of generalised for other thrusters: 

sum_forces_Trim1_1=B_dir_Trim1_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_1)*u_d_1(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_1)*u_d_1(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_1)*u_d_1(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_1)*u_d_1(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim2_1=B_dir_Trim2_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_2)*u_d_2(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_2)*u_d_2(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_2)*u_d_2(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_2)*u_d_2(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim3_1=B_dir_Trim3_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_3)*u_d_3(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_3)*u_d_3(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_3)*u_d_3(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_3)*u_d_3(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim4_1=B_dir_Trim4_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_4)*u_d_4(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_4)*u_d_4(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_4)*u_d_4(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_4)*u_d_4(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim5_1=B_dir_Trim5_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_5)*u_d_5(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_5)*u_d_5(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_5)*u_d_5(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_5)*u_d_5(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim6_1=B_dir_Trim6_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_6)*u_d_6(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_6)*u_d_6(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_6)*u_d_6(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_6)*u_d_6(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim7_1=B_dir_Trim7_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_7)*u_d_7(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_7)*u_d_7(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_7)*u_d_7(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_7)*u_d_7(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
    

% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_1;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_1;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_1;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_1;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_1;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_1;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_1;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% change the format of the least square problem into quadratic format

% Convert || A*x-b ||^2 into the form 1/2 x'*P0*x + q0'*r + r0

% x'*A'*A*x-b'*A*x+b'*b
% 
% P0_T1=2*B_dir_1'*B_dir_1*n_trimtraj;
% 
% q0_T1=-B_dir_1'*tau_d_1_quasi-B_dir_1'*tau_d_2_quasi-B_dir_1'*tau_d_3_quasi-B_dir_1'*tau_d_4_quasi-B_dir_1'*tau_d_5_quasi-B_dir_1'*tau_d_6_quasi-B_dir_1'*tau_d_7_quasi;
% 
% r0_T1=tau_d_1_quasi'*tau_d_1_quasi+tau_d_2_quasi'*tau_d_2_quasi+tau_d_3_quasi'*tau_d_3_quasi+tau_d_4_quasi'*tau_d_4_quasi+tau_d_5_quasi'*tau_d_5_quasi+tau_d_6_quasi'*tau_d_6_quasi+tau_d_7_quasi'*tau_d_7_quasi;


% formulate the constraint, formulate the constraint into the form  1/2 x'*P1*x + q1'*r + r1 <= 0

% for our decision variable 

% Solved a QCQP with 3 inequalities:
%           minimize    1/2 x'*P0*x + q0'*r + r0
%               s.t.    1/2 x'*P1*x + q1'*r + r1 <= 0  
% and verifies that strong duality holds.

% dir is a three-dimensional vector

cvx_begin
    variable dir_T1(3)
    dual variables lam1
    % minimize( 0.5*quad_form(dir_T1,P0_T1) + q0_T1'*dir_T1 + r0_T1)
    
   minimize(norm(B_dir_Trim1_1*dir_T1-tau_d_1_quasi)+norm(B_dir_Trim2_1*dir_T1-tau_d_2_quasi)+norm(B_dir_Trim3_1*dir_T1-tau_d_3_quasi)+norm(B_dir_Trim4_1*dir_T1-tau_d_4_quasi)+...
    norm(B_dir_Trim5_1*dir_T1-tau_d_5_quasi)+norm(B_dir_Trim6_1*dir_T1-tau_d_6_quasi)+norm(B_dir_Trim7_1*dir_T1-tau_d_7_quasi))
    lam1: dir_T1'*dir_T1 - 1 <= 0;
cvx_end

obj1_T1 = cvx_optval;

% P_lam_T1 = P0_T1 + lam1*P1_T1;
% q_lam_T1 = q0_T1 + lam1*q1_T1;
% r_lam_T1 = r0_T1 + lam1*r1_T1;
% obj2 = -0.5*q_lam_T1'*inv(P_lam_T1)*q_lam_T1 + r_lam_T1;

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

% fun_diropt = @(x)0.5*[x(1),x(2),x(3)]*P0_T1*[x(1);x(2);x(3)] + q0_T1'*[x(1);x(2);x(3)] + r0_T1;

% fun_diropt=@(x)norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_1_quasi);


fun_diropt=@(x)norm(B_dir_Trim1_1*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_Trim2_1*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_Trim3_1*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_Trim4_1*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_Trim5_1*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_Trim6_1*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_Trim7_1*[x(1);x(2);x(3)]-tau_d_7_quasi);

A_diropt = [];
b_diropt = [];

Aeq_diropt = [];
beq_diropt = [];
lb_diropt = [];
ub_diropt = [];

% unit vector constraint
nonlcon_diropt = @nonl_constraints_dir;

% the initial value comes from the optimization result of QCQP
x0_diropt = dir_T1;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T1=x_diropt;


%% thruster 2



% sum of generalised for other thrusters

sum_forces_Trim1_2=B_dir_Trim1_1*veh.d_T1+...
    C_L*a_F1*b_F1*q(u_ind_1)*u_d_1(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_1)*u_d_1(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_1)*u_d_1(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_1)*u_d_1(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim2_2=B_dir_Trim2_1*veh.d_T1+...
    C_L*a_F1*b_F1*q(u_ind_2)*u_d_2(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_2)*u_d_2(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_2)*u_d_2(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_2)*u_d_2(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim3_2=B_dir_Trim3_1*veh.d_T1+...
    C_L*a_F1*b_F1*q(u_ind_3)*u_d_3(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_3)*u_d_3(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_3)*u_d_3(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_3)*u_d_3(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim4_2=B_dir_Trim4_1*veh.d_T1+...
    C_L*a_F1*b_F1*q(u_ind_4)*u_d_4(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_4)*u_d_4(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_4)*u_d_4(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_4)*u_d_4(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim5_2=B_dir_Trim5_1*veh.d_T1+...
    C_L*a_F1*b_F1*q(u_ind_5)*u_d_5(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_5)*u_d_5(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_5)*u_d_5(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_5)*u_d_5(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim6_2=B_dir_Trim6_1*veh.d_T1+...
    C_L*a_F1*b_F1*q(u_ind_6)*u_d_6(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_6)*u_d_6(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_6)*u_d_6(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_6)*u_d_6(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
sum_forces_Trim7_2=B_dir_Trim7_1*veh.d_T1+...
    C_L*a_F1*b_F1*q(u_ind_7)*u_d_7(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_7)*u_d_7(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_7)*u_d_7(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_7)*u_d_7(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];
    



% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_2;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_2;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_2;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_2;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_2;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_2;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_2;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint, formulate the constraint into the form  1/2 x'*P1*x + q1'*r + r1 <= 0


% Solved a QCQP with 3 inequalities:
%           minimize    1/2 x'*P0*x + q0'*r + r0
%               s.t.    1/2 x'*P1*x + q1'*r + r1 <= 0  
% and verifies that strong duality holds.

% dir is a three-dimensional vector

cvx_begin
    variable dir_T2(3)
    dual variables lam2
    minimize(norm(B_dir_Trim1_2*dir_T2-tau_d_1_quasi)+norm(B_dir_Trim2_2*dir_T2-tau_d_2_quasi)+norm(B_dir_Trim3_2*dir_T2-tau_d_3_quasi)+norm(B_dir_Trim4_2*dir_T2-tau_d_4_quasi)+...
    norm(B_dir_Trim5_2*dir_T2-tau_d_5_quasi)+norm(B_dir_Trim6_2*dir_T2-tau_d_6_quasi)+norm(B_dir_Trim7_2*dir_T2-tau_d_7_quasi))
    lam2: dir_T2'*dir_T2 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_Trim1_2*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_Trim2_2*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_Trim3_2*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_Trim4_2*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_Trim5_2*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_Trim6_2*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_Trim7_2*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T2;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T2=x_diropt;

%% optimization setting  for the fin orientation angle gamma

options_gammaopt_sqp = optimoptions('fmincon','Display','iter','Algorithm','sqp');


%% optimization for fins

% read the value of fin direction gamma

gamma_F1=veh.gamma_F1;

gamma_F2=veh.gamma_F2;

gamma_F3=veh.gamma_F3;

gamma_F4=veh.gamma_F4;


%% fin 1

% the optimization is formulated as nonlinear optimization problem, the
% intial value of the first iteration is radnomly generated
% the initial value of last iteration is the result from the last iteration

% do not forget the input signal 

sum_forces_Trim1_Fin1=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+...
    C_L*a_F2*b_F2*q(u_ind_1)*u_d_1(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_1)*u_d_1(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_1)*u_d_1(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim2_Fin1=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+...
    C_L*a_F2*b_F2*q(u_ind_2)*u_d_2(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_2)*u_d_2(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_2)*u_d_2(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim3_Fin1=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+...
    C_L*a_F2*b_F2*q(u_ind_3)*u_d_3(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_3)*u_d_3(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_3)*u_d_3(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim4_Fin1=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+...
    C_L*a_F2*b_F2*q(u_ind_4)*u_d_4(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_4)*u_d_4(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_4)*u_d_4(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim5_Fin1=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+...
    C_L*a_F2*b_F2*q(u_ind_5)*u_d_5(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_5)*u_d_5(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_5)*u_d_5(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim6_Fin1=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+...
    C_L*a_F2*b_F2*q(u_ind_6)*u_d_6(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_6)*u_d_6(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_6)*u_d_6(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim7_Fin1=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+...
    C_L*a_F2*b_F2*q(u_ind_7)*u_d_7(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_7)*u_d_7(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_7)*u_d_7(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

% calculate the quasi generalised torque 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_Fin1;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_Fin1;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_Fin1;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_Fin1;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_Fin1;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_Fin1;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_Fin1;

% constraints

A_gammaopt = [];
b_gammaopt = [];

Aeq_gammaopt = [];
beq_gammaopt = [];

% The angle gamma should be located in the range [-pi,pi]
lb_gammaopt = -pi;
ub_gammaopt = pi;

% we formulate the optimization as nonlinear progeramming for gamma_F1

% decision variable x_gamma: one dimensional 

fun_gammaopt=@(x_gamma)norm(C_L*a_F1*b_F1*q(u_ind_1)*u_d_1(3)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_1_quasi)+...
    norm(C_L*a_F1*b_F1*q(u_ind_2)*u_d_2(3)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_2_quasi)+...
    norm(C_L*a_F1*b_F1*q(u_ind_3)*u_d_3(3)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_3_quasi)+...
    norm(C_L*a_F1*b_F1*q(u_ind_4)*u_d_4(3)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_4_quasi)+...
    norm(C_L*a_F1*b_F1*q(u_ind_5)*u_d_5(3)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_5_quasi)+...
    norm(C_L*a_F1*b_F1*q(u_ind_6)*u_d_6(3)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_6_quasi)+...
    norm(C_L*a_F1*b_F1*q(u_ind_7)*u_d_7(3)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_7_quasi);


x0_gammaopt = veh.gamma_F1;

[x_gammaopt,fval_gammaopt,exitflag,output] = fmincon(fun_gammaopt,x0_gammaopt,A_gammaopt,b_gammaopt,Aeq_gammaopt,beq_gammaopt,lb_gammaopt,ub_gammaopt,[],options_gammaopt_sqp);

% use the optimal direction vector

veh.gamma_F1=x_gammaopt;

    
    




%% fin 2

% the optimization is formulated as nonlinear optimization problem, the
% intial value of the first iteration is radnomly generated
% the initial value of last iteration is the result from the last iteration

sum_forces_Trim1_Fin2=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_1)*u_d_1(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F3*b_F3*q(u_ind_1)*u_d_1(5)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_1)*u_d_1(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim2_Fin2=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_2)*u_d_2(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F3*b_F3*q(u_ind_2)*u_d_2(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_2)*u_d_2(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim3_Fin2=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_3)*u_d_3(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F3*b_F3*q(u_ind_3)*u_d_3(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_3)*u_d_3(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim4_Fin2=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_4)*u_d_4(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F3*b_F3*q(u_ind_4)*u_d_4(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_4)*u_d_4(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim5_Fin2=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_5)*u_d_5(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F3*b_F3*q(u_ind_5)*u_d_5(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_5)*u_d_5(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim6_Fin2=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_6)*u_d_6(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F3*b_F3*q(u_ind_6)*u_d_6(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_6)*u_d_6(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim7_Fin2=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_7)*u_d_7(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F3*b_F3*q(u_ind_7)*u_d_7(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)]+...
    C_L*a_F4*b_F4*q(u_ind_7)*u_d_7(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

% calculate the quasi generalised torque 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_Fin2;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_Fin2;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_Fin2;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_Fin2;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_Fin2;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_Fin2;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_Fin2;

% we formulate the optimization as nonlinear progeramming for gamma_F2

% decision variable x_gamma: one dimensional 

fun_gammaopt=@(x_gamma)norm(C_L*a_F2*b_F2*q(u_ind_1)*u_d_1(4)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F2*sin(x_gamma);veh.x_F2*cos(x_gamma)]-tau_d_1_quasi)+...
    norm(C_L*a_F2*b_F2*q(u_ind_2)*u_d_2(4)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F2*sin(x_gamma);veh.x_F2*cos(x_gamma)]-tau_d_2_quasi)+...
    norm(C_L*a_F2*b_F2*q(u_ind_3)*u_d_3(4)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F2*sin(x_gamma);veh.x_F2*cos(x_gamma)]-tau_d_3_quasi)+...
    norm(C_L*a_F2*b_F2*q(u_ind_4)*u_d_4(4)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F2*sin(x_gamma);veh.x_F2*cos(x_gamma)]-tau_d_4_quasi)+...
    norm(C_L*a_F2*b_F2*q(u_ind_5)*u_d_5(4)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F2*sin(x_gamma);veh.x_F2*cos(x_gamma)]-tau_d_5_quasi)+...
    norm(C_L*a_F2*b_F2*q(u_ind_6)*u_d_6(4)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F2*sin(x_gamma);veh.x_F2*cos(x_gamma)]-tau_d_6_quasi)+...
    norm(C_L*a_F2*b_F2*q(u_ind_7)*u_d_7(4)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F2);veh.x_F2*sin(x_gamma);veh.x_F2*cos(x_gamma)]-tau_d_7_quasi);


x0_gammaopt = veh.gamma_F2;

[x_gammaopt,fval_gammaopt,exitflag,output] = fmincon(fun_gammaopt,x0_gammaopt,A_gammaopt,b_gammaopt,Aeq_gammaopt,beq_gammaopt,lb_gammaopt,ub_gammaopt,[],options_gammaopt_sqp);

% use the optimal direction vector

veh.gamma_F2=x_gammaopt;

% 

%% fin 3

% the optimization is formulated as nonlinear optimization problem, the
% intial value of the first iteration is radnomly generated
% the initial value of last iteration is the result from the last iteration

sum_forces_Trim1_Fin3=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_1)*u_d_1(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_1)*u_d_1(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F4*b_F4*q(u_ind_1)*u_d_1(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim2_Fin3=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_2)*u_d_2(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_2)*u_d_2(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F4*b_F4*q(u_ind_2)*u_d_2(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim3_Fin3=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_3)*u_d_3(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_3)*u_d_3(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F4*b_F4*q(u_ind_3)*u_d_3(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim4_Fin3=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_4)*u_d_4(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_4)*u_d_4(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F4*b_F4*q(u_ind_4)*u_d_4(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim5_Fin3=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_5)*u_d_5(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_5)*u_d_5(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F3*sin(veh.gamma_F2);veh.x_F3*cos(veh.gamma_F2)]+...
    C_L*a_F4*b_F4*q(u_ind_5)*u_d_5(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim6_Fin3=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_6)*u_d_6(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_6)*u_d_6(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F3*sin(veh.gamma_F2);veh.x_F3*cos(veh.gamma_F2)]+...
    C_L*a_F4*b_F4*q(u_ind_6)*u_d_6(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

sum_forces_Trim7_Fin3=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_7)*u_d_7(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_7)*u_d_7(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F3*cos(veh.gamma_F2)]+...
    C_L*a_F4*b_F4*q(u_ind_7)*u_d_7(6)*[0;cos(veh.gamma_F4);-sin(veh.gamma_F4);-0.5*(d_H+a_F4);veh.x_F4*sin(veh.gamma_F4);veh.x_F4*cos(veh.gamma_F4)];

% optimization the fin geometric parameter only

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_Fin3;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_Fin3;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_Fin3;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_Fin3;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_Fin3;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_Fin3;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_Fin3;

% we formulate the optimization as nonlinear progeramming for gamma_F3

% decision variable x_gamma: one dimensional 

fun_gammaopt=@(x_gamma)norm(C_L*a_F3*b_F3*q(u_ind_1)*u_d_1(5)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F3);veh.x_F3*sin(x_gamma);veh.x_F3*cos(x_gamma)]-tau_d_1_quasi)+...
    norm(C_L*a_F3*b_F3*q(u_ind_2)*u_d_2(5)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F3);veh.x_F3*sin(x_gamma);veh.x_F3*cos(x_gamma)]-tau_d_2_quasi)+...
    norm(C_L*a_F3*b_F3*q(u_ind_3)*u_d_3(5)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F3);veh.x_F3*sin(x_gamma);veh.x_F3*cos(x_gamma)]-tau_d_3_quasi)+...
    norm(C_L*a_F3*b_F3*q(u_ind_4)*u_d_4(5)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F3);veh.x_F3*sin(x_gamma);veh.x_F3*cos(x_gamma)]-tau_d_4_quasi)+...
    norm(C_L*a_F3*b_F3*q(u_ind_5)*u_d_5(5)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F3);veh.x_F3*sin(x_gamma);veh.x_F3*cos(x_gamma)]-tau_d_5_quasi)+...
    norm(C_L*a_F3*b_F3*q(u_ind_6)*u_d_6(5)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F3);veh.x_F3*sin(x_gamma);veh.x_F3*cos(x_gamma)]-tau_d_6_quasi)+...
    norm(C_L*a_F3*b_F3*q(u_ind_7)*u_d_7(5)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F3);veh.x_F3*sin(x_gamma);veh.x_F3*cos(x_gamma)]-tau_d_7_quasi);


x0_gammaopt = veh.gamma_F3;

[x_gammaopt,fval_gammaopt,exitflag,output] = fmincon(fun_gammaopt,x0_gammaopt,A_gammaopt,b_gammaopt,Aeq_gammaopt,beq_gammaopt,lb_gammaopt,ub_gammaopt,[],options_gammaopt_sqp);

% use the optimal direction vector

veh.gamma_F3=x_gammaopt;





%% fin 4

% the optimization is formulated as nonlinear optimization problem, the
% intial value of the first iteration is radnomly generated
% the initial value of last iteration is the result from the last iteration

sum_forces_Trim1_Fin4=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_1)*u_d_1(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_1)*u_d_1(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_1)*u_d_1(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)];

sum_forces_Trim2_Fin4=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_2)*u_d_2(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_2)*u_d_2(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_2)*u_d_2(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)];

sum_forces_Trim3_Fin4=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_3)*u_d_3(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_3)*u_d_3(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_3)*u_d_3(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)];

sum_forces_Trim4_Fin4=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_4)*u_d_4(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_4)*u_d_4(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_4)*u_d_4(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)];

sum_forces_Trim5_Fin4=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_5)*u_d_5(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_5)*u_d_5(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_5)*u_d_5(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)];

sum_forces_Trim6_Fin4=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_6)*u_d_6(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_6)*u_d_6(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_6)*u_d_6(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)];

sum_forces_Trim7_Fin4=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+...
    C_L*a_F1*b_F1*q(u_ind_7)*u_d_7(3)*[0;cos(veh.gamma_F1);-sin(veh.gamma_F1);-0.5*(d_H+a_F1);veh.x_F1*sin(veh.gamma_F1);veh.x_F1*cos(veh.gamma_F1)]+...
    C_L*a_F2*b_F2*q(u_ind_7)*u_d_7(4)*[0;cos(veh.gamma_F2);-sin(veh.gamma_F2);-0.5*(d_H+a_F2);veh.x_F2*sin(veh.gamma_F2);veh.x_F2*cos(veh.gamma_F2)]+...
    C_L*a_F3*b_F3*q(u_ind_7)*u_d_7(5)*[0;cos(veh.gamma_F3);-sin(veh.gamma_F3);-0.5*(d_H+a_F3);veh.x_F3*sin(veh.gamma_F3);veh.x_F3*cos(veh.gamma_F3)];

% the quasi desired torque

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_Fin4;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_Fin4;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_Fin4;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_Fin4;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_Fin4;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_Fin4;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_Fin4;

% now formulate nonlinear programming for the fin orientatiton gamma_F4

% decision variable x_gamma: one dimensional 

fun_gammaopt=@(x_gamma)norm(C_L*a_F4*b_F4*q(u_ind_1)*u_d_1(6)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F4);veh.x_F1*sin(x_gamma);veh.x_F1*cos(x_gamma)]-tau_d_1_quasi)+...
    norm(C_L*a_F4*b_F4*q(u_ind_2)*u_d_2(6)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F4);veh.x_F4*sin(x_gamma);veh.x_F4*cos(x_gamma)]-tau_d_2_quasi)+...
    norm(C_L*a_F4*b_F4*q(u_ind_3)*u_d_3(6)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F4);veh.x_F4*sin(x_gamma);veh.x_F4*cos(x_gamma)]-tau_d_3_quasi)+...
    norm(C_L*a_F4*b_F4*q(u_ind_4)*u_d_4(6)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F4);veh.x_F4*sin(x_gamma);veh.x_F4*cos(x_gamma)]-tau_d_4_quasi)+...
    norm(C_L*a_F4*b_F4*q(u_ind_5)*u_d_5(6)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F4);veh.x_F4*sin(x_gamma);veh.x_F4*cos(x_gamma)]-tau_d_5_quasi)+...
    norm(C_L*a_F4*b_F4*q(u_ind_6)*u_d_6(6)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F4);veh.x_F4*sin(x_gamma);veh.x_F4*cos(x_gamma)]-tau_d_6_quasi)+...
    norm(C_L*a_F4*b_F4*q(u_ind_7)*u_d_7(6)*[0;cos(x_gamma);-sin(x_gamma);-0.5*(d_H+a_F4);veh.x_F4*sin(x_gamma);veh.x_F4*cos(x_gamma)]-tau_d_7_quasi);


x0_gammaopt = veh.gamma_F4;

[x_gammaopt,fval_gammaopt,exitflag,output] = fmincon(fun_gammaopt,x0_gammaopt,A_gammaopt,b_gammaopt,Aeq_gammaopt,beq_gammaopt,lb_gammaopt,ub_gammaopt,[],options_gammaopt_sqp);

% use the optimal direction vector

veh.gamma_F4=x_gammaopt;



%% after optimization assign back the optimal value 

% veh.gamma_F1=gamma_F1;
% 
% veh.gamma_F2=gamma_F2;
% 
% veh.gamma_F3=gamma_F3;
% 
% veh.gamma_F4=gamma_F4;







