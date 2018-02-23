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
b_T3=veh.b_T3;
b_T4=veh.b_T4;
b_T5=veh.b_T5;
b_T6=veh.b_T6;

% torque-force ratio

lambda_T1=veh.lambda1;
lambda_T2=veh.lambda2;
lambda_T3=veh.lambda3;
lambda_T4=veh.lambda4;
lambda_T5=veh.lambda5;
lambda_T6=veh.lambda6;


% read the postion vector from the the global vector veh

r_T1=veh.r_T1;
r_T2=veh.r_T2;
r_T3=veh.r_T3;
r_T4=veh.r_T4;
r_T5=veh.r_T5;
r_T6=veh.r_T6;



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

B_dir_Trim1_3=DirObjFormulation(u_d_1(3),b_T3,lambda_T3,r_T3);

B_dir_Trim1_4=DirObjFormulation(u_d_1(4),b_T4,lambda_T4,r_T4);

B_dir_Trim1_5=DirObjFormulation(u_d_1(5),b_T5,lambda_T5,r_T5);

B_dir_Trim1_6=DirObjFormulation(u_d_1(6),b_T6,lambda_T6,r_T6);

% second trim 

B_dir_Trim2_1=DirObjFormulation(u_d_2(1),b_T1,lambda_T1,r_T1);

B_dir_Trim2_2=DirObjFormulation(u_d_2(2),b_T2,lambda_T2,r_T2);

B_dir_Trim2_3=DirObjFormulation(u_d_2(3),b_T3,lambda_T3,r_T3);

B_dir_Trim2_4=DirObjFormulation(u_d_2(4),b_T4,lambda_T4,r_T4);

B_dir_Trim2_5=DirObjFormulation(u_d_2(5),b_T5,lambda_T5,r_T5);

B_dir_Trim2_6=DirObjFormulation(u_d_2(6),b_T6,lambda_T6,r_T6);

% third trim 

B_dir_Trim3_1=DirObjFormulation(u_d_3(1),b_T1,lambda_T1,r_T1);

B_dir_Trim3_2=DirObjFormulation(u_d_3(2),b_T2,lambda_T2,r_T2);

B_dir_Trim3_3=DirObjFormulation(u_d_3(3),b_T3,lambda_T3,r_T3);

B_dir_Trim3_4=DirObjFormulation(u_d_3(4),b_T4,lambda_T4,r_T4);

B_dir_Trim3_5=DirObjFormulation(u_d_3(5),b_T5,lambda_T5,r_T5);

B_dir_Trim3_6=DirObjFormulation(u_d_3(6),b_T6,lambda_T6,r_T6);

% fourth trim 

B_dir_Trim4_1=DirObjFormulation(u_d_4(1),b_T1,lambda_T1,r_T1);

B_dir_Trim4_2=DirObjFormulation(u_d_4(2),b_T2,lambda_T2,r_T2);

B_dir_Trim4_3=DirObjFormulation(u_d_4(3),b_T3,lambda_T3,r_T3);

B_dir_Trim4_4=DirObjFormulation(u_d_4(4),b_T4,lambda_T4,r_T4);

B_dir_Trim4_5=DirObjFormulation(u_d_4(5),b_T5,lambda_T5,r_T5);

B_dir_Trim4_6=DirObjFormulation(u_d_4(6),b_T6,lambda_T6,r_T6);

% fifth trim 

B_dir_Trim5_1=DirObjFormulation(u_d_5(1),b_T1,lambda_T1,r_T1);

B_dir_Trim5_2=DirObjFormulation(u_d_5(2),b_T2,lambda_T2,r_T2);

B_dir_Trim5_3=DirObjFormulation(u_d_5(3),b_T3,lambda_T3,r_T3);

B_dir_Trim5_4=DirObjFormulation(u_d_5(4),b_T4,lambda_T4,r_T4);

B_dir_Trim5_5=DirObjFormulation(u_d_5(5),b_T5,lambda_T5,r_T5);

B_dir_Trim5_6=DirObjFormulation(u_d_5(6),b_T6,lambda_T6,r_T6);

% sixth trim 

B_dir_Trim6_1=DirObjFormulation(u_d_6(1),b_T1,lambda_T1,r_T1);

B_dir_Trim6_2=DirObjFormulation(u_d_6(2),b_T2,lambda_T2,r_T2);

B_dir_Trim6_3=DirObjFormulation(u_d_6(3),b_T3,lambda_T3,r_T3);

B_dir_Trim6_4=DirObjFormulation(u_d_6(4),b_T4,lambda_T4,r_T4);

B_dir_Trim6_5=DirObjFormulation(u_d_6(5),b_T5,lambda_T5,r_T5);

B_dir_Trim6_6=DirObjFormulation(u_d_6(6),b_T6,lambda_T6,r_T6);

% seventh trim 

B_dir_Trim7_1=DirObjFormulation(u_d_7(1),b_T1,lambda_T1,r_T1);

B_dir_Trim7_2=DirObjFormulation(u_d_7(2),b_T2,lambda_T2,r_T2);

B_dir_Trim7_3=DirObjFormulation(u_d_7(3),b_T3,lambda_T3,r_T3);

B_dir_Trim7_4=DirObjFormulation(u_d_7(4),b_T4,lambda_T4,r_T4);

B_dir_Trim7_5=DirObjFormulation(u_d_7(5),b_T5,lambda_T5,r_T5);

B_dir_Trim7_6=DirObjFormulation(u_d_7(6),b_T6,lambda_T6,r_T6);



% when we optimize the first direction vector, the other direction vectors
% are kept constantly

% optimization of all thrusters in sequence 

%% Thruster 1




% sum of generalised for other thrusters: 

sum_forces_Trim1_1=B_dir_Trim1_2*veh.d_T2+B_dir_Trim1_3*veh.d_T3+B_dir_Trim1_4*veh.d_T4+B_dir_Trim1_5*veh.d_T5+B_dir_Trim1_6*veh.d_T6;
sum_forces_Trim2_1=B_dir_Trim2_2*veh.d_T2+B_dir_Trim2_3*veh.d_T3+B_dir_Trim2_4*veh.d_T4+B_dir_Trim2_5*veh.d_T5+B_dir_Trim2_6*veh.d_T6;
sum_forces_Trim3_1=B_dir_Trim3_2*veh.d_T2+B_dir_Trim3_3*veh.d_T3+B_dir_Trim3_4*veh.d_T4+B_dir_Trim3_5*veh.d_T5+B_dir_Trim3_6*veh.d_T6;
sum_forces_Trim4_1=B_dir_Trim4_2*veh.d_T2+B_dir_Trim4_3*veh.d_T3+B_dir_Trim4_4*veh.d_T4+B_dir_Trim4_5*veh.d_T5+B_dir_Trim4_6*veh.d_T6;
sum_forces_Trim5_1=B_dir_Trim5_2*veh.d_T2+B_dir_Trim5_3*veh.d_T3+B_dir_Trim5_4*veh.d_T4+B_dir_Trim5_5*veh.d_T5+B_dir_Trim5_6*veh.d_T6;
sum_forces_Trim6_1=B_dir_Trim6_2*veh.d_T2+B_dir_Trim6_3*veh.d_T3+B_dir_Trim6_4*veh.d_T4+B_dir_Trim6_5*veh.d_T5+B_dir_Trim6_6*veh.d_T6;
sum_forces_Trim7_1=B_dir_Trim7_2*veh.d_T2+B_dir_Trim7_3*veh.d_T3+B_dir_Trim7_4*veh.d_T4+B_dir_Trim7_5*veh.d_T5+B_dir_Trim7_6*veh.d_T6;

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

sum_forces_Trim1_2=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_3*veh.d_T3+B_dir_Trim1_4*veh.d_T4+B_dir_Trim1_5*veh.d_T5+B_dir_Trim1_6*veh.d_T6;
sum_forces_Trim2_2=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_3*veh.d_T3+B_dir_Trim2_4*veh.d_T4+B_dir_Trim2_5*veh.d_T5+B_dir_Trim2_6*veh.d_T6;
sum_forces_Trim3_2=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_3*veh.d_T3+B_dir_Trim3_4*veh.d_T4+B_dir_Trim3_5*veh.d_T5+B_dir_Trim3_6*veh.d_T6;
sum_forces_Trim4_2=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_3*veh.d_T3+B_dir_Trim4_4*veh.d_T4+B_dir_Trim4_5*veh.d_T5+B_dir_Trim4_6*veh.d_T6;
sum_forces_Trim5_2=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_3*veh.d_T3+B_dir_Trim5_4*veh.d_T4+B_dir_Trim5_5*veh.d_T5+B_dir_Trim5_6*veh.d_T6;
sum_forces_Trim6_2=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_3*veh.d_T3+B_dir_Trim6_4*veh.d_T4+B_dir_Trim6_5*veh.d_T5+B_dir_Trim6_6*veh.d_T6;
sum_forces_Trim7_2=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_3*veh.d_T3+B_dir_Trim7_4*veh.d_T4+B_dir_Trim7_5*veh.d_T5+B_dir_Trim7_6*veh.d_T6;



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

% 
%% thruster 3



% sum of generalised for other thrusters

sum_forces_Trim1_3=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+B_dir_Trim1_4*veh.d_T4+B_dir_Trim1_5*veh.d_T5+B_dir_Trim1_6*veh.d_T6;
sum_forces_Trim2_3=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+B_dir_Trim2_4*veh.d_T4+B_dir_Trim2_5*veh.d_T5+B_dir_Trim2_6*veh.d_T6;
sum_forces_Trim3_3=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+B_dir_Trim3_4*veh.d_T4+B_dir_Trim3_5*veh.d_T5+B_dir_Trim3_6*veh.d_T6;
sum_forces_Trim4_3=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+B_dir_Trim4_4*veh.d_T4+B_dir_Trim4_5*veh.d_T5+B_dir_Trim4_6*veh.d_T6;
sum_forces_Trim5_3=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+B_dir_Trim5_4*veh.d_T4+B_dir_Trim5_5*veh.d_T5+B_dir_Trim5_6*veh.d_T6;
sum_forces_Trim6_3=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+B_dir_Trim6_4*veh.d_T4+B_dir_Trim6_5*veh.d_T5+B_dir_Trim6_6*veh.d_T6;
sum_forces_Trim7_3=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+B_dir_Trim7_4*veh.d_T4+B_dir_Trim7_5*veh.d_T5+B_dir_Trim7_6*veh.d_T6;



% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_3;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_3;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_3;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_3;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_3;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_3;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_3;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T3(3)
    dual variables lam3
    minimize(norm(B_dir_Trim1_3*dir_T3-tau_d_1_quasi)+norm(B_dir_Trim2_3*dir_T3-tau_d_2_quasi)+norm(B_dir_Trim3_3*dir_T3-tau_d_3_quasi)+norm(B_dir_Trim4_3*dir_T3-tau_d_4_quasi)+...
    norm(B_dir_Trim5_3*dir_T3-tau_d_5_quasi)+norm(B_dir_Trim6_3*dir_T3-tau_d_6_quasi)+norm(B_dir_Trim7_3*dir_T3-tau_d_7_quasi))
    lam3: dir_T3'*dir_T3 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_Trim1_3*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_Trim2_3*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_Trim3_3*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_Trim4_3*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_Trim5_3*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_Trim6_3*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_Trim7_3*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T3;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T3=x_diropt;

%% thruster 4


% sum of generalised for other thrusters

sum_forces_Trim1_4=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+B_dir_Trim1_3*veh.d_T3+B_dir_Trim1_5*veh.d_T5+B_dir_Trim1_6*veh.d_T6;
sum_forces_Trim2_4=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+B_dir_Trim2_3*veh.d_T3+B_dir_Trim2_5*veh.d_T5+B_dir_Trim2_6*veh.d_T6;
sum_forces_Trim3_4=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+B_dir_Trim3_3*veh.d_T3+B_dir_Trim3_5*veh.d_T5+B_dir_Trim3_6*veh.d_T6;
sum_forces_Trim4_4=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+B_dir_Trim4_3*veh.d_T3+B_dir_Trim4_5*veh.d_T5+B_dir_Trim4_6*veh.d_T6;
sum_forces_Trim5_4=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+B_dir_Trim5_3*veh.d_T3+B_dir_Trim5_5*veh.d_T5+B_dir_Trim5_6*veh.d_T6;
sum_forces_Trim6_4=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+B_dir_Trim6_3*veh.d_T3+B_dir_Trim6_5*veh.d_T5+B_dir_Trim6_6*veh.d_T6;
sum_forces_Trim7_4=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+B_dir_Trim7_3*veh.d_T3+B_dir_Trim7_5*veh.d_T5+B_dir_Trim7_6*veh.d_T6;

% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_4;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_4;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_4;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_4;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_4;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_4;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_4;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T4(3)
    dual variables lam4
    minimize(norm(B_dir_Trim1_4*dir_T4-tau_d_1_quasi)+norm(B_dir_Trim2_4*dir_T4-tau_d_2_quasi)+norm(B_dir_Trim3_4*dir_T4-tau_d_3_quasi)+norm(B_dir_Trim4_4*dir_T4-tau_d_4_quasi)+...
    norm(B_dir_Trim5_4*dir_T4-tau_d_5_quasi)+norm(B_dir_Trim6_4*dir_T4-tau_d_6_quasi)+norm(B_dir_Trim7_4*dir_T4-tau_d_7_quasi))
    lam4: dir_T4'*dir_T4 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_Trim1_4*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_Trim2_4*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_Trim3_4*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_Trim4_4*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_Trim5_4*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_Trim6_4*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_Trim7_4*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T4;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T4=x_diropt;

%% thruster 5


% sum of generalised for other thrusters

sum_forces_Trim1_5=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+B_dir_Trim1_3*veh.d_T3+B_dir_Trim1_4*veh.d_T4+B_dir_Trim1_6*veh.d_T6;
sum_forces_Trim2_5=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+B_dir_Trim2_3*veh.d_T3+B_dir_Trim2_4*veh.d_T4+B_dir_Trim2_6*veh.d_T6;
sum_forces_Trim3_5=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+B_dir_Trim3_3*veh.d_T3+B_dir_Trim3_4*veh.d_T4+B_dir_Trim3_6*veh.d_T6;
sum_forces_Trim4_5=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+B_dir_Trim4_3*veh.d_T3+B_dir_Trim4_4*veh.d_T4+B_dir_Trim4_6*veh.d_T6;
sum_forces_Trim5_5=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+B_dir_Trim5_3*veh.d_T3+B_dir_Trim5_4*veh.d_T4+B_dir_Trim5_6*veh.d_T6;
sum_forces_Trim6_5=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+B_dir_Trim6_3*veh.d_T3+B_dir_Trim6_4*veh.d_T4+B_dir_Trim6_6*veh.d_T6;
sum_forces_Trim7_5=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+B_dir_Trim7_3*veh.d_T3+B_dir_Trim7_4*veh.d_T4+B_dir_Trim7_6*veh.d_T6;



% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_5;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_5;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_5;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_5;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_5;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_5;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_5;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T5(3)
    dual variables lam5
    minimize(norm(B_dir_Trim1_5*dir_T5-tau_d_1_quasi)+norm(B_dir_Trim2_5*dir_T5-tau_d_2_quasi)+norm(B_dir_Trim3_5*dir_T5-tau_d_3_quasi)+norm(B_dir_Trim4_5*dir_T5-tau_d_4_quasi)+...
    norm(B_dir_Trim5_5*dir_T5-tau_d_5_quasi)+norm(B_dir_Trim6_5*dir_T5-tau_d_6_quasi)+norm(B_dir_Trim7_5*dir_T5-tau_d_7_quasi))
    lam5: dir_T5'*dir_T5 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_Trim1_5*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_Trim2_5*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_Trim3_5*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_Trim4_5*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_Trim5_5*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_Trim6_5*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_Trim7_5*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T5;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T5=x_diropt;

%% thruster 6

% sum of generalised for other thrusters

sum_forces_Trim1_6=B_dir_Trim1_1*veh.d_T1+B_dir_Trim1_2*veh.d_T2+B_dir_Trim1_3*veh.d_T3+B_dir_Trim1_4*veh.d_T4+B_dir_Trim1_5*veh.d_T5;
sum_forces_Trim2_6=B_dir_Trim2_1*veh.d_T1+B_dir_Trim2_2*veh.d_T2+B_dir_Trim2_3*veh.d_T3+B_dir_Trim2_4*veh.d_T4+B_dir_Trim2_5*veh.d_T5;
sum_forces_Trim3_6=B_dir_Trim3_1*veh.d_T1+B_dir_Trim3_2*veh.d_T2+B_dir_Trim3_3*veh.d_T3+B_dir_Trim3_4*veh.d_T4+B_dir_Trim3_5*veh.d_T5;
sum_forces_Trim4_6=B_dir_Trim4_1*veh.d_T1+B_dir_Trim4_2*veh.d_T2+B_dir_Trim4_3*veh.d_T3+B_dir_Trim4_4*veh.d_T4+B_dir_Trim4_5*veh.d_T5;
sum_forces_Trim5_6=B_dir_Trim5_1*veh.d_T1+B_dir_Trim5_2*veh.d_T2+B_dir_Trim5_3*veh.d_T3+B_dir_Trim5_4*veh.d_T4+B_dir_Trim5_5*veh.d_T5;
sum_forces_Trim6_6=B_dir_Trim6_1*veh.d_T1+B_dir_Trim6_2*veh.d_T2+B_dir_Trim6_3*veh.d_T3+B_dir_Trim6_4*veh.d_T4+B_dir_Trim6_5*veh.d_T5;
sum_forces_Trim7_6=B_dir_Trim7_1*veh.d_T1+B_dir_Trim7_2*veh.d_T2+B_dir_Trim7_3*veh.d_T3+B_dir_Trim7_4*veh.d_T4+B_dir_Trim7_5*veh.d_T5;

% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_Trim1_6;

tau_d_2_quasi=tau_d_2-sum_forces_Trim2_6;

tau_d_3_quasi=tau_d_3-sum_forces_Trim3_6;

tau_d_4_quasi=tau_d_4-sum_forces_Trim4_6;

tau_d_5_quasi=tau_d_5-sum_forces_Trim5_6;

tau_d_6_quasi=tau_d_6-sum_forces_Trim6_6;

tau_d_7_quasi=tau_d_7-sum_forces_Trim7_6;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T6(3)
    dual variables lam6
    minimize(norm(B_dir_Trim1_6*dir_T6-tau_d_1_quasi)+norm(B_dir_Trim2_6*dir_T6-tau_d_2_quasi)+norm(B_dir_Trim3_6*dir_T6-tau_d_3_quasi)+norm(B_dir_Trim4_6*dir_T6-tau_d_4_quasi)+...
    norm(B_dir_Trim5_6*dir_T6-tau_d_5_quasi)+norm(B_dir_Trim6_6*dir_T6-tau_d_6_quasi)+norm(B_dir_Trim7_6*dir_T6-tau_d_7_quasi))
    lam6: dir_T6'*dir_T6 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_Trim1_6*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_Trim2_6*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_Trim3_6*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_Trim4_6*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_Trim5_6*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_Trim6_6*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_Trim7_6*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T6;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T6=x_diropt;




