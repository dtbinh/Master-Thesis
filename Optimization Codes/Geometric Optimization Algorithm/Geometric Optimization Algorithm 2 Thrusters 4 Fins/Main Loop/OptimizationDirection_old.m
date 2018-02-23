%%  Yongyu Chen, formulate the optimization of the direction vector as convex optimization 

% in this file, we use QCQP to get the initial value of the direction optimization problem

% peform the initial guess for each thruster

% formulate the optimization of the direction vector as qcqp optimization
% problem, the optimization options are unchangable for all direction 
% optimization problems

% use sequential quadratic programming 

options_diropt_sqp = optimoptions('fmincon','Display','iter','Algorithm','sqp');

% read the desired input from the first optimization phase 

u_d_1=u_d(1);
u_d_2=u_d(2);
u_d_3=u_d(3);
u_d_4=u_d(4);
u_d_5=u_d(5);
u_d_6=u_d(6);


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


%% Thruster 1

% read the postion vector from the the global vector veh

r_T1=veh.r_T1;
r_T2=veh.r_T2;
r_T3=veh.r_T3;
r_T4=veh.r_T4;
r_T5=veh.r_T5;
r_T6=veh.r_T6;

% these parameters will be updated in the following sequential optimization

% calculate the current direction mapping matrix 

% the direction mapping matrix depends on geometric parameters from
% previous optimization phase, they will not change in this optimization
% phase

B_dir_1=DirObjFormulation(u_d_1,b_T1,lambda_T1,r_T1);

B_dir_2=DirObjFormulation(u_d_2,b_T2,lambda_T2,r_T2);

B_dir_3=DirObjFormulation(u_d_3,b_T3,lambda_T3,r_T3);

B_dir_4=DirObjFormulation(u_d_4,b_T4,lambda_T4,r_T4);

B_dir_5=DirObjFormulation(u_d_5,b_T5,lambda_T5,r_T5);

B_dir_6=DirObjFormulation(u_d_6,b_T6,lambda_T6,r_T6);




% when we optimize the first direction vector, the other direction vectors
% are kept constantly

% optimization of all thrusters in sequence 

%% Thruster 1


% sum of generalised for other thrusters: 

sum_forces_1=B_dir_2*veh.d_T2+B_dir_3*veh.d_T3+B_dir_4*veh.d_T4+B_dir_5*veh.d_T5+B_dir_6*veh.d_T6;


% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_1;

tau_d_2_quasi=tau_d_2-sum_forces_1;

tau_d_3_quasi=tau_d_3-sum_forces_1;

tau_d_4_quasi=tau_d_4-sum_forces_1;

tau_d_5_quasi=tau_d_5-sum_forces_1;

tau_d_6_quasi=tau_d_6-sum_forces_1;

tau_d_7_quasi=tau_d_7-sum_forces_1;

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
    
   minimize(norm(B_dir_1*dir_T1-tau_d_1_quasi)+norm(B_dir_1*dir_T1-tau_d_2_quasi)+norm(B_dir_1*dir_T1-tau_d_3_quasi)+norm(B_dir_1*dir_T1-tau_d_4_quasi)+...
    norm(B_dir_1*dir_T1-tau_d_5_quasi)+norm(B_dir_1*dir_T1-tau_d_6_quasi)+norm(B_dir_1*dir_T1-tau_d_7_quasi))
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


fun_diropt=@(x)norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_7_quasi);

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

sum_forces_2=B_dir_1*veh.d_T1+B_dir_3*veh.d_T3+B_dir_4*veh.d_T4+B_dir_5*veh.d_T5+B_dir_6*veh.d_T6;


% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_2;

tau_d_2_quasi=tau_d_2-sum_forces_2;

tau_d_3_quasi=tau_d_3-sum_forces_2;

tau_d_4_quasi=tau_d_4-sum_forces_2;

tau_d_5_quasi=tau_d_5-sum_forces_2;

tau_d_6_quasi=tau_d_6-sum_forces_2;

tau_d_7_quasi=tau_d_7-sum_forces_2;

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
    minimize(norm(B_dir_2*dir_T2-tau_d_1_quasi)+norm(B_dir_2*dir_T2-tau_d_2_quasi)+norm(B_dir_2*dir_T2-tau_d_3_quasi)+norm(B_dir_2*dir_T2-tau_d_4_quasi)+...
    norm(B_dir_2*dir_T2-tau_d_5_quasi)+norm(B_dir_2*dir_T2-tau_d_6_quasi)+norm(B_dir_2*dir_T2-tau_d_7_quasi))
    lam2: dir_T2'*dir_T2 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T2;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T2=x_diropt;

% 
%% thruster 3

% sum of generalised for other thrusters

sum_forces_3=B_dir_1*veh.d_T1+B_dir_2*veh.d_T2+B_dir_4*veh.d_T4+B_dir_5*veh.d_T5+B_dir_6*veh.d_T6;


% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_3;

tau_d_2_quasi=tau_d_2-sum_forces_3;

tau_d_3_quasi=tau_d_3-sum_forces_3;

tau_d_4_quasi=tau_d_4-sum_forces_3;

tau_d_5_quasi=tau_d_5-sum_forces_3;

tau_d_6_quasi=tau_d_6-sum_forces_3;

tau_d_7_quasi=tau_d_7-sum_forces_3;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T3(3)
    dual variables lam3
    minimize(norm(B_dir_1*dir_T3-tau_d_1_quasi)+norm(B_dir_2*dir_T3-tau_d_2_quasi)+norm(B_dir_3*dir_T3-tau_d_3_quasi)+norm(B_dir_4*dir_T3-tau_d_4_quasi)+...
    norm(B_dir_5*dir_T3-tau_d_5_quasi)+norm(B_dir_6*dir_T3-tau_d_6_quasi))
    lam3: dir_T3'*dir_T3 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_1*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_2*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_3*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_6_quasi);

x0_diropt = dir_T3;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T3=x_diropt;

%% thruster 4

% sum of generalised for other thrusters

sum_forces_4=B_dir_1*veh.d_T1+B_dir_2*veh.d_T2+B_dir_3*veh.d_T3+B_dir_5*veh.d_T5+B_dir_6*veh.d_T6;


% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_4;

tau_d_2_quasi=tau_d_2-sum_forces_4;

tau_d_3_quasi=tau_d_3-sum_forces_4;

tau_d_4_quasi=tau_d_4-sum_forces_4;

tau_d_5_quasi=tau_d_5-sum_forces_4;

tau_d_6_quasi=tau_d_6-sum_forces_4;

tau_d_7_quasi=tau_d_7-sum_forces_4;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T4(3)
    dual variables lam4
    minimize(norm(B_dir_4*dir_T4-tau_d_1_quasi)+norm(B_dir_4*dir_T3-tau_d_2_quasi)+norm(B_dir_4*dir_T3-tau_d_3_quasi)+norm(B_dir_4*dir_T3-tau_d_4_quasi)+...
    norm(B_dir_4*dir_T3-tau_d_5_quasi)+norm(B_dir_4*dir_T3-tau_d_6_quasi)+norm(B_dir_4*dir_T3-tau_d_7_quasi))
    lam4: dir_T4'*dir_T4 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_4*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T4;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T4=x_diropt;

%% thruster 5

% sum of generalised for other thrusters

sum_forces_5=B_dir_1*veh.d_T1+B_dir_2*veh.d_T2+B_dir_3*veh.d_T3+B_dir_4*veh.d_T4+B_dir_6*veh.d_T6;


% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_5;

tau_d_2_quasi=tau_d_2-sum_forces_5;

tau_d_3_quasi=tau_d_3-sum_forces_5;

tau_d_4_quasi=tau_d_4-sum_forces_5;

tau_d_5_quasi=tau_d_5-sum_forces_5;

tau_d_6_quasi=tau_d_6-sum_forces_5;

tau_d_7_quasi=tau_d_7-sum_forces_5;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T5(3)
    dual variables lam5
    minimize(norm(B_dir_5*dir_T5-tau_d_1_quasi)+norm(B_dir_5*dir_T5-tau_d_2_quasi)+norm(B_dir_5*dir_T5-tau_d_3_quasi)+norm(B_dir_5*dir_T5-tau_d_4_quasi)+...
    norm(B_dir_5*dir_T5-tau_d_5_quasi)+norm(B_dir_5*dir_T5-tau_d_6_quasi)+norm(B_dir_5*dir_T5-tau_d_7_quasi))
    lam5: dir_T5'*dir_T5 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_5*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T5;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T5=x_diropt;

%% thruster 6

% sum of generalised for other thrusters

sum_forces_6=B_dir_1*veh.d_T1+B_dir_2*veh.d_T2+B_dir_3*veh.d_T3+B_dir_4*veh.d_T4+B_dir_5*veh.d_T5;


% This result comes from the first optimization phase, we use the following
% desired input to build our mapping matrix

% note that we also need the desired input from all trim trajectories to
% build our optimization problem

% for our case we need tau_d_1, tau_d_2, ..., tau_d_7 minus the 

tau_d_1_quasi=tau_d_1-sum_forces_6;

tau_d_2_quasi=tau_d_2-sum_forces_6;

tau_d_3_quasi=tau_d_3-sum_forces_6;

tau_d_4_quasi=tau_d_4-sum_forces_6;

tau_d_5_quasi=tau_d_5-sum_forces_6;

tau_d_6_quasi=tau_d_6-sum_forces_6;

tau_d_7_quasi=tau_d_7-sum_forces_6;

% This result comes from the preprocessing 

% then formulate our problem into QCQP programming to get the initial value

% formulate the objective function

% formulate the constraint

cvx_begin
    variable dir_T6(3)
    dual variables lam6
    minimize(norm(B_dir_6*dir_T6-tau_d_1_quasi)+norm(B_dir_6*dir_T6-tau_d_2_quasi)+norm(B_dir_6*dir_T6-tau_d_3_quasi)+norm(B_dir_6*dir_T6-tau_d_4_quasi)+...
    norm(B_dir_6*dir_T6-tau_d_5_quasi)+norm(B_dir_6*dir_T6-tau_d_6_quasi)+norm(B_dir_6*dir_T6-tau_d_7_quasi))
    lam6: dir_T6'*dir_T6 - 1 <= 0;
cvx_end

% now we peform sequetial quadratic programming to determine the
% direction of the thruster

% note that the objective function does not change 

fun_diropt=@(x)norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_1_quasi)+norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_2_quasi)+norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_3_quasi)+norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_4_quasi)+...
    norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_5_quasi)+norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_6_quasi)+norm(B_dir_6*[x(1);x(2);x(3)]-tau_d_7_quasi);

x0_diropt = dir_T6;

[x_diropt,fval_diropt,exitflag,output] = fmincon(fun_diropt,x0_diropt,A_diropt,b_diropt,Aeq_diropt,beq_diropt,lb_diropt,ub_diropt,nonlcon_diropt,options_diropt_sqp);

% use the optimal direction vector

veh.d_T6=x_diropt;




