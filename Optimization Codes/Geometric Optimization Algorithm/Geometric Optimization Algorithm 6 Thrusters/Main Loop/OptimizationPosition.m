%% Optimization for the position

%% first we read the position vector of all thrusters from the global variable veh


% in this optimization problem we need to optimize the position vector

r_T1=veh.r_T1;
r_T2=veh.r_T2;
r_T3=veh.r_T3;
r_T4=veh.r_T4;
r_T5=veh.r_T5;
r_T6=veh.r_T6;

r_H=d_H/2;

% transfer location of all thrusters into body frame intial value for the
% nonlinear programming

[x_Cyl_T1,phi_T1,rho_T1]=CoorTransferCy2Car(r_T1(1),r_T1(2),r_T1(3));
[x_Cyl_T2,phi_T2,rho_T2]=CoorTransferCy2Car(r_T2(1),r_T2(2),r_T2(3));
[x_Cyl_T3,phi_T3,rho_T3]=CoorTransferCy2Car(r_T3(1),r_T3(2),r_T3(3));
[x_Cyl_T4,phi_T4,rho_T4]=CoorTransferCy2Car(r_T4(1),r_T4(2),r_T4(3));
[x_Cyl_T5,phi_T5,rho_T5]=CoorTransferCy2Car(r_T5(1),r_T5(2),r_T5(3));
[x_Cyl_T6,phi_T6,rho_T6]=CoorTransferCy2Car(r_T6(1),r_T6(2),r_T6(3));


rho_T1_c{index_loop}=rho_T1;
rho_T2_c{index_loop}=rho_T2;
rho_T3_c{index_loop}=rho_T3;
rho_T4_c{index_loop}=rho_T4;
rho_T5_c{index_loop}=rho_T5;
rho_T6_c{index_loop}=rho_T6;

% lower bound and upper bound are determined by the hull size 

% formulate the optimization problem as convex optimization problem

% actually the position of the thrusters only affect the moment

% thus, firstly we extract the moment part from the desired input 

M_d_1=tau_d_1(4:6);
M_d_2=tau_d_2(4:6);
M_d_3=tau_d_3(4:6);
M_d_4=tau_d_4(4:6);
M_d_5=tau_d_5(4:6);
M_d_6=tau_d_6(4:6);
M_d_7=tau_d_7(4:6);


% sum of moment comes from the result of the last step 

% sum of moment for each trim trajectory segment

sum_moment_1=veh.b_T1*veh.lambda1*u_d_1(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_1(2)*veh.d_T2+veh.b_T3*veh.lambda3*u_d_1(3)*veh.d_T3+...
    veh.b_T4*veh.lambda4*u_d_1(4)*veh.d_T4+veh.b_T5*veh.lambda5*u_d_1(5)*veh.d_T5+veh.b_T6*veh.lambda6*u_d_1(6)*veh.d_T6;

sum_moment_2=veh.b_T1*veh.lambda1*u_d_2(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_2(2)*veh.d_T2+veh.b_T3*veh.lambda3*u_d_2(3)*veh.d_T3+...
    veh.b_T4*veh.lambda4*u_d_2(4)*veh.d_T4+veh.b_T5*veh.lambda5*u_d_2(5)*veh.d_T5+veh.b_T6*veh.lambda6*u_d_2(6)*veh.d_T6;

sum_moment_3=veh.b_T1*veh.lambda1*u_d_3(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_3(2)*veh.d_T2+veh.b_T3*veh.lambda3*u_d_3(3)*veh.d_T3+...
    veh.b_T4*veh.lambda4*u_d_3(4)*veh.d_T4+veh.b_T5*veh.lambda5*u_d_3(5)*veh.d_T5+veh.b_T6*veh.lambda6*u_d_3(6)*veh.d_T6;

sum_moment_4=veh.b_T1*veh.lambda1*u_d_4(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_4(2)*veh.d_T2+veh.b_T3*veh.lambda3*u_d_4(3)*veh.d_T3+...
    veh.b_T4*veh.lambda4*u_d_4(4)*veh.d_T4+veh.b_T5*veh.lambda5*u_d_4(5)*veh.d_T5+veh.b_T6*veh.lambda6*u_d_4(6)*veh.d_T6;

sum_moment_5=veh.b_T1*veh.lambda1*u_d_5(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_5(2)*veh.d_T2+veh.b_T3*veh.lambda3*u_d_5(3)*veh.d_T3+...
    veh.b_T4*veh.lambda4*u_d_5(4)*veh.d_T4+veh.b_T5*veh.lambda5*u_d_5(5)*veh.d_T5+veh.b_T6*veh.lambda6*u_d_5(6)*veh.d_T6;

sum_moment_6=veh.b_T1*veh.lambda1*u_d_6(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_6(2)*veh.d_T2+veh.b_T3*veh.lambda3*u_d_6(3)*veh.d_T3+...
    veh.b_T4*veh.lambda4*u_d_6(4)*veh.d_T4+veh.b_T5*veh.lambda5*u_d_6(5)*veh.d_T5+veh.b_T6*veh.lambda6*u_d_6(6)*veh.d_T6;

sum_moment_7=veh.b_T1*veh.lambda1*u_d_7(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_7(2)*veh.d_T2+veh.b_T3*veh.lambda3*u_d_7(3)*veh.d_T3+...
    veh.b_T4*veh.lambda4*u_d_7(4)*veh.d_T4+veh.b_T5*veh.lambda5*u_d_7(5)*veh.d_T5+veh.b_T6*veh.lambda6*u_d_7(6)*veh.d_T6;

% Then we have quasi-desired moment for each trim trajectory

M_d_1_subs=M_d_1-sum_moment_1;
M_d_2_subs=M_d_2-sum_moment_2;
M_d_3_subs=M_d_3-sum_moment_3;
M_d_4_subs=M_d_4-sum_moment_4;
M_d_5_subs=M_d_5-sum_moment_5;
M_d_6_subs=M_d_6-sum_moment_6;
M_d_7_subs=M_d_7-sum_moment_7;

% stack all the desired input into a column vector

M_d=[M_d_1_subs;M_d_2_subs;M_d_3_subs;M_d_4_subs;M_d_5_subs;M_d_6_subs;M_d_7_subs];


% note that the variable u_d with dimension of number of actuators for each
% trim trajectory

% the first trim trajectory

udx_1_1=-u_d_1(1)*vp(veh.d_T1);
udx_1_2=-u_d_1(2)*vp(veh.d_T2);
udx_1_3=-u_d_1(3)*vp(veh.d_T3);
udx_1_4=-u_d_1(4)*vp(veh.d_T4);
udx_1_5=-u_d_1(5)*vp(veh.d_T5);
udx_1_6=-u_d_1(6)*vp(veh.d_T6);

% the second trim trajectory

udx_2_1=-u_d_2(1)*vp(veh.d_T1);
udx_2_2=-u_d_2(2)*vp(veh.d_T2);
udx_2_3=-u_d_2(3)*vp(veh.d_T3);
udx_2_4=-u_d_2(4)*vp(veh.d_T4);
udx_2_5=-u_d_2(5)*vp(veh.d_T5);
udx_2_6=-u_d_2(6)*vp(veh.d_T6);

% the third trim trajectory

udx_3_1=-u_d_3(1)*vp(veh.d_T1);
udx_3_2=-u_d_3(2)*vp(veh.d_T2);
udx_3_3=-u_d_3(3)*vp(veh.d_T3);
udx_3_4=-u_d_3(4)*vp(veh.d_T4);
udx_3_5=-u_d_3(5)*vp(veh.d_T5);
udx_3_6=-u_d_3(6)*vp(veh.d_T6);

% the fourth trim trajectory

udx_4_1=-u_d_4(1)*vp(veh.d_T1);
udx_4_2=-u_d_4(2)*vp(veh.d_T2);
udx_4_3=-u_d_4(3)*vp(veh.d_T3);
udx_4_4=-u_d_4(4)*vp(veh.d_T4);
udx_4_5=-u_d_4(5)*vp(veh.d_T5);
udx_4_6=-u_d_4(6)*vp(veh.d_T6);

% the fifth trim trajectory

udx_5_1=-u_d_5(1)*vp(veh.d_T1);
udx_5_2=-u_d_5(2)*vp(veh.d_T2);
udx_5_3=-u_d_5(3)*vp(veh.d_T3);
udx_5_4=-u_d_5(4)*vp(veh.d_T4);
udx_5_5=-u_d_5(5)*vp(veh.d_T5);
udx_5_6=-u_d_5(6)*vp(veh.d_T6);

% the sixth trim trajectory

udx_6_1=-u_d_6(1)*vp(veh.d_T1);
udx_6_2=-u_d_6(2)*vp(veh.d_T2);
udx_6_3=-u_d_6(3)*vp(veh.d_T3);
udx_6_4=-u_d_6(4)*vp(veh.d_T4);
udx_6_5=-u_d_6(5)*vp(veh.d_T5);
udx_6_6=-u_d_6(6)*vp(veh.d_T6);

% the seventh trim trajectory

udx_7_1=-u_d_7(1)*vp(veh.d_T1);
udx_7_2=-u_d_7(2)*vp(veh.d_T2);
udx_7_3=-u_d_7(3)*vp(veh.d_T3);
udx_7_4=-u_d_7(4)*vp(veh.d_T4);
udx_7_5=-u_d_7(5)*vp(veh.d_T5);
udx_7_6=-u_d_7(6)*vp(veh.d_T6);



B_pos=[udx_1_1,udx_1_2,udx_1_3,udx_1_4,udx_1_5,udx_1_6;udx_2_1,udx_2_2,udx_2_3,udx_2_4,udx_2_5,udx_2_6;udx_3_1,udx_3_2,udx_3_3,udx_3_4,udx_3_5,udx_3_6;udx_4_1,udx_4_2,udx_4_3,udx_4_4,udx_4_5,udx_4_6;udx_5_1,udx_5_2,udx_5_3,udx_5_4,udx_5_5,udx_5_6;udx_6_1,udx_6_2,udx_6_3,udx_6_4,udx_6_5,udx_6_6;udx_7_1,udx_7_2,udx_7_3,udx_7_4,udx_7_5,udx_7_6];

% lower bound and upper bound 
  
%% formulate the problem as SQP programming

% since we have six thrusters, we have 12 decision vectors

% The initial guess of the positions are generated by the 


% The following two hull geometric parameters come from the first
% optimization phase

% the hull radius and length should be read from the global variable veh.

% specification of decsion variables:

% x_pos(1): x_Cyl_T1
% x_pos(2): phi_T1

% x_pos(3): x_Cyl_T2
% x_pos(4): phi_T2

% x_pos(5): x_Cyl_T3
% x_pos(6): phi_T3

% x_pos(7): x_Cyl_T4
% x_pos(8): phi_T4

% x_pos(9): x_Cyl_T5
% x_pos(10): phi_T5

% x_pos(11): x_Cyl_T6
% x_pos(12): phi_T6

options_posopt = optimoptions('fmincon','Display','iter','Algorithm','active-set');

fun=@(x_pos)norm(B_pos*[x_pos(1);rho_T1*cos(x_pos(2));rho_T1*sin(x_pos(2));x_pos(3);rho_T2*cos(x_pos(4));rho_T2*sin(x_pos(4));x_pos(5);rho_T3*cos(x_pos(6));rho_T3*sin(x_pos(6));x_pos(7);rho_T4*cos(x_pos(8));rho_T4*sin(x_pos(8));x_pos(9);rho_T5*cos(x_pos(10));rho_T5*sin(x_pos(10));x_pos(11);rho_T6*cos(x_pos(12));rho_T6*sin(x_pos(12))]-M_d);

% for the first iteration, the initial value should be generated randomly

% for the following iteration we should use 

 x0_pos=[x_Cyl_T1,phi_T1,x_Cyl_T2,phi_T2,x_Cyl_T3,phi_T3,x_Cyl_T4,phi_T4,x_Cyl_T5,phi_T5,x_Cyl_T6,phi_T6];

% x0_pos=[x_T1_init_Cyl,phi_T1_init,x_T2_init_Cyl,phi_T2_init,x_T3_init_Cyl,phi_T3_init,x_T4_init_Cyl,phi_T4_init,x_T5_init_Cyl,phi_T5_init,x_T6_init_Cyl,phi_T6_init];

A_pos=[];
b_pos=[];

Aeq_pos=[];
beq_pos=[];

lb_pos=[-l_H/2,-pi,-l_H/2,-pi,-l_H/2,-pi,-l_H/2,-pi,-l_H/2,-pi,-l_H/2,-pi];
ub_pos=[l_H/2,pi,l_H/2,pi,l_H/2,pi,l_H/2,pi,l_H/2,pi,l_H/2,pi];

[x_pos,fval_pos,exitflg,output]=fmincon(fun,x0_pos,A_pos,b_pos,Aeq_pos,beq_pos,lb_pos,ub_pos,[],options_posopt);

fval_pos_c{index_loop}=fval_pos;

%% The optimal value of the abovementioned optimization should be given back

% The decision variables are all represented in cylindrical frame we should
% transfer them into Cartesian Coordinate first

% read the result from the optimization result

x_Cyl_T1=x_pos(1);
phi_T1=x_pos(2);
x_Cyl_T2=x_pos(3);
phi_T2=x_pos(4);
x_Cyl_T3=x_pos(5);
phi_T3=x_pos(6);
x_Cyl_T4=x_pos(7);
phi_T4=x_pos(8);
x_Cyl_T5=x_pos(9);
phi_T5=x_pos(10);
x_Cyl_T6=x_pos(11);
phi_T6=x_pos(12);


% transfer the optimal value in cylindrical coordinates to Cartesian
% coordinates

[r_T1(1),r_T1(2),r_T1(3)]=CoorTransferCar2Cy(x_Cyl_T1,phi_T1,d_H/2);
[r_T2(1),r_T2(2),r_T2(3)]=CoorTransferCar2Cy(x_Cyl_T2,phi_T2,d_H/2);
[r_T3(1),r_T3(2),r_T3(3)]=CoorTransferCar2Cy(x_Cyl_T3,phi_T3,d_H/2);
[r_T4(1),r_T4(2),r_T4(3)]=CoorTransferCar2Cy(x_Cyl_T4,phi_T4,d_H/2);
[r_T5(1),r_T5(2),r_T5(3)]=CoorTransferCar2Cy(x_Cyl_T5,phi_T5,d_H/2);
[r_T6(1),r_T6(2),r_T6(3)]=CoorTransferCar2Cy(x_Cyl_T6,phi_T6,d_H/2);

% assign back the value into the global value for the next iterations

veh.r_T1=r_T1;
veh.r_T2=r_T2;
veh.r_T3=r_T3;
veh.r_T4=r_T4;
veh.r_T5=r_T5;
veh.r_T6=r_T6;






