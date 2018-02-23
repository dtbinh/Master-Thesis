%% Optimization for the position

%% first we read the position vector of all thrusters from the global variable veh


% in this optimization problem we need to optimize the position vector

r_T1=veh.r_T1;
r_T2=veh.r_T2;

%% for fins we also optimize the position x_F, the following four variables should be optimized

x_F1=veh.x_F1;
x_F2=veh.x_F2;
x_F3=veh.x_F3;
x_F4=veh.x_F4;

gamma_F1=veh.gamma_F1;
gamma_F2=veh.gamma_F2;
gamma_F3=veh.gamma_F3;
gamma_F4=veh.gamma_F4;

%%
r_H=d_H/2;

% transfer location of all thrusters into body frame intial value for the
% nonlinear programming

[x_Cyl_T1,phi_T1,rho_T1]=CoorTransferCy2Car(r_T1(1),r_T1(2),r_T1(3));
[x_Cyl_T2,phi_T2,rho_T2]=CoorTransferCy2Car(r_T2(1),r_T2(2),r_T2(3));



rho_T1_c{index_loop}=rho_T1;
rho_T2_c{index_loop}=rho_T2;


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


% we just need 

% sum of moment for each trim trajectory segment

sum_moment_1=veh.b_T1*veh.lambda1*u_d_1(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_1(2)*veh.d_T2;

sum_moment_2=veh.b_T1*veh.lambda1*u_d_2(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_2(2)*veh.d_T2;

sum_moment_3=veh.b_T1*veh.lambda1*u_d_3(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_3(2)*veh.d_T2;

sum_moment_4=veh.b_T1*veh.lambda1*u_d_4(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_4(2)*veh.d_T2;

sum_moment_5=veh.b_T1*veh.lambda1*u_d_5(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_5(2)*veh.d_T2;

sum_moment_6=veh.b_T1*veh.lambda1*u_d_6(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_6(2)*veh.d_T2;

sum_moment_7=veh.b_T1*veh.lambda1*u_d_7(1)*veh.d_T1+veh.b_T2*veh.lambda2*u_d_7(2)*veh.d_T2;

% Then we have quasi-desired moment for each trim trajectory

M_d_1_subs=M_d_1-sum_moment_1;
M_d_2_subs=M_d_2-sum_moment_2;
M_d_3_subs=M_d_3-sum_moment_3;
M_d_4_subs=M_d_4-sum_moment_4;
M_d_5_subs=M_d_5-sum_moment_5;
M_d_6_subs=M_d_6-sum_moment_6;
M_d_7_subs=M_d_7-sum_moment_7;

%% the force part

F_d_1=tau_d_1(1:3);
F_d_2=tau_d_2(1:3);
F_d_3=tau_d_3(1:3);
F_d_4=tau_d_4(1:3);
F_d_5=tau_d_5(1:3);
F_d_6=tau_d_6(1:3);
F_d_7=tau_d_7(1:3);

%% concatenate 
tau_d_1_quasi=vertcat(F_d_1,M_d_1_subs);
tau_d_2_quasi=vertcat(F_d_2,M_d_2_subs);
tau_d_3_quasi=vertcat(F_d_3,M_d_3_subs);
tau_d_4_quasi=vertcat(F_d_4,M_d_4_subs);
tau_d_5_quasi=vertcat(F_d_5,M_d_5_subs);
tau_d_6_quasi=vertcat(F_d_6,M_d_6_subs);


% stack all the desired input into a column vector

% M_d=[M_d_1_subs;M_d_2_subs;M_d_3_subs;M_d_4_subs;M_d_5_subs;M_d_6_subs;M_d_7_subs];


% note that the variable u_d with dimension of number of actuators for each
% trim trajectory

% the first trim trajectory

% % The following two matrices are 3 x 3 dimensional matrices for thrusters
%  
udx_1_1=-u_d_1(1)*vp(veh.d_T1);
udx_1_2=-u_d_1(2)*vp(veh.d_T2);
% 
% % The following four matrices are 3 X 3 dimensional matrices for four fins
% 
% % B_pos_fin_j_i
% 
% % the variable u_ind_j indicate the trim 
% 
% B_pos_fin_1_1=MomentCalculationFin(C_L,a_F1,b_F1,u_ind_1,x_F1,veh.gamma_F1,d_H);
% B_pos_fin_1_2=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_1,x_F2,veh.gamma_F2,d_H);
% B_pos_fin_1_3=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_1,x_F3,veh.gamma_F3,d_H);
% B_pos_fin_1_4=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_1,x_F4,veh.gamma_F4,d_H);
% 
% 
% 
% 
% 
% 
% % the second trim trajectory
% 
udx_2_1=-u_d_2(1)*vp(veh.d_T1);
udx_2_2=-u_d_2(2)*vp(veh.d_T2);
% 
% % for fins
% 
% B_pos_fin_2_1=MomentCalculationFin(C_L,a_F1,b_F1,u_ind_2,x_F1,veh.gamma_F1,d_H);
% B_pos_fin_2_2=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_2,x_F2,veh.gamma_F2,d_H);
% B_pos_fin_2_3=MomentCalculationFin(C_L,a_F3,b_F3,u_ind_2,x_F3,veh.gamma_F3,d_H);
% B_pos_fin_2_4=MomentCalculationFin(C_L,a_F4,b_F4,u_ind_2,x_F4,veh.gamma_F4,d_H);
% 
% % the third trim trajectory
% 
udx_3_1=-u_d_3(1)*vp(veh.d_T1);
udx_3_2=-u_d_3(2)*vp(veh.d_T2);
% 
% B_pos_fin_3_1=MomentCalculationFin(C_L,a_F1,b_F1,u_ind_3,x_F1,veh.gamma_F1,d_H);
% B_pos_fin_3_2=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_3,x_F2,veh.gamma_F2,d_H);
% B_pos_fin_3_3=MomentCalculationFin(C_L,a_F3,b_F2,u_ind_3,x_F3,veh.gamma_F3,d_H);
% B_pos_fin_3_4=MomentCalculationFin(C_L,a_F4,b_F2,u_ind_3,x_F4,veh.gamma_F4,d_H);
% 
% 
% % the fourth trim trajectory
% 
udx_4_1=-u_d_4(1)*vp(veh.d_T1);
udx_4_2=-u_d_4(2)*vp(veh.d_T2);
% 
% B_pos_fin_4_1=MomentCalculationFin(C_L,a_F1,b_F1,u_ind_4,x_F1,veh.gamma_F1,d_H);
% B_pos_fin_4_2=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_4,x_F2,veh.gamma_F2,d_H);
% B_pos_fin_4_3=MomentCalculationFin(C_L,a_F3,b_F3,u_ind_4,x_F3,veh.gamma_F3,d_H);
% B_pos_fin_4_4=MomentCalculationFin(C_L,a_F4,b_F4,u_ind_4,x_F4,veh.gamma_F4,d_H);
% 
% 
% 
% % the fifth trim trajectory
% 
udx_5_1=-u_d_5(1)*vp(veh.d_T1);
udx_5_2=-u_d_5(2)*vp(veh.d_T2);
% 
% B_pos_fin_5_1=MomentCalculationFin(C_L,a_F1,b_F1,u_ind_5,x_F1,veh.gamma_F1,d_H);
% B_pos_fin_5_2=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_5,x_F2,veh.gamma_F2,d_H);
% B_pos_fin_5_3=MomentCalculationFin(C_L,a_F3,b_F3,u_ind_5,x_F3,veh.gamma_F3,d_H);
% B_pos_fin_5_4=MomentCalculationFin(C_L,a_F4,b_F4,u_ind_5,x_F4,veh.gamma_F4,d_H);
% 
% 
% % the sixth trim trajectory
% 
udx_6_1=-u_d_6(1)*vp(veh.d_T1);
udx_6_2=-u_d_6(2)*vp(veh.d_T2);
% 
% B_pos_fin_6_1=MomentCalculationFin(C_L,a_F1,b_F1,u_ind_6,x_F1,veh.gamma_F1,d_H);
% B_pos_fin_6_2=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_6,x_F2,veh.gamma_F2,d_H);
% B_pos_fin_6_3=MomentCalculationFin(C_L,a_F3,b_F3,u_ind_6,x_F3,veh.gamma_F3,d_H);
% B_pos_fin_6_4=MomentCalculationFin(C_L,a_F4,b_F4,u_ind_6,x_F4,veh.gamma_F4,d_H);
% 
% 
% 
% % the seventh trim trajectory
% 
udx_7_1=-u_d_7(1)*vp(veh.d_T1);
udx_7_2=-u_d_7(2)*vp(veh.d_T2);
% 
% B_pos_fin_7_1=MomentCalculationFin(C_L,a_F1,b_F1,u_ind_7,x_F1,veh.gamma_F1,d_H);
% B_pos_fin_7_2=MomentCalculationFin(C_L,a_F2,b_F2,u_ind_7,x_F2,veh.gamma_F2,d_H);
% B_pos_fin_7_3=MomentCalculationFin(C_L,a_F3,b_F3,u_ind_7,x_F3,veh.gamma_F3,d_H);
% B_pos_fin_7_4=MomentCalculationFin(C_L,a_F4,b_F4,u_ind_7,x_F4,veh.gamma_F4,d_H);
% 
% 
% 
% 
% 
% B_pos=[udx_1_1,udx_1_2,B_pos_fin_1_1,B_pos_fin_1_2,B_pos_fin_1_3,B_pos_fin_1_4;
%        udx_2_1,udx_2_2,B_pos_fin_2_1,B_pos_fin_2_2,B_pos_fin_2_3,B_pos_fin_2_4;
%        udx_3_1,udx_3_2,B_pos_fin_3_1,B_pos_fin_3_2,B_pos_fin_3_3,B_pos_fin_3_4;
%        udx_4_1,udx_4_2,B_pos_fin_4_1,B_pos_fin_4_2,B_pos_fin_4_3,B_pos_fin_4_4;
%        udx_5_1,udx_5_2,B_pos_fin_5_1,B_pos_fin_5_2,B_pos_fin_5_3,B_pos_fin_5_4;
%        udx_6_1,udx_6_2,B_pos_fin_6_1,B_pos_fin_6_2,B_pos_fin_6_3,B_pos_fin_6_4;
%        udx_7_1,udx_7_2,B_pos_fin_7_1,B_pos_fin_7_2,B_pos_fin_7_3,B_pos_fin_7_4;];

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

% x_pos(5): x_F_1
% x_pos(6): gamma_F1

% x_pos(7): x_F_2
% x_pos(8): gamma_F2

% x_pos(9): x_F_3
% x_pos(10): gamma_F3

% x_pos(11): x_F_4
% x_pos(12): gamma_F4

options_posopt = optimoptions('fmincon','Display','iter','Algorithm','active-set');

fun=@(x_pos)norm(C_L*q(u_ind_1)*[0;
                                 a_F1*b_F1*cos(x_pos(6))*u_d_1(3)+a_F2*b_F2*cos(x_pos(8))*u_d_1(4)+a_F3*b_F3*cos(x_pos(10))*u_d_1(5)+a_F4*b_F4*cos(x_pos(12))*u_d_1(6);
                                 a_F1*b_F1*sin(x_pos(6))*u_d_1(3)+a_F2*b_F2*sin(x_pos(8))*u_d_1(4)+a_F3*b_F3*sin(x_pos(10))*u_d_1(5)+a_F4*b_F4*sin(x_pos(12))*u_d_1(6);
                                 -0.5*(a_F1*b_F1*(d_H+a_F1)*u_d_1(3)+a_F2*b_F2*(d_H+a_F2)*u_d_1(4)+a_F3*b_F3*(d_H+a_F3)*u_d_1(5)+a_F4*b_F4*(d_H+a_F4)*u_d_1(6));
                                 a_F1*b_F1*x_pos(5)*sin(x_pos(6))*u_d_1(3)+a_F2*b_F2*x_pos(7)*sin(x_pos(8))*u_d_1(4)+a_F3*b_F3*x_pos(9)*sin(x_pos(10))*u_d_1(5)+a_F4*b_F4*x_pos(11)*sin(x_pos(12))*u_d_1(6);
                                 a_F1*b_F1*x_pos(5)*cos(x_pos(6))*u_d_1(3)+a_F2*b_F2*x_pos(7)*cos(x_pos(8))*u_d_1(4)+a_F3*b_F3*x_pos(9)*cos(x_pos(10))*u_d_1(5)+a_F4*b_F4*x_pos(11)*cos(x_pos(12))*u_d_1(6)]+...
                                 vertcat([0;0;0],udx_1_1*[x_pos(1);rho_T1*cos(x_pos(2));rho_T1*sin(x_pos(2))]+udx_1_2*[x_pos(3);rho_T1*cos(x_pos(4));rho_T1*sin(x_pos(4))])-tau_d_1_quasi)+...
                norm(C_L*q(u_ind_2)*[0;
                                 a_F1*b_F1*cos(x_pos(6))*u_d_2(3)+a_F2*b_F2*cos(x_pos(8))*u_d_2(4)+a_F3*b_F3*cos(x_pos(10))*u_d_2(5)+a_F4*b_F4*cos(x_pos(12))*u_d_2(6);
                                 a_F1*b_F1*sin(x_pos(6))*u_d_2(3)+a_F2*b_F2*sin(x_pos(8))*u_d_2(4)+a_F3*b_F3*sin(x_pos(10))*u_d_2(5)+a_F4*b_F4*sin(x_pos(12))*u_d_2(6);
                                 -0.5*(a_F1*b_F1*(d_H+a_F1)*u_d_2(3)+a_F2*b_F2*(d_H+a_F2)*u_d_2(4)+a_F3*b_F3*(d_H+a_F3)*u_d_2(5)+a_F4*b_F4*(d_H+a_F4)*u_d_2(6));
                                 a_F1*b_F1*x_pos(5)*sin(x_pos(6))*u_d_2(3)+a_F2*b_F2*x_pos(7)*sin(x_pos(8))*u_d_2(4)+a_F3*b_F3*x_pos(9)*sin(x_pos(10))*u_d_2(5)+a_F4*b_F4*x_pos(11)*sin(x_pos(12))*u_d_2(6);
                                 a_F1*b_F1*x_pos(5)*cos(x_pos(6))*u_d_2(3)+a_F2*b_F2*x_pos(7)*cos(x_pos(8))*u_d_2(4)+a_F3*b_F3*x_pos(9)*cos(x_pos(10))*u_d_2(5)+a_F4*b_F4*x_pos(11)*cos(x_pos(12))*u_d_2(6)]+...
                                 vertcat([0;0;0],udx_2_1*[x_pos(1);rho_T1*cos(x_pos(2));rho_T1*sin(x_pos(2))]+udx_2_2*[x_pos(3);rho_T1*cos(x_pos(4));rho_T1*sin(x_pos(4))])-tau_d_1_quasi);



% for the first iteration, the initial value should be generated randomly

% for the following iteration we should use 

 x0_pos=[x_Cyl_T1,phi_T1,x_Cyl_T2,phi_T2,x_F1,gamma_F1,x_F2,gamma_F2,x_F3,gamma_F3,x_F4,gamma_F4];

% x0_pos=[x_T1_init_Cyl,phi_T1_init,x_T2_init_Cyl,phi_T2_init,x_T3_init_Cyl,phi_T3_init,x_T4_init_Cyl,phi_T4_init,x_T5_init_Cyl,phi_T5_init,x_T6_init_Cyl,phi_T6_init];

A_pos=[];
b_pos=[];

Aeq_pos=[];
beq_pos=[];

lb_pos=[-l_H/2,-pi,-l_H/2,-pi,-(l_H-b_F1)/2,-pi,-(l_H-b_F2)/2,-pi,-(l_H-b_F3)/2,-pi,-(l_H-b_F4)/2,-pi];
ub_pos=[l_H/2,pi,l_H/2,pi,(l_H-b_F1)/2,pi,(l_H-b_F2)/2,pi,(l_H-b_F3)/2,pi,(l_H-b_F4)/2,pi];

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

% for fins

x_F1=x_pos(5);

gamma_F1=x_pos(6);

x_F2=x_pos(7);

gamma_F2=x_pos(8);

x_F3=x_pos(9);

gamma_F3=x_pos(10);

x_F4=x_pos(11);

gamma_F4=x_pos(12);







% transfer the optimal value in cylindrical coordinates to Cartesian
% coordinates

[r_T1(1),r_T1(2),r_T1(3)]=CoorTransferCar2Cy(x_Cyl_T1,phi_T1,d_H/2);
[r_T2(1),r_T2(2),r_T2(3)]=CoorTransferCar2Cy(x_Cyl_T2,phi_T2,d_H/2);


% assign back the value into the global value for the next iterations

% in this optimization phase we we only opti

veh.r_T1=r_T1;
veh.r_T2=r_T2;

veh.x_F1=x_F1;
veh.gamma_F1=gamma_F1;

veh.x_F2=x_F2;
veh.gamma_F2=gamma_F2;

veh.x_F3=gamma_F3;
veh.gamma_F3=gamma_F3;

veh.x_F4=x_F4;
veh.gamma_F4=gamma_F4;






