%% Linearization in the mathematical way,i.e.:

% subscript l denotes linearization

% 1. first we transfer the dynamic equation of underwater robotics in to standard state space form 

% (Mrb+Ma)*v_dot+(Crb(v)+Ca(v)+)*v+D(v)*v+g(eta)=tau 

% transform the dynamic equation into standard state space form

% dv/dt=
% -1/(Mrb+Ma)*(Crb(v)+Ca(v))-1/(Mrb+Ma)*D(v)-1/(Mrb+Ma)*g(eta)+1/(Mrb+Ma)*tau

% F=-1/(Mrb+Ma)*(Crb+Ca)*v-1/(Mrb+Ma)*D(v)-1/(Mrb+Ma)*g(eta)
% BH=1/(Mrb+Ma)*tau: for the generalised case 


% dynamic states

syms u_l v_l w_l p_l q_l r_l

% generalised forces

% syms X_l Y_l Z_l K_l M_l N_l

% W: total gravity for the underwarer robot
% B: total buoyancy for the underwater robot

% These two variables should be fixed during the optimization
% In the first optimization phase, B should be fixed because we have 

syms W B 

% Euler angles

syms  phi_l theta_l psi_l 

% position of center of mass

syms x_g y_g z_g

% position of center of buoyancy

syms x_b y_b z_b

%% total inertia matrix 
M=veh.Ma+veh.Mrb;

%% totoal Coriolis matrix: rigid body 
[CRB,CA]=tau_cor(veh,[u_l;v_l;w_l;p_l;q_l;r_l],[u_l;v_l;w_l;p_l;q_l;r_l]);

%% total drag
Dg=veh.D*[u_l*abs(u_l);v_l*abs(v_l);w_l*abs(w_l);p_l*abs(p_l);q_l*abs(q_l);r_l*abs(r_l)];

%% total restoring force
% The dynamics of underwater robotics is influenced by the kinematic
% variable  because of the restoring forece

% r_g is position of the center of mass in body frame {b}

r_g=[x_g y_g z_g];

% r_b is the position of the center of mass in 

r_b=[x_b y_b z_b];

% g is the restoring force part in the dynamic equation

g=gvect(W,B,theta_l,phi_l,r_g,r_b);

%% F_kin

F_lambda=-inv(M)*g;

% divide the kinematic part in dynamic equation into two parts (linear velocity part and angular velocity part)

F_v_lambda=F_lambda(1:3,:);

F_w_lambda=F_lambda(4:6,:);

%% F_dyn

F=-inv(M)*(CRB+CA+Dg);

% divide the dynamic part in dynamic equation into two parts (linear velocity part and angular velocity part) 

F_v=F(1:3,:);

F_w=F(4:6,:);

%% B_dyn

% this part should be changed for differnt configuration

% BH=inv(M)*[X_l;Y_l;Z_l;K_l;M_l;N_l];  

% consider the configuration with only thrusters as actuators

syms T1 T2 T3 T4 T5 T6

% B_T is the configuration matrix for thrusters

% intialize condition 
T_S_1={veh.r_T1,veh.b_T1,veh.d_T1};
T_S_2={veh.r_T2,veh.b_T2,veh.d_T2};
T_S_3={veh.r_T3,veh.b_T3,veh.d_T3};
T_S_4={veh.r_T4,veh.b_T4,veh.d_T4};
T_S_5={veh.r_T5,veh.b_T5,veh.d_T5};
T_S_6={veh.r_T6,veh.b_T6,veh.d_T6};


% compute the thruster configuration matrix 

B_T=ThrusterConfigurationMatrix(T_S_1,T_S_2,T_S_3,T_S_4,T_S_5,T_S_6);

BH=inv(M)*B_T*[T1;T2;T3;T4;T5;T6];

% separate the thruster configuration matrix into two parts

BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

%% perform linearization for the system 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);



% the input matrix for thrusters is not defined as symbolic function, since it has
% no parameter

B_v_l=jacobian(BH_v,[X_l Y_l Z_l K_l M_l N_l]);


% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);

% the input matrix for thrusters is not defined as symbolic function, since
% it has no parameter

B_w_l=jacobian(BH_w,[X_l Y_l Z_l K_l M_l N_l]);





