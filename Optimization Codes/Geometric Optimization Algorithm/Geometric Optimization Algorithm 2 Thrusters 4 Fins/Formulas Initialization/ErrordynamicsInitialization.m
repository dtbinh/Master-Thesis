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


% W: total gravity for the underwarer robot
% B: total buoyancy for the underwater robot

% These two variables should be fixed during the optimization
% In the first optimization phase, B should be fixed because in the first
% optimization phase we want realize the specification "Buoyancy Neutral"

% Thus the weight W should be equal to the Buoyancy B: W-B=0

% for the further development, we can make them different 

syms W B 

% Euler angles:

% for trim trajectory segment phi_l and theta_l are constant 

% psi_l is changing during the segment with constant yaw rate

syms  phi_l theta_l 

syms psi_l 

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

% two actuators and four fins as actuators

% thruster force

syms T1 T2 

% deflection angle
% 
syms delta1 delta2 delta3 delta4

% B_T is the configuration matrix for thrusters

% intialize geometry for thrusters

% two thrusters 

% initialize geometry for fins



% % stack the thruster parameters in to a cell
% 
% T_S_1={veh.r_T1,b_T1,veh.d_T1};
% T_S_2={veh.r_T2,b_T2,veh.d_T2};
% 
% 
% % compute the thruster configuration matrix for each spin direction
% % combination 
% 
% B_T=ThrusterConfigurationMatrix(T_S_1,T_S_2);
% 
% % four fins 
% 
% % the first parameter is the lift coefficient 
% % the second parameter denotes the size of the fins
% 
% % the third parameter is the surge velocity which is determined by the trim
% % trajectory segments 
% 
% % The fourth parameter denotes the position and orientation of robot fins
% % in the body frame {b}
% 
% 
% % Note that the fin configuration is dynamic state (six velocities) indepent
% 
% % according to our simplified model, the fin configuration matrix is only
% % surge velocity dependent, u depent
% 
% % the third parameter u should be updated 
% 
% 
% Fin1={C_L,[a_F1,b_F1],v_d_1(1),[x_F1,gama_F1]};
% 
% Fin2={C_L,[a_F2,b_F2],v_d_1(1),[x_F2,gama_F2]};
% 
% Fin3={C_L,[a_F3,b_F3],v_d_1(1),[x_F3,gama_F3]};
% 
% Fin4={C_L,[a_F4,b_F4],v_d_1(1),[x_F4,gama_F4]};
% 
% 
% % compute the fin configuration matrix 
% 
% B_F=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);
% 
% % concatenate the matrix 
% 
% B=horzcat(B_T,B_F);
% 
% 
% % compute the thruster configuration matrix 
% 
% B_T=ThrusterConfigurationMatrix(T_S_1,T_S_2);
% 
% % compute the fin configuration matrix 
% 
% B_F=FinConfigurationMatrix(Fin_1,Fin_2,Fin_3,Fin_4);
% 
% 
% % concatenate the matrix
% 
% B=horzcat(B_T,B_F);
% 
% BH=inv(M)*B*[T1;T2;delta1;delta2;delta3;delta4];
% 
% % separate the thruster configuration matrix into two parts
% 
% BH_v=BH(1:3,:);
% 
% BH_w=BH(4:6,:);
% 
% %% perform linearization for the system 
% 
% %------ linear velocity part ----------------------------------------------
% %------------------------------------------------------------------------------------------------------
% 
% % dynamic states 
% 
% A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);
% 
% A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);
% 
% 
% % kinematic states 
% 
% A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);
% 
% 
% 
% % the input matrix for thrusters is not defined as symbolic function, since it has
% % no parameter
% 
% % Linearize the input matrix 
% 
% B_v_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);
% 
% 
% % ---- angular velocity part----------------------------------------------
% % -------------------------------------------------------------------------------------------------------
% 
% % dynamic states 
% 
% A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);
% 
% A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);
% 
% 
% % kinematic states 
% 
% A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);
% 
% % the input matrix for thrusters is not defined as symbolic function, since
% % it has no parameter
% 
% B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);


%% define the global variable CtrlCheck to the controllability

CtrlCheck=1;

% The intial value for the index is 0: nont controllable

% must be controllable for all trim trajectory segments

CtrlCheck_1=1;

CtrlCheck_2=1;

CtrlCheck_3=1;

CtrlCheck_4=1;

CtrlCheck_5=1;

CtrlCheck_6=1;

CtrlCheck_7=1;







