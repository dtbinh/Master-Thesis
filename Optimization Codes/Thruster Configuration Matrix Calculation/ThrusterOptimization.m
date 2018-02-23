% In this file we want to formulate the basic optimization problem, how to
% optimize locations of thrusters, 

% we have already an initial value of all geometric variables, first we
% optimize location of the thrusters, 
% 

% we use the Snookie configuration as our initial configuration for
% optimization

%%-------------------------------------------------------------------------------------
% Intial geometry variables:

% 1. number of thrusters 

% first assume that we have 6 thrusters, 

n_thrusters=6;

% 2. initial position and orientation for all thruster should be specified
% at first, spin direction should be optimized at the first step 
% 

%%-------------------------------------------------------------------------------------
% Preprocessing

% at the first step, we only determine the spin direction of all thrusters
% we have 2^n_thruster configurations

% we test with the second trim trajectory part

bestConditionNumber=inf;
spin_i=1;

for b_T1=-1:2:1
  for b_T2=-1:2:1
      for b_T3=-1:2:1
         for b_T4=-1:2:1
             for b_T5=-1:2:1
               for b_T6=-1:2:1
                   
 b_T_t=[b_T1;b_T2;b_T3;b_T3;b_T4;b_T5;b_T6];
 
b_T_tt{spin_i}=b_T_t;
T_S_1={veh.r_T1,b_T1,veh.d_T1};
T_S_2={veh.r_T2,b_T2,veh.d_T2};
T_S_3={veh.r_T3,b_T3,veh.d_T3};
T_S_4={veh.r_T4,b_T4,veh.d_T4};
T_S_5={veh.r_T5,b_T5,veh.d_T5};
T_S_6={veh.r_T6,b_T6,veh.d_T6};


% compute the thruster configuration matrix 

B_T=ThrusterConfigurationMatrix(T_S_1,T_S_2,T_S_3,T_S_4,T_S_5,T_S_6);

B_T_spin{spin_i}=B_T;

% redefine the symbolic variable

syms u_l v_l w_l p_l q_l r_l

% after the configuration is changed, we have a new input mapping matrix,  
% the term BH and all other terms involving BH should be updated in the
% script "LinearizationFormulas"

% we update  BH at first 

BH=inv(M)*B_T*[T1;T2;T3;T4;T5;T6];

BH_v=BH(1:3,:);
BH_w=BH(4:6,:);

% dynamic states

A_v_v_l=jacobian(F_v+BH_v,[u_l v_l w_l]);

A_v_w_l=jacobian(F_v+BH_v,[p_l q_l r_l]);

B_v_l=jacobian(BH_v,[T1 T2 T3 T4 T5 T6]);

% angular velocity part

A_w_v_l=jacobian(F_w+BH_w,[u_l v_l w_l]);

A_w_w_l=jacobian(F_w+BH_w,[p_l q_l r_l]);


B_w_l=jacobian(BH_w,[T1 T2 T3 T4 T5 T6]);
% after the configuration is fixed
%% we should compute the current error dynmaics

u_l=linear_velocity_x_2(1);
v_l=linear_velocity_y_2(1);
w_l=linear_velocity_z_2(1);

p_l=angular_velocity_roll_2(1);
q_l=angular_velocity_pitch_2(1);
r_l=angular_velocity_yaw_2(1);

% for the linearized error dynamics 

% The dynamic part is linearized mathematically using jacobian 

A_v_v_t=eval(A_v_v_l);

A_v_w_t=eval(A_v_w_l);

B_v_t=eval(B_v_l);

A_w_v_t=eval(A_w_v_l);

A_w_w_t=eval(A_w_w_l);

B_w_t=eval(B_w_l);

A_v_w_t=eval(A_v_w_l);

B_v_t=eval(B_v_l);

A_w_v_t=eval(A_w_v_l);

A_w_w_t=eval(A_w_w_l);

% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_t=[linear_velocity_x_2(1);linear_velocity_y_2(1);linear_velocity_z_2(1)];

omega_C_t=[angular_velocity_roll_2(1);angular_velocity_pitch_2(1);angular_velocity_yaw_2(1)];

% error dynamics for position
A_p_v_t=eye(3);
A_p_w_t=zeros(3);
A_p_p_t=-vp(omega_C_t);
A_p_lambda_t=-vp(nu_C_t);

% error dynamics for orientation
A_lambda_v_t=zeros(3);
A_lambda_w_t=eye(3);
A_lambda_p_t=zeros(3);
A_lambda_lambda_t=-vp(omega_C_t);

% error dynamics for linear velocities

A_v_p_t=zeros(3);
A_v_lambda_t=zeros(3);


% error dynamics for angular velocities

A_w_p_t=zeros(3);
A_w_lambda_t=zeros(3);

% formulate the error dynamics
A_E_t=[A_v_v_t A_v_w_t A_v_p_t A_v_lambda_t;A_w_v_t A_w_w_t A_w_p_t A_w_lambda_t;A_p_v_t A_p_w_t A_p_p_t A_p_lambda_t;A_lambda_v_t A_lambda_w_t A_lambda_p_t A_lambda_lambda_t];
B_E_t=[B_v_t;B_w_t;zeros(6)];

B_E_tt{spin_i}=B_E_t;
%% build the controllability matrix

Co_t=ctrb(A_E_t,B_E_t);

Co{spin_i}=cond(Co_t);


spin_i=spin_i+1;

               end
             end
         end
      end
  end
end

% chose the direction with the smallest condition number 
[b_T_min,min_index]=min(Co_t);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main loop for optimization

%% use the special trim trajectory segments 

n_trimtraj=7;
% we have seven trim trajecotories 

% for each trim trajectory segment, we have a linearised system
% thus, theorectically we should check the controllability matrix for each
% trim trajectory, then we have totally n_trimtraj*n_thrusters
% Controllability matices


% first let us specify the desired generalised control input

tau_d=tau_d_2(:,:,1);
u_d=pinv(B_T_spin{40})*tau_d;

% formulate the optimization problem

l_bound=[0;0;0;0;0;0];
u_bound=[20;20;20;20;20;20];

cvx_begin
variable x(6)
minimize(norm(B_T_spin{40}*x-tau_d))
subject to
l_bound<= x <= u_bound
cvx_end

%% formulate the problem as Bound-constrained least squares problem
AA=B_T_spin{40};
bb=tau_d;
x_qp = quadprog( 2*AA'*AA, -2*AA'*bb, [], [], [], [], l_bound, u_bound );
%% In the second phase, we optimize the position vector of the underwater robot 

%% In the third phase,
% formualte the optimization of the direction vector as sequential
% quadraric programming 