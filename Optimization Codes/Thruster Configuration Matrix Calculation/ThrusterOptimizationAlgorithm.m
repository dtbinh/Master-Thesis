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

%%------------------------------------------------------------------------------------------------------------------------------------------------
% Preprocessing 

% at the first step, we only determine the spin direction of all thrusters
% we have 2^n_thruster configurations

% we want to test all trim trajectory segments 

% here we have seven trim trajectory segments 


%% for all trim trajectories 


% we want to find the best spin direction group,i.e, the spin direction
% group with the smallest condition number 
% but we should iterate within all trim trajectory segments 

bestConditionNumber=inf;

% index for number of different spin directions
spin_i=1;

for b_T1=-1:2:1
  for b_T2=-1:2:1
      for b_T3=-1:2:1
         for b_T4=-1:2:1
             for b_T5=-1:2:1
               for b_T6=-1:2:1
                   
% stack spin directions of all thrusters into one vector 

b_T=[b_T1;b_T2;b_T3;b_T3;b_T4;b_T5;b_T6];

% put all different spin direction configuration into a a cell

b_T_g{spin_i}=b_T;

% put the current spin direction of all thrusters into the mapping matrix
% for each thruster 

T_S_1={veh.r_T1,b_T1,veh.d_T1};
T_S_2={veh.r_T2,b_T2,veh.d_T2};
T_S_3={veh.r_T3,b_T3,veh.d_T3};
T_S_4={veh.r_T4,b_T4,veh.d_T4};
T_S_5={veh.r_T5,b_T5,veh.d_T5};
T_S_6={veh.r_T6,b_T6,veh.d_T6};


% compute the thruster configuration matrix for each spin direction
% combination 

B_T=ThrusterConfigurationMatrix(T_S_1,T_S_2,T_S_3,T_S_4,T_S_5,T_S_6);

% put all thruster configuration matrices with different spin directions 

B_T_spin{spin_i}=B_T;


% after the configuration is changed, we have a new input mapping matrix,  
% the term BH and all other terms involving BH should be updated in the
% script "LinearizationFormulas"

% we update BH at first 

BH=inv(M)*B_T*[T1;T2;T3;T4;T5;T6];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 T3 T4 T5 T6]);


B_w_l=jacobian(BH_w,[T1 T2 T3 T4 T5 T6]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);


% B_v_l=jacobian(BH_v,[X_l Y_l Z_l K_l M_l N_l]);


% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);


% B_w_l=jacobian(BH_w,[X_l Y_l Z_l K_l M_l N_l]);


% after the configuration is fixed
% we should compute the current error dynmaics for each trim trajectory
% segments 

% u_l, v_l, w_l, p_l, q_l, r_l are already specified in the file 
% they are calculated in the initialization file
% "ThrusterConfigurationOptiInit.m"

% for the linearized error dynamics 

% The dynamic part is linearized mathematically using jacobian 

% first trim trajectory

A_v_v_1=A_v_v_l(u_l_1,v_l_1,w_l_1,p_l_1,q_l_1,r_l_1);

A_v_w_1=A_v_w_l(u_l_1,v_l_1,w_l_1,p_l_1,q_l_1,r_l_1);



A_w_v_1=A_w_v_l(u_l_1,v_l_1,w_l_1,p_l_1,q_l_1,r_l_1);

A_w_w_1=A_w_w_l(u_l_1,v_l_1,w_l_1,p_l_1,q_l_1,r_l_1);



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_1=[u_l_1;v_l_1;w_l_1];

omega_C_1=[p_l_1;q_l_1;r_l_1];

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

% formulate the error dynamics

A_E_1=[A_v_v_1 A_v_w_1 A_v_p_1 A_v_lambda_1;A_w_v_1 A_w_w_1 A_w_p_1 A_w_lambda_1;A_p_v_1 A_p_w_1 A_p_p_1 A_p_lambda_1;A_lambda_v_1 A_lambda_w_1 A_lambda_p_1 A_lambda_lambda_1];

B_E_1=[B_v_1;B_w_1;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_1{spin_i}=B_E_1;

% build the controllability matrix

Co_1=ctrb(A_E_1,B_E_1);

% Co_c: Controllability matrix cluster

Co_c_1{spin_i}=cond(Co_1);


% second trim trajectory

A_v_v_2=A_v_v_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);

A_v_w_2=A_v_w_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);



A_w_v_2=A_w_v_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);

A_w_w_2=A_w_w_l(u_l_2,v_l_2,w_l_2,p_l_2,q_l_2,r_l_2);



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_2=[u_l_1;v_l_1;w_l_1];

omega_C_2=[p_l_1;q_l_1;r_l_1];

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

% formulate the error dynamics

A_E_2=[A_v_v_2 A_v_w_2 A_v_p_2 A_v_lambda_2;A_w_v_2 A_w_w_2 A_w_p_2 A_w_lambda_2;A_p_v_2 A_p_w_2 A_p_p_2 A_p_lambda_2;A_lambda_v_2 A_lambda_w_2 A_lambda_p_2 A_lambda_lambda_2];

B_E_2=[B_v_2;B_w_2;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_2{spin_i}=B_E_2;

% build the controllability matrix

Co_2=ctrb(A_E_2,B_E_2);

% Co_c: Controllability matrix cluster

Co_c_2{spin_i}=cond(Co_2);


% third trim trajectory

A_v_v_3=A_v_v_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);


A_v_w_3=A_v_w_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);



A_w_v_3=A_w_v_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);


A_w_w_3=A_w_w_l(u_l_3,v_l_3,w_l_3,p_l_3,q_l_3,r_l_3);



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_3=[u_l_3;v_l_3;w_l_3];

omega_C_3=[p_l_3;q_l_3;r_l_3];

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

% formulate the error dynamics

A_E_3=[A_v_v_3 A_v_w_3 A_v_p_3 A_v_lambda_3;A_w_v_3 A_w_w_3 A_w_p_3 A_w_lambda_3;A_p_v_3 A_p_w_3 A_p_p_3 A_p_lambda_3;A_lambda_v_3 A_lambda_w_3 A_lambda_p_3 A_lambda_lambda_3];

B_E_3=[B_v_3;B_w_3;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_3{spin_i}=B_E_3;

% build the controllability matrix

Co_3=ctrb(A_E_3,B_E_3);

% Co_c: Controllability matrix cluster

Co_c_3{spin_i}=cond(Co_3);

% fourth trim trajectory

A_v_v_4=A_v_v_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);

A_v_w_4=A_v_w_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);



A_w_v_4=A_w_v_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);

A_w_w_4=A_w_w_l(u_l_4,v_l_4,w_l_4,p_l_4,q_l_4,r_l_4);



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_4=[u_l_4;v_l_4;w_l_4];

omega_C_4=[p_l_4;q_l_4;r_l_4];

% error dynamics for position

A_p_v_4=eye(3);

A_p_w_4=zeros(3);

A_p_p_4=-vp(omega_C_4);

A_p_lambda_4=-vp(nu_C_4);

% error dynamics for orientation

A_lambda_v_4=zeros(3);

A_lambda_p_4=zeros(3);

A_lambda_lambda_3=-vp(omega_C_4);

% error dynamics for linear velocities

A_v_p_4=zeros(3);

A_v_lambda_4=zeros(3);


% error dynamics for angular velocities

A_w_p_4=zeros(3);

A_w_lambda_4=zeros(3);

% formulate the error dynamics

A_E_4=[A_v_v_4 A_v_w_4 A_v_p_4 A_v_lambda_4;A_w_v_4 A_w_w_4 A_w_p_4 A_w_lambda_4;A_p_v_4 A_p_w_4 A_p_p_4 A_p_lambda_4;A_lambda_v_4 A_lambda_w_4 A_lambda_p_4 A_lambda_lambda_4];

B_E_4=[B_v_4;B_w_4;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_4{spin_i}=B_E_4;

% build the controllability matrix

Co_4=ctrb(A_E_4,B_E_4);

% Co_c: Controllability matrix cluster

Co_c_4{spin_i}=cond(Co_4);

% fifth trim trajectory

A_v_v_5=A_v_v_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);

A_v_w_5=A_v_w_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);


A_w_v_5=A_w_v_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);

A_w_w_5=A_w_w_l(u_l_5,v_l_5,w_l_5,p_l_5,q_l_5,r_l_5);


% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_5=[u_l_5;v_l_5;w_l_5];

omega_C_5=[p_l_5;q_l_5;r_l_5];

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

B_E_5=[B_v_5;B_w_5;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_5{spin_i}=B_E_5;

% build the controllability matrix

Co_5=ctrb(A_E_5,B_E_5);

% Co_c: Controllability matrix cluster

Co_c_5{spin_i}=cond(Co_5);

% sixth trim trajectory

A_v_v_6=A_v_v_l(u_l_6,v_l_6,w_l_6,p_l_6,q_l_6,r_l_6);

A_v_w_6=A_v_w_l(u_l_6,v_l_6,w_l_6,p_l_6,q_l_6,r_l_6);



A_w_v_6=A_w_v_l(u_l_6,v_l_6,w_l_6,p_l_6,q_l_6,r_l_6);

A_w_w_6=A_w_w_l(u_l_6,v_l_6,w_l_6,p_l_6,q_l_6,r_l_6);



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_6=[u_l_6;v_l_6;w_l_6];

omega_C_6=[p_l_6;q_l_6;r_l_6];

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

B_E_6=[B_v_6;B_w_6;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_6{spin_i}=B_E_6;

% build the controllability matrix

Co_6=ctrb(A_E_6,B_E_6);

% Co_c: Controllability matrix cluster

Co_c_6{spin_i}=cond(Co_6);

% seventh trim trajectory

A_v_v_7=A_v_v_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);

A_v_w_7=A_v_w_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);



A_w_v_7=A_w_v_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);

A_w_w_7=A_w_w_l(u_l_7,v_l_7,w_l_7,p_l_7,q_l_7,r_l_7);



% stack all constant velocities into compact vectors 
% The kinematic part: 

nu_C_7=[u_l_7;v_l_7;w_l_7];

omega_C_7=[p_l_7;q_l_7;r_l_7];

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

B_E_7=[B_v_7;B_w_7;zeros(6)];

% B_E_c: Error Input Matrix Clusters for the first trim trajectories:  

B_E_c_7{spin_i}=B_E_7;

% build the controllability matrix

Co_7=ctrb(A_E_7,B_E_7);

% Co_c: Controllability matrix cluster

Co_c_7{spin_i}=cond(Co_7);

% index for different spin directions: 
spin_i=spin_i+1;

               end
             end
         end
      end
  end
end

% after the iteration of all trim trajectory segments, 
% chose the direction with the smallest condition number for each trim
% trajectory segment 

[b_T_min_1,min_index_1]=min(Co_c_1);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
[b_T_min_2,min_index_2]=min(Co_c_2); 
[b_T_min_3,min_index_3]=min(Co_c_3);
[b_T_min_4,min_index_4]=min(Co_c_4);
[b_T_min_5,min_index_5]=min(Co_c_5);
[b_T_min_6,min_index_6]=min(Co_c_6);
[b_T_min_7,min_index_7]=min(Co_c_7);

% now our problem is how to choose the 

%%-------------------------------------------------------------------------------------------------------------------------------------------------------
% main loop for optimization

% use the special trim trajectory segments 

n_trimtraj=7;
% we have seven trim trajecotories 

% for each trim trajectory segment, we have a linearised system
% thus, theorectically we should check the controllability matrix for each
% trim trajectory, then we have totally n_trimtraj*n_thrusters
% Controllability matices


% first let us specify the desired generalised control input

tau_1=tau_d_1(:,:,1);


% formulate the optimization problem as bounded least square optimization problem  

l_bound=[0;0;0;0;0;0];

% the upper bound is the maximal thrust the thruster can generate
u_bound=[20;20;20;20;20;20];

cvx_begin
variable x(6)
minimize(norm(B_T_spin{40}*x-tau_1))
subject to
l_bound<= x <= u_bound
cvx_end



%% formulate the problem as Bound-constrained least squares problem
AA=B_T_spin{40};
bb=tau_1;
x_qp = quadprog( 2*AA'*AA, -2*AA'*bb, [], [], [], [], l_bound, u_bound );
%% In the second phase, we optimize the position vector of the underwater robot 

%% In the third phase,

% formualte the optimization of the direction vector as sequential
% quadraric programming subject to unit vector 