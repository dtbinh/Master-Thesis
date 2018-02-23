%% We just use one 
%%-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Preprocessing 

% at the first step, we only determine the spin direction of all thrusters
% we have 2^n_thruster configurations

%% for all trim trajectories 

% we want to find the best spin direction group,i.e, the spin direction
% group with the smallest condition number 
% but we should iterate within all trim trajectory segments 

bestConditionNumber=inf;


% First loop of optimization to determine the spin directions 

% In the preprocessing phase, the fin configuration matrix does not change 

% because we only change the value of the spin direction, at least in this
% pre-optimization phase, thus in this phase, we should intialize the fin
% configuration firstly for each trim trajectory segment


% spin direction index is used to the amount of different spin
% configurations 

spin_i=1;

for b_T1=-1:2:1
  for b_T2=-1:2:1
      for b_T3=-1:2:1
          for b_T4=-1:2:1
              for b_T5=-1:2:1
                  for b_T6=-1:2:1
                      
% The following thruster geometric parameter will be used in the
% ConfigurationUpdate
     
T_S_1={veh.r_T1,b_T1,veh.d_T1};
T_S_2={veh.r_T2,b_T2,veh.d_T2}; 
T_S_3={veh.r_T3,b_T3,veh.d_T3};
T_S_4={veh.r_T4,b_T4,veh.d_T4};
T_S_5={veh.r_T5,b_T5,veh.d_T5};
T_S_6={veh.r_T6,b_T6,veh.d_T6};

                   
% stack spin directions of all thrusters into one vector 

b_T=[b_T1;b_T2;b_T3;b_T4;b_T5;b_T6]; 

% put all different spin direction configuration into

% a cell group

b_T_c{spin_i}=b_T;

% We change the Dynamics when we update the spin direction of all thrusters

%%  put the current spin direction of all thrusters into the mapping matrix
% for each thruster 

% after the configuration is fixed
% we should compute the current error dynmaics for each trim trajectory
% segments 

% u_l, v_l, w_l, p_l, q_l, r_l are already specified in the file 
% they are calculated in the initialization file
% "ThrusterConfigurationOptiInit.m"

% for the linearized error dynamics 

% The dynamic part is linearized mathematically using jacobian 

%% evaluate the system matrix 

% in this case, we only have one trim trajec

% first trim trajectory

% update the configuration parameter for both thrusters and fins

% stack the thruster parameters in to a cell

u_ind=v_d_1(1);

ConfigurationUpdate;

DynamicsUpdate;

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

% A_v_lambda_1=zeros(3);
A_v_lambda_1=double(A_v_lambda_l(roll_angle_1(1),pitch_angle_1(1),yaw_angle_1(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));


% error dynamics for angular velocities

A_w_p_1=zeros(3);

% A_w_lambda_1=zeros(3);

A_w_lambda_1=double(A_w_lambda_l(roll_angle_1(1),pitch_angle_1(1),yaw_angle_1(1),veh.m*veh.g,veh.m*veh.g,veh.B_b(1),veh.B_b(2),veh.B_b(3),veh.G_b(1),veh.G_b(2),veh.G_b(3)));

% The input matrix should will not be changed 

B_v_1=double(B_v_l);

B_w_1=double(B_w_l);


% formulate the error dynamics

A_E_1=[A_v_v_1 A_v_w_1 A_v_p_1 A_v_lambda_1;A_w_v_1 A_w_w_1 A_w_p_1 A_w_lambda_1;A_p_v_1 A_p_w_1 A_p_p_1 A_p_lambda_1;A_lambda_v_1 A_lambda_w_1 A_lambda_p_1 A_lambda_lambda_1];

B_E_1=[B_v_1;B_w_1;zeros(6)];


% A_E_c: System Matrix Clusters for the first trim tra

% B_E_c: Error Input Matrix Clusters for the first trim trajectory:  

B_E_c_1{spin_i}=B_E_1;

% build the controllability matrix

Co_1=ctrb(A_E_1,B_E_1);

% Co_c: Controllability matrix cluster

Co_c_1{spin_i}=Co_1;

% index for different spin directions: 
spin_i=spin_i+1;
                  end
              end
          end
      end

   end
end

%% after the iteration of all trim trajectory segments,

% chose the direction with the smallest condition number for each trim
% trajectory segment 

% since here we have 7 trim trajectories


%% using the following methods to chose the best configuration

cond_c=zeros(64,1);
num_cond_c=zeros(64,1);

for i_spin=1:64
    if rank(Co_c_1{i_spin})==12
        cond_c(i_spin)=cond_c(i_spin)+cond(Co_c_1{i_spin});
        num_cond_c(i_spin)=num_cond_c(i_spin)+1;
    end
end


cond_c_av=zeros(64,1);

for j_spin=1:64
    cond_c_av(j_spin)=cond_c(j_spin)/num_cond_c(j_spin);
end

[minval,min_index]=min(cond_c_av);

BestSpinDirection=b_T_c{min_index};
min_d=[];

for i=1:64
    if cond_c_av(i)==minval
        min_d=[min_d;cond_c_av(i)];
    end
end

%% assign the optimal spin direction to the global variable veh

veh.b_T1=BestSpinDirection(1);
veh.b_T2=BestSpinDirection(2);
veh.b_T3=BestSpinDirection(3);
veh.b_T4=BestSpinDirection(4);
veh.b_T5=BestSpinDirection(5);
veh.b_T6=BestSpinDirection(6);

% Define the following variable for simualtion
b_T1_optimized=BestSpinDirection(1);
b_T2_optimized=BestSpinDirection(2);
b_T3_optimized=BestSpinDirection(3);
b_T4_optimized=BestSpinDirection(4);
b_T5_optimized=BestSpinDirection(5);
b_T6_optimized=BestSpinDirection(6);




%%-------------------------------------------------------------------------------------------------------------------------------------------------------
% main loop for optimization

% use the special trim trajectory segments 

% we have seven trim trajecotories 

% for each trim trajectory segment, we have a linearised system
% thus, theorectically we should check the controllability matrix for each
% trim trajectory, then we have totally n_trimtraj*n_thrusters
% Controllability matices


% first let us specify the desired generalised control input


% note that the foumula for calculating the desired input for each trim
% trajectory segmemt

% tau_d must be updated every time we change the geometic configurations since it will influence 
% the rigid body Coriolis matrix, since the geometric centre is changed 

% the dynamic states
% tau_d=C_RB(x_dyn,d_H,r_T,r_F)*x_dyn+C_A(x_dyn,d_H)*x_dyn+D(d_H,x_dyn)*x_dyn+g(x_kin,d_H,r_T,r_F)
% C_A and D actually should be affected by the geometric position as well.
% However, we assume that C_A and D is only determined by the hull geometry
% which is determined in the first optimization phase 

% Update the Coriolis matrix : note that the added mass Coriolis matrix and the drag matrix are not updated   




