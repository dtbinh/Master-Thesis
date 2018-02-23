%% Optimization for the control input signal 

% formulate the optimization problem as bounded least square optimization problem  

% small angle approximation: 0.2*57.3=11.46 degree


l_bound_input=[-35;-35;-1;-1;-1;-1];


u_bound_input=[40;40;1;1;1;1];

% our consideration is always to minimize the difference between the
% desired generalised force and the real generalised force under the
% current configuration

% minimization problem: unbounded least squares

% In this case, we have a unique configuration matrix for each trim
% trajectory segment, however, actually it is not the case.

% We have seven trim trajectory which means that we should have seven
% configuration matrices 

% the following problem 

% note the structure of the input u_d, u_d is a 6 dimensional vector, the
% first two components are related to the thrust force and the last four
% items are related to the 

%% note the difference between optimization with only thrusters and optimization with thrusters and fins

% when fins exist, for each trim trajectory, we will have a new input
% mapping matrix 

% the interpretation of u_d_j(6)

% u_d_j(1): thrust u1

% u_d_j(2): thrust u2

% u_d_j(3): deflection angle delta1

% u_d_j(4): deflection angle delta2

% u_d_j(5): deflection angle delta3

% u_d_j(6): deflection angle delta4

% B_input_1 B_input_2 B_input_3 B_input_4 B_input_5 B_input_6 B_input_7
% should be generated here

% 1. Trim Trajectory 

Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

B_input_1=B_input;

% 2. Trim Trajectory 

Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

B_input_2=B_input;

% 3. Trim Trajectory 

Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

B_input_3=B_input;

% 4. Trim Trajectory 

Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

B_input_4=B_input;

% 5. Trim Trajectory 

Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

B_input_5=B_input;

% 6. Trim Trajectory 

Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

B_input_6=B_input;


% 7. Trim Trajectory 

Fin1={C_L,[a_F1,b_F1],u_ind_1,[veh.x_F1,veh.gamma_F1]};

Fin2={C_L,[a_F2,b_F2],u_ind_1,[veh.x_F2,veh.gamma_F2]};

Fin3={C_L,[a_F3,b_F3],u_ind_1,[veh.x_F3,veh.gamma_F3]};

Fin4={C_L,[a_F4,b_F4],u_ind_1,[veh.x_F4,veh.gamma_F4]};


ConfigurationUpdate;

B_input_7=B_input;



cvx_begin
variables u_d_1(6) u_d_2(6) u_d_3(6) u_d_4(6) u_d_5(6) u_d_6(6) u_d_7(6)

minimize(norm(B_input_1*u_d_1-tau_d_1)+norm(B_input_2*u_d_2-tau_d_2)+norm(B_input_3*u_d_3-tau_d_3)+norm(B_input_4*u_d_4-tau_d_4)+norm(B_input_5*u_d_5-tau_d_5)+norm(B_input_6*u_d_6-tau_d_6)+norm(B_input_7*u_d_7-tau_d_7))
subject to
l_bound_input<= u_d_1 <= u_bound_input
l_bound_input<= u_d_2 <= u_bound_input
l_bound_input<= u_d_3 <= u_bound_input
l_bound_input<= u_d_4 <= u_bound_input
l_bound_input<= u_d_5 <= u_bound_input
l_bound_input<= u_d_6 <= u_bound_input
cvx_end


%% save the mistake 

input_error_c{index_loop}=cvx_optval;



%% In the second phase, we optimize the position vector of the underwater robot 

%% In the third phase, 

% formualte the optimization of the direction vector as sequential
% quadraric programming subject to unit vector 

