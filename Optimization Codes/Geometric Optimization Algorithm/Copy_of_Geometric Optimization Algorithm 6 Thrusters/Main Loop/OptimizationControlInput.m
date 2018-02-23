%% Optimization for the control input signal 

% formulate the optimization problem as bounded least square optimization problem  

% small angle approximation: 0.2*57.3=11.46 degree


l_bound_input=[-35;-35;-35;-35;-35;-35];
% 
% l_bound_input=[-350;-350;-350;-350;-350;-350];

% the upper bound is the maximal thrust the thruster can generate

u_bound_input=[40;40;40;40;40;40];

% u_bound_input=[400;400;400;400;400;400]

% our consideration is always to minimize the difference between the
% desired generalised force and the real generalised force under the
% current configuration

% minimization problem: unbounded least squares

% In this case, we have a unique configuration matrix for each trim
% trajectory segment, however, actually it is not the case.

% We have seven trim trajectory which means that we should have seven
% configuration matrices 

% the following problem 


cvx_begin
variables u_d_1(6) u_d_2(6) u_d_3(6) u_d_4(6) u_d_5(6) u_d_6(6) u_d_7(6)
% minimize(norm(B*u_d-tau_d_1)+norm(B*u_d-tau_d_2)+norm(B*u_d-tau_d_3)+norm(B*u_d-tau_d_4)+norm(B*u_d-tau_d_5)+norm(B*u_d-tau_d_6)+norm(B*u_d-tau_d_7))
% B is not unique
minimize(norm(B_input*u_d_1-tau_d_1)+norm(B_input*u_d_2-tau_d_2)+norm(B_input*u_d_3-tau_d_3)+norm(B_input*u_d_4-tau_d_4)+norm(B_input*u_d_5-tau_d_5)+norm(B_input*u_d_6-tau_d_6)+norm(B_input*u_d_7-tau_d_7))
subject to
l_bound_input<= u_d_1 <= u_bound_input
l_bound_input<= u_d_2 <= u_bound_input
l_bound_input<= u_d_3 <= u_bound_input
l_bound_input<= u_d_4 <= u_bound_input
l_bound_input<= u_d_5 <= u_bound_input
l_bound_input<= u_d_6 <= u_bound_input
cvx_end

input_error_c{index_loop}=cvx_optval;

% cvx_begin
% variable u_d_1_1(6)
% minimize(norm(B_input*u_d_1_1-tau_d_1))
% subject to
% l_bound_input<= u_d_1_1 <= u_bound_input
% cvx_end
% 
% cvx_begin
% variable u_d_2_1(6)
% minimize(norm(B_input*u_d_2_1-tau_d_2))
% subject to
% l_bound_input<= u_d_2_1 <= u_bound_input
% cvx_end
% 
% cvx_begin
% variable u_d_3_1(6)
% minimize(norm(B_input*u_d_3_1-tau_d_3))
% subject to
% l_bound_input<= u_d_2_1 <= u_bound_input
% cvx_end

% %% formulate the problem as Bound-constrained least squares problem 
% AA=B_T_spin{40};
% bb=tau_1;
% x_qp = quadprog( 2*AA'*AA, -2*AA'*bb, [], [], [], [], l_bound, u_bound );


%% In the second phase, we optimize the position vector of the underwater robot 

%% In the third phase,

% formualte the optimization of the direction vector as sequential
% quadraric programming subject to unit vector 

