%% Optimization for the control input signal 

% formulate the optimization problem as bounded least square optimization problem  

% small angle approximation: 0.2*57.3=11.46 degree


l_bound_input=[-35;-35;-35;-35;-35;-35];

% the upper bound is the maximal thrust the thruster can generate

u_bound_input=[40;40;40;40;40;40];

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
variable u_d(6)
% minimize(norm(B*u_d-tau_d_1)+norm(B*u_d-tau_d_2)+norm(B*u_d-tau_d_3)+norm(B*u_d-tau_d_4)+norm(B*u_d-tau_d_5)+norm(B*u_d-tau_d_6)+norm(B*u_d-tau_d_7))
% B is not unique
minimize(norm(B_input*u_d-tau_d_1)+norm(B_input*u_d-tau_d_2)+norm(B_input*u_d-tau_d_3)+norm(B_input*u_d-tau_d_4)+norm(B_input*u_d-tau_d_5)+norm(B_input*u_d-tau_d_6)+norm(B_input*u_d-tau_d_7))
subject to
l_bound_input<= u_d <= u_bound_input
cvx_end



% %% formulate the problem as Bound-constrained least squares problem 
% AA=B_T_spin{40};
% bb=tau_1;
% x_qp = quadprog( 2*AA'*AA, -2*AA'*bb, [], [], [], [], l_bound, u_bound );


%% In the second phase, we optimize the position vector of the underwater robot 

%% In the third phase,

% formualte the optimization of the direction vector as sequential
% quadraric programming subject to unit vector 

