%% perform linearization for the system 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);


% B_v_l=jacobian(BH_v,[X_l Y_l Z_l K_l M_l N_l]);

B_v_l=symfun(jacobian(BH_v,[T1 T2 T3 T4 T5 T6]),[T1 T2 T3 T4 T5 T6]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l]);


% B_w_l=jacobian(BH_w,[X_l Y_l Z_l K_l M_l N_l]);


B_w_l=symfun(jacobian(BH_w,[T1 T2 T3 T4 T5 T6]),[T1 T2 T3 T4 T5 T6]);