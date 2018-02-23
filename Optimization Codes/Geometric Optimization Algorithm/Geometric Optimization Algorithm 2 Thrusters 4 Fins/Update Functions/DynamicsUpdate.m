veh.r_F1=[veh.x_F1;0.5*(d_H+a_F1)*sin(veh.gamma_F1);0.5*(d_H+a_F1)*cos(veh.gamma_F1)];
veh.r_F2=[veh.x_F2;0.5*(d_H+a_F2)*sin(veh.gamma_F2);0.5*(d_H+a_F2)*cos(veh.gamma_F2)];
veh.r_F3=[veh.x_F3;0.5*(d_H+a_F3)*sin(veh.gamma_F3);0.5*(d_H+a_F3)*cos(veh.gamma_F3)];
veh.r_F4=[veh.x_F4;0.5*(d_H+a_F4)*sin(veh.gamma_F4);0.5*(d_H+a_F4)*cos(veh.gamma_F4)];




r_g_new=CenterofMass({m_H,r_H},{m_T1,veh.r_T1},{m_T2,veh.r_T2},{m_F1,veh.r_F1},{m_F2,veh.r_F2},{m_F3,veh.r_F3},{m_F4,veh.r_F4});

% update the center of mass in the global variable veh

veh.G_b=r_g_new;

% now let us update the rigid body inertia Matrix since the third parameter
% r_G is updated with the new one r_g_new

% Note that the Moment of Inertia changes too

% However we assume that the Moment of Inertia is only determined by the
% hull, hence it is fixed during the second optimization phase 

Mrb_new=RigidBodyInertiaMatrix(veh.m,veh.iota,r_g_new);

% then we update the veh.Mrb by Mrb_new



veh.Mrb=Mrb_new;



%% Dynamics Update 

% after the configuration is changed, we have a new input mapping matrix,  
% the term BH and all other terms involving BH should be updated in the
% script "LinearizationFormulas"

% we update BH at first 

M_RB=veh.Mrb;


%% F_kin

F_lambda=-inv(M_RB+veh.Ma)*g;

% divide the kinematic part in dynamic equation into two parts (linear velocity part and angular velocity part)

F_v_lambda=F_lambda(1:3,:);

F_w_lambda=F_lambda(4:6,:);

%% F_dyn

F=-inv(M_RB+veh.Ma)*(CRB+CA+Dg);

% divide the dynamic part in dynamic equation into two parts (linear velocity part and angular velocity part) 

F_v=F(1:3,:);

F_w=F(4:6,:);




BH=inv(M_RB+veh.Ma)*B_input*[T1;T2;delta1;delta2;delta3;delta4];


BH_v=BH(1:3,:);

BH_w=BH(4:6,:);

% then we should update all the matices relating to BH

% update of the inputs 

% input matrix 

B_v_l=jacobian(BH_v,[T1 T2 delta1 delta2 delta3 delta4]);


B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% update all the symbolic function 

%------ linear velocity part ----------------------------------------------

% dynamic states 

A_v_v_l=symfun(jacobian(F_v+BH_v,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_v_w_l=symfun(jacobian(F_v+BH_v,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_v_lambda_l=symfun(jacobian(F_v_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);

% ---- angular velocity part
% ----------------------------------------------------------------------

% dynamic states 

A_w_v_l=symfun(jacobian(F_w+BH_w,[u_l v_l w_l]),[u_l v_l w_l p_l q_l r_l]);

A_w_w_l=symfun(jacobian(F_w+BH_w,[p_l q_l r_l]),[u_l v_l w_l p_l q_l r_l]);


% kinematic states 

A_w_lambda_l=symfun(jacobian(F_w_lambda,[phi_l,theta_l,psi_l]),[phi_l theta_l psi_l B W x_b y_b z_b x_g y_g z_g]);