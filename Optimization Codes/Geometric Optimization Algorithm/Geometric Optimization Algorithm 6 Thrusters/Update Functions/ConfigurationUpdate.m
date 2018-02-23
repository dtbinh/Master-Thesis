


% compute the thruster configuration matrix for each spin direction
% combination 

B_T=ThrusterConfigurationMatrix(T_S_1,T_S_2,T_S_3,T_S_4,T_S_5,T_S_6);

% put all thruster configuration matrices with different spin directions 

B_input=B_T;

% concatenate the matrix 


% we have seven trim trajectories 
% thus we have seven initial input mapping matrices for the fins
% which will be determined by different trim trajectory segment

% Fin configuration matrix is trim-trajectory dependent 

% 1. Trim Trajectory

% build fin configuration matrix 




% compute the fin configuration matrix 

% B_F=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);

% 2. Trim Trajectory

% build fin configuration matrix 
% 
% Fin1={C_L,[a_F1,b_F1],u2,[x_F1,gama_F1]};
% 
% Fin2={C_L,[a_F2,b_F2],u2,[x_F2,gama_F2]};
% 
% Fin3={C_L,[a_F3,b_F3],u2,[x_F3,gama_F3]};
% 
% Fin4={C_L,[a_F4,b_F4],u2,[x_F4,gama_F4]};


% compute the fin configuration matrix 

% B_F_2=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);

% 3. Trim Trajectory

% build fin configuration matrix 

% Fin1={C_L,[a_F1,b_F1],u3,[x_F1,gama_F1]};
% 
% Fin2={C_L,[a_F2,b_F2],u3,[x_F2,gama_F2]};
% 
% Fin3={C_L,[a_F3,b_F3],u3,[x_F3,gama_F3]};
% 
% Fin4={C_L,[a_F4,b_F4],u3,[x_F4,gama_F4]};


% compute the fin configuration matrix 

% B_F_3=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);

% 4. Trim Trajectory

% build fin configuration matrix 

% Fin1={C_L,[a_F1,b_F1],u4,[x_F1,gama_F1]};
% 
% Fin2={C_L,[a_F2,b_F2],u4,[x_F2,gama_F2]};
% 
% Fin3={C_L,[a_F3,b_F3],u4,[x_F3,gama_F3]};
% 
% Fin4={C_L,[a_F4,b_F4],u4,[x_F4,gama_F4]};


% compute the fin configuration matrix 

% B_F_4=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);

% 5. Trim Trajectory

% build fin configuration matrix 

% Fin1={C_L,[a_F1,b_F1],u5,[x_F1,gama_F1]};
% 
% Fin2={C_L,[a_F2,b_F2],u5,[x_F2,gama_F2]};
% 
% Fin3={C_L,[a_F3,b_F3],u5,[x_F3,gama_F3]};
% 
% Fin4={C_L,[a_F4,b_F4],u5,[x_F4,gama_F4]};


% compute the fin configuration matrix 

% B_F_5=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);

% 6. Trim Trajectory

% build fin configuration matrix 

% Fin1={C_L,[a_F1,b_F1],u6,[x_F1,gama_F1]};
% 
% Fin2={C_L,[a_F2,b_F2],u6,[x_F2,gama_F2]};
% 
% Fin3={C_L,[a_F3,b_F3],u6,[x_F3,gama_F3]};
% 
% Fin4={C_L,[a_F4,b_F4],u6,[x_F4,gama_F4]};


% compute the fin configuration matrix 

% B_F_6=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);

% 6. Trim Trajectory

% build fin configuration matrix 

% Fin1={C_L,[a_F1,b_F1],u7,[x_F1,gama_F1]};
% 
% Fin2={C_L,[a_F2,b_F2],u7,[x_F2,gama_F2]};
% 
% Fin3={C_L,[a_F3,b_F3],u7,[x_F3,gama_F3]};
% 
% Fin4={C_L,[a_F4,b_F4],u7,[x_F4,gama_F4]};


% compute the fin configuration matrix 

% B_F_7=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);

% compute the configuration matrix for each trim trajectory segment 

%B=horzcat(B_T,B_F);

% B2=horzcat(B_T,B_F_2);
% 
% B3=horzcat(B_T,B_F_3);
% 
% B4=horzcat(B_T,B_F_4);
% 
% B5=horzcat(B_T,B_F_5);
% 
% B6=horzcat(B_T,B_F_6);
% 
% B7=horzcat(B_T,B_F_7);
% 
% 
% % B_spin{spin_i}=B;
