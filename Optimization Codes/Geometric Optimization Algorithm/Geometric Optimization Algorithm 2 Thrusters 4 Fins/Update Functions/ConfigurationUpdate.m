%% unlike the configuration matrix for thrusters, the configuration matrix is trim-trajectory dependent
%% i.e. state-dependent

%% The thruster configuration matrix is not state-dependent
% only two thrusters

B_T=ThrusterConfigurationMatrix(T_S_1,T_S_2);

B_F=FinConfigurationMatrix(Fin1,Fin2,Fin3,Fin4);



% we have seven trim trajectories 
% thus we have seven initial input mapping matrices for the fins
% which will be determined by different trim trajectory segment

% Fin configuration matrix is trim-trajectory dependent 



% compute the fin configuration matrix for the first trim

% B_F_1=FinConfigurationMatrix(Fin1_Trim1,Fin2_Trim1,Fin3_Trim1,Fin4_Trim1);




% compute the fin configuration matrix for the second trim

% B_F_2=FinConfigurationMatrix(Fin1_Trim2,Fin2_Trim2,Fin3_Trim2,Fin4_Trim2);

B_input=horzcat(B_T,B_F);


% compute the fin configuration matrix 

% B_F_3=FinConfigurationMatrix(Fin1_Trim3,Fin2_Trim3,Fin3_Trim3,Fin4_Trim3);




% compute the fin configuration matrix 

% B_F_4=FinConfigurationMatrix(Fin1_Trim4,Fin2_Trim4,Fin3_Trim4,Fin4_Trim4);

% 5. Trim Trajectory

% build fin configuration matrix 


% compute the fin configuration matrix 

% B_F_5=FinConfigurationMatrix(Fin1_Trim5,Fin2_Trim5,Fin3_Trim5,Fin4_Trim5);

% 6. Trim Trajectory

% build fin configuration matrix 



% compute the fin configuration matrix 

% B_F_6=FinConfigurationMatrix(Fin1_Trim6,Fin2_Trim6,Fin3_Trim6,Fin4_Trim6);

% 6. Trim Trajectory

% build fin configuration matrix 




% compute the fin configuration matrix 

% B_F_7=FinConfigurationMatrix(Fin1_Trim7,Fin2_Trim7,Fin3_Trim7,Fin4_Trim7);

% compute the configuration matrix for each trim trajectory segment 






