function u_d = DesiredThrusterInput(tau_d,B_T)

% tau_d is the generalised control input in six degrees of freedom 
% we do the transformation to calculate the actual input for each thrusters
% 

u_d=pinv(B_T)*tau_d;


end

