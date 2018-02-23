%% check the relationship between angle of attack, flight path angle and pitch angle 
alpha=[]; % angle of attack
for i=1:length(linear_velocity_x)
    u=linear_velocity_x(i);
    w=linear_velocity_z(i);
    Alpha=AttackAngleCalculation(u,w);
    alpha=[alpha;Alpha];
end

%% check the Euler angle velocity
% accoring to the trimming theory 