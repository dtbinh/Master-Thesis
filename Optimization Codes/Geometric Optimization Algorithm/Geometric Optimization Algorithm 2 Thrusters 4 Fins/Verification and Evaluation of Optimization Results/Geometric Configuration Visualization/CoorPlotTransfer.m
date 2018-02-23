function [start_point,end_point] = CoorPlotTransfer(r_T_optimized,d_T_optimized,shortenrate)

% In this function we perform two adaptions for the coordinates

% The plot frame is different from the robot body frame, in plot frame, the z-axis is up

% in the robot body frame, the z-axis is down

% Thus, two adaptions:

% 1. we shorten the direction vector proportionally with coefficient 0.5

% 2. the z axis should be reversed 


% we just reverse the x-component and the z-component to transfer 

% transfer the coordinates from robot body frame into plot frame 

start_point=[-r_T_optimized(1);r_T_optimized(2);-r_T_optimized(3)];

end_point=[shortenrate*(-r_T_optimized(1)+d_T_optimized(1)),shortenrate*(r_T_optimized(2)+d_T_optimized(2)),-shortenrate*(r_T_optimized(3)+d_T_optimized(3))];




end

