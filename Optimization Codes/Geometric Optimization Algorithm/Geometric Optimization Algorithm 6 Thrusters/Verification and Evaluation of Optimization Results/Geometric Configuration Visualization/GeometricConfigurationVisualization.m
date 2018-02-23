%% visualization for the underwater robot prototype

%  visualize the hull as a cylinder the length l_H and radius r_H come 
%  from the first optimization phase

%%  use the surface to draw the the hull

N=50;
R=d_H/2;

    %// Compute cylinder coords (mostly borrowed from builtin 'cylinder')   
    theta         = 2*pi*(0:N)/N;
    sintheta      = sin(theta); 
    sintheta(N+1) = 0;

    M = length(R);
    if M==1 
        R = [R;R]; M = 2; 
    end

    y = R(:) * cos(theta);
    z = R(:) * sintheta;
    x = (0:M-1).'/(M-1) * ones(1,N+1);  
    
    % make the 
    x=x.*l_H-l_H/2;
figure;
surface(x,y,z)
hold on;

%% plot the body coordinate frame
% note that for the underwater robot, the positive direction of the z-axis
% is down
% to keep the right-hand-rule, the x-coordinate should also be reversed 
% 

fcn_plotUnitVectors([0 0 0],[-0.01 0 0],[0 0.01 0],[0 0 -0.01],8);
hold on;

%% plot the thrusters of according to the optimized directions and positions

%  Thruster 1
scatter3(-r_T1_optimized(1),r_T1_optimized(2),-r_T1_optimized(3),'y','filled');
hold on;
[start_point,end_point] = CoorPlotTransfer(r_T1_optimized,d_T1_optimized,0.5);
quiver3(start_point(1),start_point(2),start_point(3),end_point(1),end_point(2),end_point(3),'Color','y','LineWidth',3,'MaxHeadSize',0.5);
hold on;

%  Thruster 2 
scatter3(-r_T2_optimized(1),r_T2_optimized(2),-r_T2_optimized(3),'r','filled');
hold on;
[start_point,end_point] = CoorPlotTransfer(r_T2_optimized,d_T2_optimized,0.5);
quiver3(start_point(1),start_point(2),start_point(3),end_point(1),end_point(2),end_point(3),'Color','r','LineWidth',3,'MaxHeadSize',0.5);
hold on;

%  Thruster 3
scatter3(-r_T3_optimized(1),r_T3_optimized(2),-r_T3_optimized(3),'g','filled');
hold on;
[start_point,end_point] = CoorPlotTransfer(r_T3_optimized,d_T3_optimized,0.5);
quiver3(start_point(1),start_point(2),start_point(3),end_point(1),end_point(2),end_point(3),'Color','g','LineWidth',3,'MaxHeadSize',0.5)
hold on;

% Thruster 4
scatter3(-r_T4_optimized(1),r_T4_optimized(2),-r_T4_optimized(3),'b','filled');
hold on;
[start_point,end_point] = CoorPlotTransfer(r_T4_optimized,d_T4_optimized,0.5);
quiver3(start_point(1),start_point(2),start_point(3),end_point(1),end_point(2),end_point(3),'Color','b','LineWidth',3,'MaxHeadSize',0.5)
hold on;

% Thruster 5
scatter3(-r_T5_optimized(1),r_T5_optimized(2),-r_T5_optimized(3),'m','filled');
hold on;
[start_point,end_point] = CoorPlotTransfer(r_T5_optimized,d_T5_optimized,0.5);
quiver3(start_point(1),start_point(2),start_point(3),end_point(1),end_point(2),end_point(3),'Color','m','LineWidth',3,'MaxHeadSize',0.5)
hold on;

% Thruster 6 
scatter3(-r_T6_optimized(1),r_T6_optimized(2),-r_T6_optimized(3),'c','filled')
hold on;
[start_point,end_point] = CoorPlotTransfer(r_T6_optimized,d_T6_optimized,0.5);
quiver3(start_point(1),start_point(2),start_point(3),end_point(1),end_point(2),end_point(3),'Color','c','LineWidth',3,'MaxHeadSize',0.5)
hold on;
