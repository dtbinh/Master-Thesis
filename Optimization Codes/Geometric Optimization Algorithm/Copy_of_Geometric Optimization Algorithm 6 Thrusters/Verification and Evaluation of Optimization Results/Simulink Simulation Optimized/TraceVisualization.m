%% Compare the desired path with real path for the first segment 
figure;
% plot the reference trajectory
plot3(x_r.Data,y_r.Data,z_r.Data,'LineWidth',2);
hold on;
% plot the real trajectory
plot3(x_s.Data,y_s.Data,z_s.Data,'LineWidth',2);
title('Comparision of Reference Trajectory and Real Trajectory');
xlabel('X: (meter:m)');
ylabel('Y: (meter:m)');
zlabel('Z: (meter:m)');
% axis([0 25 -30 10 -20 5]);
legend('Reference Trajectory','Real Trajectory');