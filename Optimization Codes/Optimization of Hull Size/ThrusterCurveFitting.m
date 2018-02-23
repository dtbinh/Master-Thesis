
%% thrust-power/thrust-current curve fitting for T100 Thruster
clear all;
clc;
load T100Thruster; % all data about the thruster T100 are stored in T100Thruster
thrust_newton_T100=ForceConversion(thrust,1); % convert the thrust from unit pound-force to Newton
f1_T100=fit(thrust_newton_T100,power,'poly5')
figure('Name','Thrust-power characteristic curve of T100 thruster')
plot(f1_T100,thrust_newton_T100,power);
title('Thrust-power characteristic curbe of T100 thruster');
xlabel('Thrust (N)');
ylabel('Power (W)');
f2_T100=fit(thrust_newton_T100,current,'poly5')
figure('Name','Thrust-current characteristic curve of T100 thruster')
plot(f2_T100,thrust_newton_T100,current);
title('Thrust-current characteristic curve of T200 thruster');
xlabel('Thrust (N)');
ylabel('Current (A)')

%% thrust-power/thrust-current curve fitting for T200 Thruster 
clear all;
clc;
load T200Thruster; % all data about the thruster T200 are stored in T200Thruster
% @ 12V
thrust_newton_T200_12V=ForceConversion(thrust_12V,1); % convert the thrust from unit pound-force to Newton
f1_T200_12V=fit(thrust_newton_T200_12V,power_12V,'poly5');
figure('Name','Thrust-power charactersitic curve of T200 thruster @12V');
plot(f1_T200_12V,thrust_newton_T200_12V,power_12V);
title('Thrust-power characteristic curbe of T200 thruster @12V');
xlabel('Thrust (N)');
ylabel('Power (W)');

% @ 16V
thrust_newton_T200_16V=ForceConversion(thrust_16V,1); % convert the thrust from unit pound-force to Newton
f1_T200_16V=fit(thrust_newton_T200_16V,power_16V,'poly5');
figure('Name','Thrust-power charactersitic curve of T200 thruster @16V');
plot(f1_T200_16V,thrust_newton_T200_16V,power_16V);
title('Thrust-power characteristic curbe of T200 thruster @16V');
xlabel('Thrust (N)');
ylabel('Power (W)');
