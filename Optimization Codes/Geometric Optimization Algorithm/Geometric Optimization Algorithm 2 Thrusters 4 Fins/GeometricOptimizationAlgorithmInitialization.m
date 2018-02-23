
%% Root file for the optimization algorithm

%% Configuration Information 

%% 2 Thrusters and four fins 

%%

% In this file we want to formulate the basic optimization problem, how to
% optimize locations of thrusters, 

% we have already an initial value of all geometric variables, first we
% optimize location of the thrusters, 
% 

% we use the Snookie configuration as our initial configura tion for
% optimization


%% Optimization Algorithm for actuators placement

addpath(strcat(pwd,'/','Formulas Initialization'));

addpath(strcat(pwd,'/','Robot Model Functions'));

addpath(pwd,'/','Update Functions');

addpath(pwd,'/','Verification and Evaluation of Optimization Results');

addpath(strcat(pwd,'/','Preprocessing'));

addpath(strcat(pwd,'/','Main Loop'));

addpath(strcat(pwd,'/','Test Data'));

addpath(strcat(pwd,'/','Verification and Evaluation of Optimization Results'));

addpath(strcat(pwd,'/','Verification and Evaluation of Optimization Results','/','Simulink Simulation Unptimized'));

addpath(strcat(pwd,'/','Verification and Evaluation of Optimization Results','/','Simulink Simulation Optimized'));

addpath(strcat(pwd,'/','Verification and Evaluation of Optimization Results','/','Geometric Configuration Visualization'));




% %% vehicle parameter optimization
% 
% AUVParaInitialization;
% 
% %% Get the dynamic and kinematic specification from Frenet-Serret Path Planning
% 
% % Problem in this part
% % the veh.D will disappear after I run this part codes
% % repeated vehcile functions 
% 
% PathPlanningFormulation;
% 
% PathPlanningSpecification;

% DesiredInputCalculation;


