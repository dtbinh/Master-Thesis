%% Initialization  for all the parameters of thr underwater robot

% define a global variable "veh", all the geometric and configuration
% parameters are stored in this variable
% run this file before starting simulation

% the global parameter will be updated during the optimization phase

global veh;

veh=vehicle;

%% define the density of the robot hull material
global rho_f;
rho_f=1050;