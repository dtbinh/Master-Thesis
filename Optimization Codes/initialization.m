%% add paths of all subfolders

%% Robot Design Steps:
% 
% last phase should be the combination of the submodules
% from the previous design phase, we have known the position, the mass and moment of inertia of
% inertia of each components, what we want to do now is to combine all of them
% and calculate out the total mass, the moment of inertia and the centre of
% gravity of the whole robot


% the AUV parameter initialization
addpath(strcat(pwd,'/','AUV Parameter Initialization'));

% the Frenet-Serret path planning folder
addpath(strcat(pwd,'/','Path Planning Frenet Frame'));

% modular modelling of the AUV
addpath(strcat(pwd,'/','Modular Modelling of Underwater Robots'));

% subfolders of Modular Modelling of Underwater Robots
addpath(strcat(pwd,'/','Modular Modelling of Underwater Robots','/','MassMomentofInertiaCalculation'));
addpath(strcat(pwd,'/','Modular Modelling of Underwater Robots','/','Hull Hydrodynamic Parameter Calculation'));
addpath(strcat(pwd,'/','Modular Modelling of Underwater Robots','/','AddedMassCalculation'));

% linearization of the 6DOF nonlinear AUV dynamics
addpath(strcat(pwd,'/','Formulas Linearization'));

% Simulink model for UAV: switched LQR control and trim trajectory

% The input of this model is the generalised force (X,Y,Z,K,M,N)

addpath(strcat(pwd,'/','Trim Trajectory Switched LQR Control'));

% Simulink model for different actuators 

% In this model, we use different combinations of fins and thrusters

% The fin and thruster model can be selected and parameterized in the
% Simulink model 

addpath(strcat(pwd,'/','Trim Trajectory Swithced LQR Control (Optimized)'));


% formulate and linearize the error dynamics 
addpath(strcat(pwd,'/','Linearized Error Dynamics'));

% generate the configuration matrix for thruster
addpath(strcat(pwd,'/','Thruster Configuration Matrix Calculation'));

% generate the configuration matrix for fins
addpath(strcat(pwd,'/','Fin Configuration Matrix Calculation'));

% use high order polynomials 

addpath(strcat(pwd,'/','Approximation for Lift and Drag Coefficient'));

% optimization of hull 
addpath(strcat(pwd,'/','Optimization of Hull Size'));

% Model Linearization
% In this file, we use the "linmod" to linearize the Simulink model 

addpath(strcat(pwd,'/','Model Linearization'));

% Geometric optimization 

addpath(strcat(pwd,'/','Geometric Optimization Algorithm'));

% two subfolders are used to test different optimization configuration 

addpath(strcat(pwd,'/','Geometric Optimization Algorithm','/','Geometric Optimization Algorithm 6 Thrusters'));

addpath(strcat(pwd,'/','Geometric Optimization Algorithm','/','Geometric Optimization Algorithm 2 Thrusters 4 Fins'));

% convex optimization toolbox 

addpath(pwd,'/','3rdParty');

addpath(strcat(pwd,'/','3rdParty','/','cvx'));

% run cvx_setup to install the optimization toolbox



% An accurate modeling of hydrodynamic forces using fin frame 
% and calculation of lift and drag force based on angle of attack
% is to complicated and makes the optimizazation 

% %% run parameter initialization
% AUVParaInitialization;
% 
% %% perform path planning
% 
% % formualte the Frenet-Serret path planning formulas 
% PathPlanningFormulation;
% 
% % specify explicit values for each trim trajectory segment
% 
% PathPlanningSpecification;
% 
% %%  Desired Input Calculation
% 
% % For each trim trajectory the input should be constant
% 
% % calculation the static reference input 
% 
% DesiredInputCalculation;
% 
% %% Robot Body Design 


