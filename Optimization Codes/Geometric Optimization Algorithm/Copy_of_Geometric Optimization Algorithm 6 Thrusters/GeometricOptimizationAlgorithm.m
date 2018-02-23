%%  The complete geometric optimization algorithm


%% Intialize all the symbolic variables and functions

% initialize the geometric parameters of hull, fins and thrusters

GeometricParaInitialization;

  %Snookie_GeometricParaInitialization;


% initialise all the symbolic variables for the error dynamics and the
% index variable for Controllability Check

ErrordynamicsInitialization;

% calculate the terms which do not change during the optimization including

% 1. the desired 6 velocities (dynamics states)

% 2. the desired roll and pitch angles (kinematic states)

% 3. the drag matrix 

% 4. the added mass Coriolis matrix 

% determine the spin direction of thrusters

TrimTrajectorySpecifications;

% intialize the desired input
% under the randomly initialised configuration                                   

DesiredInputIntialization;

%% In this part we want to get he optimized value of the spin direction of all thrusters

PreProcessing;

% veh.b_T1=1;
% veh.b_T2=1;
% veh.b_T3=-1;
% veh.b_T4=-1;
% 
% % extract PreProcessing to debug
% 

% % 
% % % the following two lines are used to 
% % 

% 
% SnookieGeometry;
% 
% T_S_1={veh.r_T1,veh.b_T1,veh.d_T1};
% T_S_2={veh.r_T2,veh.b_T2,veh.d_T2}; 
% T_S_3={veh.r_T3,veh.b_T3,veh.d_T3};
% T_S_4={veh.r_T4,veh.b_T4,veh.d_T4};
% T_S_5={veh.r_T5,veh.b_T5,veh.d_T5};
% T_S_6={veh.r_T6,veh.b_T6,veh.d_T6};
% 
% ConfigurationUpdate;
% DynamicsUpdate;

%% In this part we want to optimize all the position and orientation of all thrusters

Mainloop;

%% after the optimization, we check the convergence of geometric parameters and simulation



