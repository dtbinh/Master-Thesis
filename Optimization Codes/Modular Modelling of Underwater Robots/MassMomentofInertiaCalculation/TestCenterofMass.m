%% Test for the function "CenterofMass"

% what we can do is to add parameters adapted from Snookie


% T100 Thruster weight in the water 0.12kg

% location and mass of each modules 

% hull {25,[0,0,0]}
% fins  {1.5,[-0.367;0.05;0]}
%       {1.5,[-0.367;0;0.05]}
%       {1.5,[-0.367;-0.05;0]}
%       {1.5,[-0.393;0;-0.05]}
% two thrusters:

%        {0.12,[0.00276;-0.1799;0]}
%        {0.12,[0.00276;-0.1799;0]}

r_G_Test=CenterofMass({1.5,[-0.367;0.05;0]},{1.5,[-0.367;0;0.05]},{1.5,[-0.367;-0.05;0]},{1.5,[-0.393;0;-0.05]},{0.12,[0.00276;-0.1799;0]},{0.12,[0.00276;-0.1799;0]});



