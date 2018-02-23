%% The fin configuration: 

% fin configuration is determined by three variables 

% The orientation of the fin is denoted by two variables:
% 1. rotate along the unit x-axis-vector [1;0;0] in body frame {b} about angle "a_f"
% 2. the mechanical rotation angle "delta" after the first rotation along
%    z axis of the fin frame [0;1;0]
% 3. the size of the fin: span length and chord length:  

% The position of the fin is denoted by three-dimensional vector r_f=[r_f_x;r_f_y;r_f_z] 
% in the body frame {b}

% The fin configuration is dynamic state dependent (the
%  velocities) (from the trim trajectories-kinematic specification)

% Fins are control surface, the input is the mechanical rotation angle
% "delta" (the mechanical angle delta)

% Thus, the fin configuration is determined by dynamic state variable 
% (from kinematic specification) and the geometric variable  

% Due to the nonlinearity and the deep coupling, the fin input delta (mechanical angle) can not
% be written separately.











