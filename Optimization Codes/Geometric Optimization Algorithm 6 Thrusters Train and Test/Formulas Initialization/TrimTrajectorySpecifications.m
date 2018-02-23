%% For testing and traning we just need two trim trajectories

%% Trim Trajectory Specifications

% here we have seven trim trajectory segments 

% The desired six dynamic states and the two kinematic states come from the Frenet-Serret Path Planning 
% and they will not change in every iteration, thus they are listed in the
% initialization phase

% desired velocity is fixed  for all trim trajectory segments: 

v_d_1=[linear_velocity_x_1(1),linear_velocity_y_1(1),linear_velocity_z_1(1),angular_velocity_roll_1(1),angular_velocity_pitch_1(1),angular_velocity_yaw_1(1)];

v_d_2=[linear_velocity_x_2(1),linear_velocity_y_2(1),linear_velocity_z_2(1),angular_velocity_roll_2(1),angular_velocity_pitch_2(1),angular_velocity_yaw_2(1)];


% The desired two kinematic states (roll,pitch) also stay constant, they
% come from the desired trim trajectories

% roll angles

roll_d_1=roll_angle_1(1);

roll_d_2=roll_angle_2(1);



% pitch angles

pitch_d_1=pitch_angle_1(1);

pitch_d_2=pitch_angle_2(1);



% both the drag and Coriolis matrix are determined by the hull geometric
% parameter, it is only an assumption to facilitate the optimization 

% The drag matrix is not updated during the actuator optimization phase 

D1=veh.D*(v_d_1'.*abs(v_d_1)');

D2=veh.D*(v_d_2'.*abs(v_d_2)');





% The added mass Coriolis matrix is not updated during the actuator phase

% update the rigid body Coriolis matrix

C_A1=tau_A_cor(veh,v_d_1');


C_A2=tau_A_cor(veh,v_d_2');



% since their coefficients are only determined by the hull geometric parameter  