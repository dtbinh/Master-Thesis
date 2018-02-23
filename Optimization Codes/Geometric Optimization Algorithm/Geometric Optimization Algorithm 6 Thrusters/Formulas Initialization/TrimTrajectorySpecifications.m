%% Trim Trajectory Specifications

% here we have seven trim trajectory segments 

% The desired six dynamic states and the two kinematic states come from the Frenet-Serret Path Planning 
% and they will not change in every iteration, thus they are listed in the
% initialization phase

% desired velocity is fixed  for all trim trajectory segments: 

v_d_1=[linear_velocity_x_1(1),linear_velocity_y_1(1),linear_velocity_z_1(1),angular_velocity_roll_1(1),angular_velocity_pitch_1(1),angular_velocity_yaw_1(1)];

v_d_2=[linear_velocity_x_2(1),linear_velocity_y_2(1),linear_velocity_z_2(1),angular_velocity_roll_2(1),angular_velocity_pitch_2(1),angular_velocity_yaw_2(1)];

v_d_3=[linear_velocity_x_3(1),linear_velocity_y_3(1),linear_velocity_z_3(1),angular_velocity_roll_3(1),angular_velocity_pitch_3(1),angular_velocity_yaw_3(1)];

v_d_4=[linear_velocity_x_4(1),linear_velocity_y_4(1),linear_velocity_z_4(1),angular_velocity_roll_4(1),angular_velocity_pitch_4(1),angular_velocity_yaw_4(1)];

v_d_5=[linear_velocity_x_5(1),linear_velocity_y_5(1),linear_velocity_z_5(1),angular_velocity_roll_5(1),angular_velocity_pitch_5(1),angular_velocity_yaw_5(1)];

v_d_6=[linear_velocity_x_6(1),linear_velocity_y_6(1),linear_velocity_z_6(1),angular_velocity_roll_6(1),angular_velocity_pitch_6(1),angular_velocity_yaw_6(1)];

v_d_7=[linear_velocity_x_7(1),linear_velocity_y_7(1),linear_velocity_z_7(1),angular_velocity_roll_7(1),angular_velocity_pitch_7(1),angular_velocity_yaw_7(1)];

% The desired two kinematic states (roll,pitch) also stay constant, they
% come from the desired trim trajectories

% roll angles

roll_d_1=roll_angle_1(1);

roll_d_2=roll_angle_2(1);

roll_d_3=roll_angle_3(1);

roll_d_4=roll_angle_4(1);

roll_d_5=roll_angle_5(1);

roll_d_6=roll_angle_6(1);

roll_d_7=roll_angle_7(1);

% pitch angles

pitch_d_1=pitch_angle_1(1);

pitch_d_2=pitch_angle_2(1);

pitch_d_3=pitch_angle_3(1);

pitch_d_4=pitch_angle_4(1);

pitch_d_5=pitch_angle_5(1);

pitch_d_6=pitch_angle_6(1);

pitch_d_7=pitch_angle_7(1);


% both the drag and Coriolis matrix are determined by the hull geometric
% parameter, it is only an assumption to facilitate the optimization 

% The drag matrix is not updated during the actuator optimization phase 

D1=veh.D*(v_d_1'.*abs(v_d_1)');

D2=veh.D*(v_d_2'.*abs(v_d_2)');

D3=veh.D*(v_d_3'.*abs(v_d_3)');

D4=veh.D*(v_d_4'.*abs(v_d_4)');

D5=veh.D*(v_d_5'.*abs(v_d_5)');

D6=veh.D*(v_d_6'.*abs(v_d_6)');

D7=veh.D*(v_d_7'.*abs(v_d_7)');



% The added mass Coriolis matrix is not updated during the actuator phase

% update the rigid body Coriolis matrix

C_A1=tau_A_cor(veh,v_d_1');


C_A2=tau_A_cor(veh,v_d_2');


C_A3=tau_A_cor(veh,v_d_3');


C_A4=tau_A_cor(veh,v_d_4');


C_A5=tau_A_cor(veh,v_d_5');


C_A6=tau_A_cor(veh,v_d_6');


C_A7=tau_A_cor(veh,v_d_7');

% since their coefficients are only determined by the hull geometric parameter  