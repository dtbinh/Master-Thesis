%% Gain Scheduling according for a trimming trajectory segment
%% The underwater robot dynamics depend on orientations (3 Euler angles) and six generalised velocities
Q=diag([1 1 1 1 1 1 1000000 1000000 1000000 1000000 1000000 1000000]);
R=diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]);
K_schedule=[];
M=veh.Mrb+veh.Ma;
%%  The operating inputs should be calculated according to the dynamic equation of the robots
v=[linear_velocity_x(1);linear_velocity_y(1);linear_velocity_z(1);angular_velocity_roll(1);angular_velocity_pitch(1);angular_velocity_yaw(1)];
u0=tau_d(:,:,1);
%% specify the linearization point
x0=[0;0;0;roll_angle(1);pitch_angle(1);0;v]; 
%% linearize the model about the point (x0,u0)
[A,B,C,D]=linmod('mlinear_SnookieConfiguration',x0,u0);
[K,P,E]=lqr(A,B,Q,R);
