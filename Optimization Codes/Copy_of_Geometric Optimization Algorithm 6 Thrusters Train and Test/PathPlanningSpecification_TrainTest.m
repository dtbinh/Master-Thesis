%% This type of trajectory tracking 

%% initialize the trajectory (x,y,z) (desired locations)

% the starting time and time interval are used to avoid the sigularity
% singularity must be deeply studied 


% characteristic of the two trim trajectories

% for the first one, intuitively we regard it as more difficult, the time
% is longer, the flight angle is bigger and the yaw rate is bigger

%% parameterize the lasting time for each trimming trajectory segment

% 1. Trimming Trajectory

t_1_last=100;

t_2_last=25;

% 2. Trimming Trajectory

% explicit values of t: discrete values

% t explicit 

t_e_1=0:1/20:t_1_last; 

t_e_2=0:1/20:t_2_last;

%% 1.Trimming Trajectory

% trajectory paramters:

U_e_1=zeros(1,length(t_e_1));
U_e_1(1,:)=2;

% yaw rate is nearly equal to zero: a very small value

psi_dot_e_1=zeros(1,length(t_e_1));
psi_dot_e_1(1,:)=0.1;

gama_e_1=zeros(1,length(t_e_1));
gama_e_1(1,:)=0.5;

% initial condition:

x_0_e_1=zeros(1,length(t_e_1));
x_0_e_1(1,:)=0;

y_0_e_1=zeros(1,length(t_e_1));
y_0_e_1(1,:)=0;

z_0_e_1=zeros(1,length(t_e_1));
z_0_e_1(1,:)=0;

psi_0_e_1=zeros(1,length(t_e_1));
psi_0_e_1(1,:)=0;

% calculate the position of the first trimming trajectory segment: 
% position(x,y,z) is important for us: this is the value we want to track.

x1=trajx(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
x1=double(x1);

y1=trajy(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
y1=double(y1);

z1=trajz(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
z1=double(z1);

% calculate the orientation of the first trimming trajectory
% for trimming trajectory, roll, pitch should be constant, yaw increases at
% constant speed

% although we konw that roll and pitch angle are time-invariant, we still calculate it for every time instant


roll_angle_1=trajphi(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
roll_angle_1=double(roll_angle_1);

pitch_angle_1=trajtheta(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
pitch_angle_1=double(pitch_angle_1);

yaw_angle_1=trajpsi(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
yaw_angle_1=double(yaw_angle_1);

% calculate the linear velocities (desired linear velocities)

linear_velocity_x_1=linvel_x(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
linear_velocity_x_1=double(linear_velocity_x_1)';

linear_velocity_y_1=linvel_y(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
linear_velocity_y_1=double(linear_velocity_y_1)';

linear_velocity_z_1=linvel_z(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
linear_velocity_z_1=double(linear_velocity_z_1)';

% calculate the linear accelerations (desired linear accelerations)

linear_acceleration_x_1=linacc_x(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
linear_acceleration_x_1=(double(linear_acceleration_x_1))';

linear_acceleration_y_1=linacc_y(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
linear_acceleration_y_1=(double(linear_acceleration_y_1))';

linear_acceleration_z_1=linacc_z(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
linear_acceleration_z_1=(double(linear_acceleration_z_1))';

% calculate angular velocities

angular_velocity_roll_1=angvel_roll(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
angular_velocity_roll_1=(double(angular_velocity_roll_1))';

angular_velocity_pitch_1=angvel_pitch(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
angular_velocity_pitch_1=double(angular_velocity_pitch_1)';

angular_velocity_yaw_1=angvel_yaw(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
angular_velocity_yaw_1=double(angular_velocity_yaw_1)';

% calculate angular accelerations

angular_acceleration_roll_1=angacc_roll(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
angular_acceleration_roll_1=(double(angular_acceleration_roll_1))';

angular_acceleration_pitch_1=angacc_pitch(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
angular_acceleration_pitch_1=(double(angular_acceleration_pitch_1))';

angular_acceleration_yaw_1=angacc_yaw(t_e_1,U_e_1,psi_dot_e_1,gama_e_1,x_0_e_1,y_0_e_1,z_0_e_1,psi_0_e_1);
angular_acceleration_yaw_1=(double(angular_acceleration_yaw_1))';

%% 2.Trimming Trajectory

% trajectory paramters:

U_e_2=zeros(1,length(t_e_2));
U_e_2(1,:)=1;

psi_dot_e_2=zeros(1,length(t_e_2));
psi_dot_e_2(1,:)=-0.35;

gama_e_2=zeros(1,length(t_e_2));
gama_e_2(1,:)=0.6;

% initial condition:

x_0_e_2=zeros(1,length(t_e_2));
x_0_e_2(1,:)=x1(end);

y_0_e_2=zeros(1,length(t_e_2));
y_0_e_2(1,:)=y1(end);

z_0_e_2=zeros(1,length(t_e_2));
z_0_e_2(1,:)=z1(end);

psi_0_e_2=zeros(1,length(t_e_2));
psi_0_e_2(1,:)=psi_0_e_1(1,1)+psi_dot_e_1(1,1)*t_e_1(end);

% calculate the position of the second trimming trajtory

x2=trajx(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
x2=double(x2);

y2=trajy(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
y2=double(y2);

z2=trajz(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
z2=double(z2);

% calculate the orientation of the second trimming trajectory

roll_angle_2=trajphi(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
roll_angle_2=double(roll_angle_2);

pitch_angle_2=trajtheta(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
pitch_angle_2=double(pitch_angle_2);

yaw_angle_2=trajpsi(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
yaw_angle_2=double(yaw_angle_2);

% calculate the linear velocities (desired linear velocities)

linear_velocity_x_2=linvel_x(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
linear_velocity_x_2=double(linear_velocity_x_2)';

linear_velocity_y_2=linvel_y(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
linear_velocity_y_2=double(linear_velocity_y_2)';

linear_velocity_z_2=linvel_z(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
linear_velocity_z_2=double(linear_velocity_z_2)';

% calculate the linear accelerations (desired linear accelerations)

linear_acceleration_x_2=linacc_x(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
linear_acceleration_x_2=(double(linear_acceleration_x_2))';

linear_acceleration_y_2=linacc_y(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
linear_acceleration_y_2=(double(linear_acceleration_y_2))';

linear_acceleration_z_2=linacc_z(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
linear_acceleration_z_2=(double(linear_acceleration_z_2))';

% calculate angular velocities

angular_velocity_roll_2=angvel_roll(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
angular_velocity_roll_2=(double(angular_velocity_roll_2))';

angular_velocity_pitch_2=angvel_pitch(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
angular_velocity_pitch_2=double(angular_velocity_pitch_2)';

angular_velocity_yaw_2=angvel_yaw(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
angular_velocity_yaw_2=double(angular_velocity_yaw_2)';

% calculate angular accelerations

angular_acceleration_roll_2=angacc_roll(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
angular_acceleration_roll_2=(double(angular_acceleration_roll_2))';

angular_acceleration_pitch_2=angacc_pitch(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
angular_acceleration_pitch_2=(double(angular_acceleration_pitch_2))';

angular_acceleration_yaw_2=angacc_yaw(t_e_2,U_e_2,psi_dot_e_2,gama_e_2,x_0_e_2,y_0_e_2,z_0_e_2,psi_0_e_2);
angular_acceleration_yaw_2=(double(angular_acceleration_yaw_2))';



%% plot the trajectory

figure;

plot3(x1,y1,z1,'LineWidth',2);

hold on;

plot3(x2,y2,z2,'LineWidth',2);

title('Training and Test');

xlabel('X: (meter:m)');
ylabel('Y: (meter:m)');
zlabel('Z: (meter:m)');

legend('Training Trim Trajectory','Test Trim Trajectory');




