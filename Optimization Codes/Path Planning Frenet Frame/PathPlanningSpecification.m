%% initialize the trajectory (x,y,z) (desired locations)
% the starting time and time interval are used to avoid the sigularity
% singularity must be deeply studied 
%% parameterize the lasting time for each trimming trajectory segment
% 1. Trimming Trajectory
t_1_last=40;
t_2_last=20;
t_3_last=20;
t_4_last=40;
t_5_last=20;
t_6_last=20;
t_7_last=40;
% 2. Trimming Trajectory
% explicit values of t: discrete values
t_e_1=0:1/20:t_1_last; % t explicit 
t_e_2=0:1/20:t_2_last;
t_e_3=0:1/20:t_3_last;
t_e_4=0:1/20:t_4_last;
t_e_5=0:1/20:t_5_last;
t_e_6=0:1/20:t_6_last;
t_e_7=0:1/20:t_7_last;
%% 1.Trimming Trajectory
% trajectory paramters:

U_e_1=zeros(1,length(t_e_1));
U_e_1(1,:)=2;

% yaw rate is nearly equal to zero: a very small value
psi_dot_e_1=zeros(1,length(t_e_1));
psi_dot_e_1(1,:)=0.000001;

gama_e_1=zeros(1,length(t_e_1));
gama_e_1(1,:)=0;

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
U_e_2(1,:)=2;

psi_dot_e_2=zeros(1,length(t_e_2));
psi_dot_e_2(1,:)=0.0803;

gama_e_2=zeros(1,length(t_e_2));
gama_e_2(1,:)=0;

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



%% 3.Trimming Trajectory

% trajectory paramters:

U_e_3=zeros(1,length(t_e_3));
U_e_3(1,:)=2;

psi_dot_e_3=zeros(1,length(t_e_3));
psi_dot_e_3(1,:)=0.0803;

gama_e_3=zeros(1,length(t_e_3));
gama_e_3(1,:)=19.5;

% initial condition:

x_0_e_3=zeros(1,length(t_e_3));
x_0_e_3(1,:)=x2(end);

y_0_e_3=zeros(1,length(t_e_3));
y_0_e_3(1,:)=y2(end);

z_0_e_3=zeros(1,length(t_e_3));
z_0_e_2(1,:)=z2(end);

psi_0_e_3=zeros(1,length(t_e_3));
psi_0_e_3(1,:)=psi_0_e_2(1,1)+psi_dot_e_2(1,1)*t_e_2(end);

% calculate the position of the second trimming trajtory

x3=trajx(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
x3=double(x3);
y3=trajy(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
y3=double(y3);
z3=trajz(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
z3=double(z3);
% calculate the orientation of the second trimming trajectory

roll_angle_3=trajphi(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
roll_angle_3=double(roll_angle_3);

pitch_angle_3=trajtheta(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
pitch_angle_3=double(pitch_angle_3);

yaw_angle_3=trajpsi(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
yaw_angle_3=double(yaw_angle_3);

% calculate the linear velocities (desired linear velocities)

linear_velocity_x_3=linvel_x(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
linear_velocity_x_3=double(linear_velocity_x_3)';

linear_velocity_y_3=linvel_y(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
linear_velocity_y_3=double(linear_velocity_y_3)';

linear_velocity_z_3=linvel_z(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
linear_velocity_z_3=double(linear_velocity_z_3)';

% calculate the linear accelerations (desired linear accelerations)

linear_acceleration_x_3=linacc_x(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
linear_acceleration_x_3=(double(linear_acceleration_x_3))';

linear_acceleration_y_3=linacc_y(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
linear_acceleration_y_3=(double(linear_acceleration_y_3))';

linear_acceleration_z_3=linacc_z(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
linear_acceleration_z_3=(double(linear_acceleration_z_3))';

% calculate angular velocities

angular_velocity_roll_3=angvel_roll(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
angular_velocity_roll_3=(double(angular_velocity_roll_3))';

angular_velocity_pitch_3=angvel_pitch(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
angular_velocity_pitch_3=double(angular_velocity_pitch_3)';

angular_velocity_yaw_3=angvel_yaw(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
angular_velocity_yaw_3=double(angular_velocity_yaw_3)';

% calculate angular accelerations

angular_acceleration_roll_3=angacc_roll(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
angular_acceleration_roll_3=(double(angular_acceleration_roll_3))';

angular_acceleration_pitch_3=angacc_pitch(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
angular_acceleration_pitch_3=(double(angular_acceleration_pitch_3))';

angular_acceleration_yaw_3=angacc_yaw(t_e_3,U_e_3,psi_dot_e_3,gama_e_3,x_0_e_3,y_0_e_3,z_0_e_3,psi_0_e_3);
angular_acceleration_yaw_3=(double(angular_acceleration_yaw_3))';

%% 4.Trimming Trajectory

% trajectory parameters:

U_e_4=zeros(1,length(t_e_4));
U_e_4(1,:)=2;

gama_e_4=zeros(1,length(t_e_4));
gama_e_4(1,:)=19.5;

psi_dot_e_4=zeros(1,length(t_e_4));
psi_dot_e_4(1,:)=0.0000001; % nearly a line 

% initial condition:

x_0_e_4=zeros(1,length(t_e_4));
x_0_e_4(1,:)=x3(end);

y_0_e_4=zeros(1,length(t_e_4));
y_0_e_4(1,:)=y3(end);

z_0_e_4=zeros(1,length(t_e_4));
z_0_e_4(1,:)=z3(end);

psi_0_e_4=zeros(1,length(t_e_4));
psi_0_e_4(1,:)=psi_0_e_3(1,1)+psi_dot_e_3(1,1)*t_e_3(end);

% choose zero yaw rate parameter function

x4=trajx(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
x4=double(x4);
y4=trajy(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
y4=double(y4);
z4=trajz(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
z4=double(z4);
% calculate the orientation of the second trimming trajectory

roll_angle_4=trajphi(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
roll_angle_4=double(roll_angle_4);

pitch_angle_4=trajtheta(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
pitch_angle_4=double(pitch_angle_4);

yaw_angle_4=trajpsi(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
yaw_angle_4=double(yaw_angle_4);

% calculate the linear velocities (desired linear velocities)

linear_velocity_x_4=linvel_x(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
linear_velocity_x_4=double(linear_velocity_x_4)';

linear_velocity_y_4=linvel_y(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
linear_velocity_y_4=double(linear_velocity_y_4)';

linear_velocity_z_4=linvel_z(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
linear_velocity_z_4=double(linear_velocity_z_4)';

% calculate the linear accelerations (desired linear accelerations)

linear_acceleration_x_4=linacc_x(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
linear_acceleration_x_4=(double(linear_acceleration_x_4))';

linear_acceleration_y_4=linacc_y(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
linear_acceleration_y_4=(double(linear_acceleration_y_4))';

linear_acceleration_z_4=linacc_z(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
linear_acceleration_z_4=(double(linear_acceleration_z_4))';

% calculate angular velocities

angular_velocity_roll_4=angvel_roll(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
angular_velocity_roll_4=(double(angular_velocity_roll_4))';

angular_velocity_pitch_4=angvel_pitch(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
angular_velocity_pitch_4=double(angular_velocity_pitch_4)';

angular_velocity_yaw_4=angvel_yaw(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
angular_velocity_yaw_4=double(angular_velocity_yaw_4)';

% calculate angular accelerations

angular_acceleration_roll_4=angacc_roll(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
angular_acceleration_roll_4=(double(angular_acceleration_roll_4))';

angular_acceleration_pitch_4=angacc_pitch(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
angular_acceleration_pitch_4=(double(angular_acceleration_pitch_4))';

angular_acceleration_yaw_4=angacc_yaw(t_e_4,U_e_4,psi_dot_e_4,gama_e_4,x_0_e_4,y_0_e_4,z_0_e_4,psi_0_e_4);
angular_acceleration_yaw_4=(double(angular_acceleration_yaw_4))';

%% 5.Trimming Trajectory

% trajectory parameters:

U_e_5=zeros(1,length(t_e_5));
U_e_5(1,:)=2;

psi_dot_e_5=zeros(1,length(t_e_5));
psi_dot_e_5(1,:)=-0.0803;

gama_e_5=zeros(1,length(t_e_5));
gama_e_5(1,:)=19.5;

% initial conditions:

x_0_e_5=zeros(1,length(t_e_5));
x_0_e_5(1,:)=x4(end);

y_0_e_5=zeros(1,length(t_e_5));
y_0_e_5(1,:)=y4(end);

z_0_e_5=zeros(1,length(t_e_5));
z_0_e_5(1,:)=z4(end);

psi_0_e_5=zeros(1,length(t_e_5));
psi_0_e_5(1,:)=psi_0_e_4(1,1)+psi_dot_e_4(1,1)*t_e_4(end);

% choose constant yaw rate parameter function

x5=trajx(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
x5=double(x5);
y5=trajy(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
y5=double(y5);
z5=trajz(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
z5=double(z5);

% calculate the orientation of the second trimming trajectory

roll_angle_5=trajphi(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
roll_angle_5=double(roll_angle_5);

pitch_angle_5=trajtheta(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
pitch_angle_5=double(pitch_angle_5);

yaw_angle_5=trajpsi(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
yaw_angle_5=double(yaw_angle_5);

% calculate the linear velocities (desired linear velocities)

linear_velocity_x_5=linvel_x(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
linear_velocity_x_5=double(linear_velocity_x_5)';

linear_velocity_y_5=linvel_y(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
linear_velocity_y_5=double(linear_velocity_y_5)';

linear_velocity_z_5=linvel_z(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
linear_velocity_z_5=double(linear_velocity_z_5)';

% calculate the linear accelerations (desired linear accelerations)

linear_acceleration_x_5=linacc_x(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
linear_acceleration_x_5=(double(linear_acceleration_x_5))';

linear_acceleration_y_5=linacc_y(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
linear_acceleration_y_5=(double(linear_acceleration_y_5))';

linear_acceleration_z_5=linacc_z(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
linear_acceleration_z_5=(double(linear_acceleration_z_5))';

% calculate angular velocities

angular_velocity_roll_5=angvel_roll(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
angular_velocity_roll_5=(double(angular_velocity_roll_5))';

angular_velocity_pitch_5=angvel_pitch(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
angular_velocity_pitch_5=double(angular_velocity_pitch_5)';

angular_velocity_yaw_5=angvel_yaw(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
angular_velocity_yaw_5=double(angular_velocity_yaw_5)';

% calculate angular accelerations

angular_acceleration_roll_5=angacc_roll(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
angular_acceleration_roll_5=(double(angular_acceleration_roll_5))';

angular_acceleration_pitch_5=angacc_pitch(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
angular_acceleration_pitch_5=(double(angular_acceleration_pitch_5))';

angular_acceleration_yaw_5=angacc_yaw(t_e_5,U_e_5,psi_dot_e_5,gama_e_5,x_0_e_5,y_0_e_5,z_0_e_5,psi_0_e_5);
angular_acceleration_yaw_5=(double(angular_acceleration_yaw_5))';


%% 6.Trimming Trajectory

% trajectory parameters:

psi_dot_e_6=zeros(1,length(t_e_6));
psi_dot_e_6(1,:)=-0.0803;

gama_e_6=zeros(1,length(t_e_6));
gama_e_6(1,:)=0;

U_e_6=zeros(1,length(t_e_6));
U_e_6(1,:)=2;

% initial conditions:

x_0_e_6=zeros(1,length(t_e_6));
x_0_e_6(1,:)=x5(end);

y_0_e_6=zeros(1,length(t_e_6));
y_0_e_6(1,:)=y5(end);

z_0_e_6=zeros(1,length(t_e_6));
z_0_e_6(1,:)=z5(end);

psi_0_e_6=zeros(1,length(t_e_6));
psi_0_e_6(1,:)=psi_0_e_5(1,1)+psi_dot_e_5(1,1)*t_e_5(end);

% choose zero yaw rate parameter function

x6=trajx(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
x6=double(x6);
y6=trajy(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
y6=double(y6);
z6=trajz(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
z6=double(z6);

% calculate the orientation of the second trimming trajectory

roll_angle_6=trajphi(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
roll_angle_6=double(roll_angle_6);

pitch_angle_6=trajtheta(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
pitch_angle_6=double(pitch_angle_6);

yaw_angle_6=trajpsi(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
yaw_angle_6=double(yaw_angle_6);

% calculate the linear velocities (desired linear velocities)

linear_velocity_x_6=linvel_x(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
linear_velocity_x_6=double(linear_velocity_x_6)';

linear_velocity_y_6=linvel_y(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
linear_velocity_y_6=double(linear_velocity_y_6)';

linear_velocity_z_6=linvel_z(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
linear_velocity_z_6=double(linear_velocity_z_6)';

% calculate the linear accelerations (desired linear accelerations)

linear_acceleration_x_6=linacc_x(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
linear_acceleration_x_6=(double(linear_acceleration_x_6))';

linear_acceleration_y_6=linacc_y(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
linear_acceleration_y_6=(double(linear_acceleration_y_6))';

linear_acceleration_z_6=linacc_z(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
linear_acceleration_z_6=(double(linear_acceleration_z_6))';

% calculate angular velocities

angular_velocity_roll_6=angvel_roll(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
angular_velocity_roll_6=(double(angular_velocity_roll_6))';

angular_velocity_pitch_6=angvel_pitch(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
angular_velocity_pitch_6=double(angular_velocity_pitch_6)';

angular_velocity_yaw_6=angvel_yaw(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
angular_velocity_yaw_6=double(angular_velocity_yaw_6)';

% calculate angular accelerations

angular_acceleration_roll_6=angacc_roll(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
angular_acceleration_roll_6=(double(angular_acceleration_roll_6))';

angular_acceleration_pitch_6=angacc_pitch(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
angular_acceleration_pitch_6=(double(angular_acceleration_pitch_6))';

angular_acceleration_yaw_6=angacc_yaw(t_e_6,U_e_6,psi_dot_e_6,gama_e_6,x_0_e_6,y_0_e_6,z_0_e_6,psi_0_e_6);
angular_acceleration_yaw_6=(double(angular_acceleration_yaw_6))';

%% 7.Trimming Trajectory

% trajectory parameters:

psi_dot_e_7=zeros(1,length(t_e_7));
psi_dot_e_7(1,:)=0.0000001; % nearly a line 

gama_e_7=zeros(1,length(t_e_7));
gama_e_7(1,:)=0;

U_e_7=zeros(1,length(t_e_7));
U_e_7(1,:)=2;

% initial condition:

x_0_e_7=zeros(1,length(t_e_7));
x_0_e_7(1,:)=x6(end);

y_0_e_7=zeros(1,length(t_e_7));
y_0_e_7(1,:)=y6(end);

z_0_e_7=zeros(1,length(t_e_7));
z_0_e_7(1,:)=z6(end);

psi_0_e_7=zeros(1,length(t_e_7));
psi_0_e_7(1,:)=psi_0_e_6(1,1)+psi_dot_e_5(1,1)*t_e_5(end);

% choose zero yaw rate parameter function

x7=trajx(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
x7=double(x7);
y7=trajy(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
y7=double(y7);
z7=trajz(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
z7=double(z7);
% calculate the orientation of the second trimming trajectory

roll_angle_7=trajphi(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
roll_angle_7=double(roll_angle_7);

pitch_angle_7=trajtheta(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
pitch_angle_7=double(pitch_angle_7);

yaw_angle_7=trajpsi(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
yaw_angle_7=double(yaw_angle_7);

% calculate the linear velocities (desired linear velocities)

linear_velocity_x_7=linvel_x(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
linear_velocity_x_7=double(linear_velocity_x_7)';

linear_velocity_y_7=linvel_y(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
linear_velocity_y_7=double(linear_velocity_y_7)';

linear_velocity_z_7=linvel_z(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
linear_velocity_z_7=double(linear_velocity_z_7)';

% calculate the linear accelerations (desired linear accelerations)

linear_acceleration_x_7=linacc_x(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
linear_acceleration_x_7=(double(linear_acceleration_x_7))';

linear_acceleration_y_7=linacc_y(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
linear_acceleration_y_7=(double(linear_acceleration_y_7))';

linear_acceleration_z_7=linacc_z(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
linear_acceleration_z_7=(double(linear_acceleration_z_7))';

% calculate angular velocities

angular_velocity_roll_7=angvel_roll(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
angular_velocity_roll_7=(double(angular_velocity_roll_7))';

angular_velocity_pitch_7=angvel_pitch(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
angular_velocity_pitch_7=double(angular_velocity_pitch_7)';

angular_velocity_yaw_7=angvel_yaw(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
angular_velocity_yaw_7=double(angular_velocity_yaw_7)';

% calculate angular accelerations

angular_acceleration_roll_7=angacc_roll(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
angular_acceleration_roll_7=(double(angular_acceleration_roll_7))';

angular_acceleration_pitch_7=angacc_pitch(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
angular_acceleration_pitch_7=(double(angular_acceleration_pitch_7))';

angular_acceleration_yaw_7=angacc_yaw(t_e_7,U_e_7,psi_dot_e_7,gama_e_7,x_0_e_7,y_0_e_7,z_0_e_7,psi_0_e_7);
angular_acceleration_yaw_7=(double(angular_acceleration_yaw_7))';



% %% the function trajectory2 is used to visulize the desired trajectory(location and orientation) of robot in world frame   
% trajectory2(x1,y1,z1,pitch_angle_1,roll_angle_1,yaw_angle_1,1,15,jet);
