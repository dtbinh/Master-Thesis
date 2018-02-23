%% Yongyu Chen 29.04.2017
%% all the quatities are parameterized as time-dependent function
%% parameterize the desired trajectory vector as three dimensional vector in space (x(t),y(t),z(t))
syms t;

% yaw rate
syms psi_dot;

% flight path angle
syms gama;

% linear speed

syms U;

% initial position in inertial world frame
syms x_0;
syms y_0;
syms z_0;

% initial yaw angle
syms psi_0;

%% use aforedefined parameters to parameterize trimming trajectory
x_t=U/psi_dot*cos(gama)*(sin(psi_dot*t+psi_0)-sin(psi_0))+x_0;
y_t=-U/psi_dot*cos(gama)*(cos(psi_dot*t+psi_0)-cos(psi_0))+y_0;
z_t=-U*sin(gama)*t+z_0;

%% define symbolic function for the trimming trajectory
trajx=symfun(x_t,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
trajy=symfun(y_t,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
trajz=symfun(z_t,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% trajectory position eta1:(x,y,z) in the world frame {i}
trajpos=[x_t y_t z_t];


%% velocity of trajectory: eta1_dot(x_dot,y_dot,z_dot)
trajvel=diff(trajpos,t);

%% function to calculate the tangent vector
trajtan=simplify(unitvec(trajvel));
tanvec=symfun(trajtan',[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% acceleration of trajectory: eta1_dot_dot(x_dot_dot,y_dot_dot,z_dot_dot)
trajacc=diff(trajvel,t);

%% function to calculate the binomal vector
trajcross=cross(trajvel,trajacc);
trajbin=simplify(unitvec(trajcross));
binvec=symfun(trajbin',[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% function to calculate the norm vector
trajnorm=simplify(cross(trajbin,trajtan));
normvec=symfun(trajnorm',[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% Rotation Matrix calculation from Frenet Frame(body frame) to inertial world frame {i}
R_FS_U=[trajtan',trajnorm',trajbin'];
RotMatrix=symfun(R_FS_U,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% function to calculate the theta vector
theta_d=-asin(trajtan(3));
trajtheta=symfun(theta_d,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% function to calculate the phi vector
phi_d=-asin(sec(theta_d)*trajnorm(3));
trajphi=symfun(phi_d,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% function to calculte the psi vector
% psi_d=atan2(trajtan(2),trajtan(1));

psi_d=psi_0+psi_dot*t;
trajpsi=symfun(psi_d,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% define the generalised postion vector in world frame {i}
eta1=trajpos;
p_dot=trajvel;

% define a function to calculate the derivative of the trajectory
% parameterized with respect to time t

position_dot=symfun(p_dot,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% generalised orientation vector in world frame {i}
eta2=[phi_d,theta_d,psi_d];

%% calculate the eta2_dot(phi_dot,theta_dot,psi_dot)
% velocity of Euler angles in inertial world frame {i} is different from robot anglur velocity
% expressed in body frame {b}, they can be transformed from each other
% according to the Jacobian matrix J22

eta2_dot=diff(eta2,t);

% explicit Euler angle velocity symbolic function
phi_dot_e=symfun(eta2_dot(1),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
theta_dot_e=symfun(eta2_dot(2),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
psi_dot_e=symfun(eta2_dot(3),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% Jacobian matrix calculation according to Euler angles
[J,J11,J22]=eulerang(phi_d,theta_d,psi_d);

%% calculate the equilibrium point of velocities
v_d=R_FS_U\p_dot';

a_d_x=diff(v_d(1),t);
a_d_y=diff(v_d(2),t);
a_d_z=diff(v_d(3),t);

% The function linvel is used to calculate the linear velocity vector parameterized with respect to t
linvel_x=symfun(v_d(1),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
linvel_y=symfun(v_d(2),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
linvel_z=symfun(v_d(3),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

% The function linacc for calculation of linear acceleration vector parameterized with respect to t
linacc_x=symfun(a_d_x,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
linacc_y=symfun(a_d_y,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
linacc_z=symfun(a_d_z,[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);


w_d=J22\eta2_dot';
alpha_d=diff(w_d,t);
% The function angvel is used to calculate the angular velocity vector parameterized with
% repect to t
angvel_roll=symfun(w_d(1),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
angvel_pitch=symfun(w_d(2),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
angvel_yaw=symfun(w_d(3),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);

%% calculation of angular acceleration in body frame
% first calculate the angular acceleration in world frame
angacc_roll=symfun(alpha_d(1),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
angacc_pitch=symfun(alpha_d(2),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);
angacc_yaw=symfun(alpha_d(3),[t,U,psi_dot,gama,x_0,y_0,z_0,psi_0]);