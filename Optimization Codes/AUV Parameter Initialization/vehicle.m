function veh = vehicle()
%% in this function, the configuration and geometric parameter will be initialized 


veh.g=9.81; % gravitational acceleration

veh.rho=1033; % surrounding water density


%% The geometric center must be updated during the optimization phase 

veh.G_b=[0;0;0]; % center of gravity wrt body frame {b}

% mass(kg) 

veh.m=32.23;
 

%% our assumptiom: The center of buoyancy is always located in the geometric center of the hull 

veh.B_b=[0;0;0]; % center of buoyncy in body frame {b}
 
% our assumption: The moment of inertia 
% moment of inertia(kg*m^2)

veh.iota=[0.23,0,0;0,1.68,0;0,0,1.70];

%% rigid body mass matrix 

% This matrix is changed by during the geometric optimization phase, since
% the geometric center of the robot is influenced by the different
% configurations of actuators

veh.Mrb=[veh.m*eye(3),-veh.m*vp(veh.G_b);veh.m*vp(veh.G_b),veh.iota-veh.m*vp(veh.G_b)*vp(veh.G_b)];


%% added mass matrix 

% add mass matrix is only determined by the hull parameter
% determined in the first phase 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m_H1=5.922;
m_H2=23.573;
iota_H=1.822;
iota_F1=16.16/1000;
iota_F2=0.718/100;
rho_F=-10;
m_F=5;

% The added mass matrix is also determined in the first optimization phase

veh.Ma=[m_H1,0,0,0,0,0;
    0,m_H2+m_F,0,0,0,-m_F*rho_F;
    0,0,m_H2+m_F,0,m_F*rho_F,0;
    0,0,0,iota_F1,0,0;
    0,0,m_F*rho_F,0,m_F*(rho_F)*rho_F+iota_F2,0;
    0,-m_F*rho_F,0,0,0,m_F*(rho_F)*rho_F+iota_H+iota_F2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% initialize the configuration vector of all thrusters 
% for each thruster three parameters: the direction vector d_T: unit sphere
% three dimensional vector with unit length 
% the location vector r_T: three dimensional vector in body frame {b}
% the spin direction b_T:{-1,1}
% the first group: horizontal thrusters

veh.d_T1=[1;0;0];
veh.d_T2=[1;0;0];
veh.d_T3=[1;0;0];
veh.d_T4=[1;0;0];



% if b_Ti is equal to 1 positive rotation in the counterclockwise direction
% if b_Ti is equal to -1 negative rotation in the clockwise direction

veh.b_T1=1;
veh.b_T2=-1;
veh.b_T3=1;
veh.b_T4=-1;

veh.r_T1=[-0.25;0.1768;0.1768];
veh.r_T2=[-0.25;0.1768;-0.1768];
veh.r_T3=[-0.25;-0.1768;0.1768];
veh.r_T4=[-0.25;-0.1768;-0.1768];

veh.lambda1=0.5;
veh.lambda2=0.5;
veh.lambda3=0.5;
veh.lambda4=0.5;

veh.d_T5=[0;0;-1];
veh.d_T6=[0;0;-1];

veh.b_T5=1;
veh.b_T6=-1;

veh.r_T5=[0;0.2;0];
veh.r_T6=[0;-0.2;0];

veh.lambda5=0.5;
veh.lambda6=0.5;

%% The torque-thrust relationship coefficient

% The torque is proportional to the thrust when the angular velocity is
% constant

veh.lambda=0.5;

%% inverse total matrix

veh.iM=inv(veh.Mrb+veh.Ma);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% fins configuration

veh.sw=0.2; % fin surface(m^2)
veh.cw=0.4; % fin width(m)
veh.bw=0.5; % fin length(m)
veh.st=0.104*0.25*0.5;

% fin middle point positions wrt body frame {b}
veh.P1_b=[-0.367;0.05;0];
veh.P2_b=[-0.367;0;0.05];
veh.P3_b=[-0.367;-0.05;0];
veh.P4_b=[-0.367;0;-0.05];
veh.P5_b=[0;0.05;0];
veh.P6_b=[0;0;0.05];
veh.P7_b=[0;-0.05;0];
veh.P8_b=[0;0;-0.05];



veh.D=DragHull(0.2,0.6);

% The total gravity and total buoyancy

veh.G=veh.m*veh.g;

% since we want to keep buoyancy neutral for the first optimization
% so the buoyancy is always euqal to gravity 

veh.B=veh.G;



end

