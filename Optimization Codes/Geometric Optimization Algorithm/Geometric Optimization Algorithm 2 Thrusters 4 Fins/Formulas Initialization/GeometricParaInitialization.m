%%
%%---------------------------------------------------------------------------------
% Intial geometry variables:

% 1. number of thrusters and fins

% first assume that we have 2 thrusters and 4 fins

n_thrusters=2;

n_fins=4;

% 2. number of trim trajectories

n_trimtraj=7;

% The initialization geometric parameters are located in the 

% 2. initial position and orientation for all thruster should be specified
% at first, spin direction should be optimized at the first step 

% parameters for hull, they will not be changed during the optimization

m_H=25;

r_H=[0;0;0];

d_H=0.25;

l_H=0.74;

% The following parameter must be changed during the optimization phase:

% r_T1 r_T2 

% d_T1 d_T2

% b_T1 b_T2


%% parameters for thrusters:

%% the intiial spin direction

b_T1_init=1;

b_T2_init=-1;



%% the intial orientation of all thrusters are genrated randomly

% 1. Choose a random value between -pi and pi

theta_1_init=-pi+2*pi*rand;

% 2. Choose a random value between -1 and 1

z_1_init=-1+2*rand;

% 3. compute the result 

d_T1_init=[sqrt(1-z_1_init^2)*cos(theta_1_init);sqrt(1-z_1_init^2)*sin(theta_1_init);z_1_init];

% 4. assign the intial value back in to the global variable 

veh.d_T1=d_T1_init;

%%  the intial position of all thrusters are generated randomly

x_T1_init_Cyl=-l_H/2+l_H*rand;

phi_T1_init=-pi+2*pi*rand;

[x_T1_init_Car,y_T1_init,z_T1_init]=CoorTransferCar2Cy(x_T1_init_Cyl,phi_T1_init,d_H/2);

r_T1_init=[x_T1_init_Car;y_T1_init;z_T1_init];

veh.r_T1=[x_T1_init_Car,y_T1_init,z_T1_init]';

% veh.r_T1=[0.00276;-0.1799;0];

m_T1=0.12;



% 1. Choose a random value between -pi and pi

theta_2_init=-pi+2*pi*rand;

% 2. Choose a random value between -1 and 1

z_2_init=-1+2*rand;

% 3. compute the result 

d_T2_init=[sqrt(1-z_2_init^2)*cos(theta_2_init);sqrt(1-z_2_init^2)*sin(theta_2_init);z_2_init];

% 4. assign the intial value back in to the global variable 

veh.d_T2=d_T2_init;

% r_T2=[0.00276;0.1799;0];

x_T2_init_Cyl=-l_H/2+l_H*rand;

phi_T2_init=-pi+2*pi*rand;

[x_T2_init_Car,y_T2_init,z_T2_init]=CoorTransferCar2Cy(x_T2_init_Cyl,phi_T2_init,d_H/2);

r_T2_init=[x_T2_init_Car;y_T2_init;z_T2_init];

m_T2=0.12;

veh.r_T2=[x_T2_init_Car,y_T2_init,z_T2_init]';



%% parameters for fins

% the lift coefficient is constant for all optimization

% Assume the lift coefficient is constant in the following cases because of the small angle of attack assumption 

C_L=0.3;
     
% parameters for fins should be transferred into 

% cylinderical coordinate

% The length and width as well as the mass can also be optimized  

a_F1=0.104;
b_F1=0.25;
m_F1=1.2;


% The following two parameters determine the position of all fins  

% randomly generate within the specified range

x_F1_init=-0.5*(l_H-b_F1)+(l_H-b_F1)*rand;
veh.x_F1=x_F1_init;

gamma_F1_init=-pi+2*pi*rand;
veh.gamma_F1=gamma_F1_init;

r_F1_init=[x_F1_init;0.5*(d_H+a_F1)*sin(gamma_F1_init);0.5*(d_H+a_F1)*cos(gamma_F1_init)];

veh.r_F1=r_F1_init;





% z is positive, it should be in the downside


a_F2=0.104;
b_F2=0.25;
m_F2=1.2;

% The following two parameters determine the postition of all fins 
% and should be generated randomly

x_F2_init=-0.5*(l_H-b_F2)+(l_H-b_F2)*rand;
veh.x_F2=x_F2_init;
gamma_F2_init=-pi+2*pi*rand;
veh.gamma_F2=gamma_F2_init;

r_F2_init=[x_F2_init;0.5*(d_H+a_F2)*sin(gamma_F2_init);0.5*(d_H+a_F2)*cos(gamma_F2_init)];

veh.r_F2=r_F2_init;

% The following two parameters should be generated randomly

a_F3=0.104;
b_F3=0.25;
m_F3=1.2;


% the following parameters should be generated randomly

x_F3_init=-0.5*(l_H-b_F3)+(l_H-b_F3)*rand;

veh.x_F3=x_F3_init;

gamma_F3_init=-pi+2*pi*rand;

veh.gamma_F3=gamma_F3_init;

r_F3_init=[x_F3_init;0.5*(d_H+a_F3)*sin(gamma_F3_init);0.5*(d_H+a_F3)*cos(gamma_F3_init)];

veh.r_F3=r_F3_init;



% z is negative, it shoulb be in the upside

a_F4=0.104;
b_F4=0.25;
m_F4=1.2;     

 
x_F4_init=-0.5*(l_H-b_F4)+(l_H-b_F4)*rand;

veh.x_F4=x_F4_init;
gamma_F4_init=-pi+2*pi*rand;

veh.gamma_F4=gamma_F4_init;
r_F4_init=[x_F4_init;0.5*(d_H+a_F4)*sin(gamma_F4_init);0.5*(d_H+a_F4)*cos(gamma_F4_init)];

veh.r_F4=r_F4_init;




% first we calculate the initial center of mass 

r_G_init=CenterofMass({m_H,r_H},{m_T1,veh.r_T1},{m_T2,veh.r_T2},{m_F1,veh.r_F1},{m_F2,veh.r_F2},{m_F3,veh.r_F3},{m_F4,veh.r_F4});

% assign the initial value into the global variable to update 

veh.G_b=r_G_init;

% now we should compute the inertia matrix 

% Thr rigid body Inertia Matrix should be initialize first, and then in the
% optimization phase, it must be updated 

% the mass equal to the sum of the hull and two fins 

m=m_H+m_T1+m_T2+m_F1+m_F2+m_F3+m_F4;

% The moment of inertia is assumed only to be determined by the hull
% parameter. Thus it is fixed in the geometric optimization phase

I_cg=veh.iota;

% initial value for rigid body inertia matrix

M_RB=RigidBodyInertiaMatrix(m,I_cg,r_G_init);

% Pass this parameter to the global variable veh.Mrb

veh.Mrb=M_RB;







