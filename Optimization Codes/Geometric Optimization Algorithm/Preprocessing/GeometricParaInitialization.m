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

% r_F1 r_F2 r_F3 r_F4 r_T1 r_T2 

% fin parameters

% the lift coefficient is constant for all optimization

C_L=0.3;
     
% parameters for fins should be transferred into 

% cylinderical coordinate

a_F1=0.104;
b_F1=0.25;
m_F1=1.2;
% r_F1=[-0.367;0.05;0];
x_F1=-0.367;
gama_F1=0;
r_F1=[x_F1;0.5*(d_H+a_F1)*sin(gama_F1);0.5*(d_H+a_F1)*cos(gama_F1)];

% r_F1=[-0.3670;0;0.1770]

% z is positive, it should be in the downside


a_F2=0.104;
b_F2=0.25;
m_F2=1.2;
% r_F2=[-0.367;0;0.05];
x_F2=-0.367;
gama_F2=pi/2;
r_F2=[x_F2;0.5*(d_H+a_F2)*sin(gama_F2);0.5*(d_H+a_F2)*cos(gama_F2)];

% r_F2=[-0.3670;0.1770;0]

% y is positive, it should be in the right side

a_F3=0.104;
b_F3=0.25;
m_F3=1.2;
% r_F3=[-0.367;-0.05;0];
x_F3=-0.367;
gama_F3=pi;
r_F3=[x_F3;0.5*(d_H+a_F3)*sin(gama_F3);0.5*(d_H+a_F3)*cos(gama_F3)];

% r_F3=[-0.3670;0;-0.1770];

% z is negative, it shoulb be in the upside

a_F4=0.104;
b_F4=0.25;
m_F4=1.2;     

% r_F4=[-0.367;0;-0.05]; 
x_F4=-0.367;
gama_F4=3*pi/2;
r_F4=[x_F4;0.5*(d_H+a_F4)*sin(gama_F4);0.5*(d_H+a_F4)*cos(gama_F4)];

% r_F4=[-0.3670;-0.1770,0];

% y is negative, it should be in the left side 

% parameters for thrusters:

r_T1=[0.00276;-0.1799;0];
m_T1=0.12;

r_T2=[0.00276;0.1799;0];
m_T2=0.12;

% first we calculate the initial center of mass 

r_G_init=CenterofMass({m_H,r_H},{m_F1,r_F1},{m_F2,r_F2},{m_F3,r_F3},{m_F4,r_F4},{m_T1,r_T1},{m_T1,r_T1});

% now we should compute the inertia matrix 

% Thr rigid body Inertia Matrix should be initialize first, and then in the
% optimization phase, it must be updated 

% the mass equal to the sum of the hull and two fins 

m=m_F1+m_F2+m_F3+m_F4+m_T1+m_T2;

% The moment of inertia is assumed only to be determined by the hull
% parameter. Thus it is fixed in the geometric optimization phase

I_cg=veh.iota;

% initial value for rigid body inertia matrix

M_RB=RigidBodyInertiaMatrix(m,I_cg,r_G_init);

% Pass this parameter to the global variable veh.Mrb

veh.Mrb=M_RB;
