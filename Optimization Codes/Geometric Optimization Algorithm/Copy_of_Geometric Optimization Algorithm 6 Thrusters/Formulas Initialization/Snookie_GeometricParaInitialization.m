%%
%%---------------------------------------------------------------------------------
% Intial geometry variables:

% 1. number of thrusters and fins

% first assume that we have 2 thrusters and 4 fins

n_thrusters=6;

n_fins=0;

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

% r_T1 r_T2 r_T3 r_T4 r_T5 r_T6

% d_T1 d_T2 d_T3 d_T4 d_T5 d_T6

% b_T1 b_T2 b_T3 b_T4 b_T5 b_T6


% parameters for thrusters:

%% the intiial spin direction

b_T1_init=1;

b_T2_init=-1;

b_T3_init=1;

b_T4_init=-1;

b_T5_init=1;

b_T6_init=-1;


%% the intial orientation of all thrusters are genrated randomly

d_T1_init=[1;0;0];

% 4. assign the intial value back in to the global variable 

veh.d_T1=d_T1_init;

%%  the intial position of all thrusters are generated randomly

r_T1_init=[0.00276;-0.1799;0];

veh.r_T1=r_T1_init;

% veh.r_T1=[0.00276;-0.1799;0];

m_T1=0.12;


% 3. compute the result 

d_T2_init=[1;0;0];

% 4. assign the intial value back in to the global variable 

veh.d_T2=d_T2_init;

r_T2_init=[0.00276;0.1799;0];

m_T2=0.12;

veh.r_T2=r_T2_init;


% left bottom thruster location

% r_T3=[-0.24078;-0.12327;0.12131];

m_T3=0.12;


% 3. compute the result 

d_T3_init=[1;0;0];

% 4. assign the intial value back into the global variable 

veh.d_T3=d_T3_init;


r_T3_init=[-0.24078;-0.12327;0.12131];

veh.r_T3=r_T3_init;



% left top thruster location

% r_T4=[-0.24078;-0.12327;-0.12131];

m_T4=0.12;

d_T4_init=[1;0;0];

% 4. assign the initial value back into the global variable

veh.d_T4=d_T4_init;

r_T4_init=[-0.24078;-0.12327;-0.12131];

veh.r_T4=r_T4_init;


% right bottom thruster location1

% r_T5=[-0.24078;0.12327;0.12131];

m_T5=0.12;


d_T5_init=[0;0;-1];

% 4. assign the initial value back into the global variable

veh.d_T5=d_T5_init;

r_T5_init=[-0.24078;0.12327;0.12131];

veh.r_T5=r_T5_init;



% right top thruster location 

% r_T6=[-0.24078;0.12327;-0.12131];

m_T6=0.12;

% 3. compute the result 

d_T6_init=[0;0;-1];

% 4. assign the initial value back into the global variable

veh.d_T6=d_T6_init;

r_T6_init=[-0.24078;0.12327;-0.12131];

veh.r_T6=r_T6_init;


% first we calculate the initial center of mass 

r_G_init=CenterofMass({m_H,r_H},{m_T1,veh.r_T1},{m_T2,veh.r_T2},{m_T3,veh.r_T3},{m_T4,veh.r_T4},{m_T5,veh.r_T5},{m_T6,veh.r_T6});

% assign the initial value into the global variable to update 

veh.G_b=r_G_init;

% now we should compute the inertia matrix 

% Thr rigid body Inertia Matrix should be initialize first, and then in the
% optimization phase, it must be updated 

% the mass equal to the sum of the hull and two fins 

m=m_H+m_T1+m_T2+m_T3+m_T4+m_T5+m_T6;

% The moment of inertia is assumed only to be determined by the hull
% parameter. Thus it is fixed in the geometric optimization phase

I_cg=veh.iota;

% initial value for rigid body inertia matrix

M_RB=RigidBodyInertiaMatrix(m,I_cg,r_G_init);

% Pass this parameter to the global variable veh.Mrb

veh.Mrb=M_RB;






