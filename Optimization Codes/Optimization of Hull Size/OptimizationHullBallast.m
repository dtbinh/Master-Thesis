%% The robot is assumed to be a cylinder, two decision variable should be determined by mixed integer nonlinear optimization

% specify the minimal radius and the minimal the minimal length of the
% robot hull
% 
% specify some constants

% budget constraint: $
budget=5000;

% cost for batteries per unit
cost_bat=249;

% cost for cylindric enclosure per m^3


% cost for end caps



% gravity acceleration
g=9.81;

% density of the ballast, should be much bigger than 
rho_ballast=10000;

% density of the robot hull, approximately equal to the water density
rho_h=1050; 

% density of the fluid
rho_f=1033;

% air density
rho_air=1.225;

% thickness of the hull, 5 mm
thk_h=0.005;

% basic data for the battery
% battery is also approximated as a cylinder

% length of the battery
l_bat=0.141;

% radius of the battery
r_bat=0.075/2;

% weight of the battery
m_bat=1.152;

% volume of the battery
V_bat=pi*r_bat^2*l_bat;

% the length of the robot hull shoulb at least contain one battery which means the diameter of the hull should at least 
% be half the battery length 
% some space should be reserved, we define the a constant volume for

% including eg.: IMU unit and all other devices, model it as a cylinder 
% constant length
l_constant=0.3;

% constant radius
% in order to include the all the basic devices, we set the minimum radius
% for the underwater robot
r_constant=0.15;

% constant volume
V_constant=pi*r_constant^2*l_constant;

% constant weight
% 5kg 
W_constant=5;

% minimal radius
% radius hull lower bound
% r_h_l=l_bat/2;

% minimal length 
% length hull lower bound
% l_h_l=0.6;

%% tuning the weight coefficients

w_energy=0.005; % weight for increasing the hull length, to 
w_bn=10; % weight buoyancy neutral
w_ballast=50; % weight for ballast 
w_l=0.0001; % weight for minimizing the length, because of economical reasons  
w_r=0.0001; % weigth for miminizing the radius because of economical reasons

%% formulate the nonlinear programming problem with different solvers 

% the hull of the robot is approximated as a cylinder,
% The decision variable is defined as d_H={r_H,l_H}, and the amount of the
% ballast material

% specifications: 1. we want to minimize the surge drag, thus we want to minimize the r_H 
%                 2. we want to enhance the total energy storage, hence we want
%                    to maximize the length l_H, l_H should be formulated as
%                    the multiple of battery length
%                 3. buoyancy neutral, is used to modify the radius r_H and l_H
%                    to keep the buoyancy neutral property
%                 4. we want to save cost, thus, both r_H and l_H should be
%                    small
%             
%   
% 
% x(1) is the radius of the hull r_H
% x(2) is the length of the hull: formulate as integers: multiple of
% battery length, battery length is 0.141 m("docs.bluerobotics.com/batteries")
% then the length of the hull is 0.141*l_H
% x(3) is the volume of the ballast material


% Calculatation of the buoyancy

% buoyancy
% B=rho_f*g*(pi*x(1)^2*x(2)*(r_bat*2));

% battery weight
% W1=x(2)*m_bat*g;

% end caps weight
% W2=2*pi*x(1)^2*thk_h*rho_h*g;

% profile surface weight
% W3=pi*(x(1)^2-(x(1)-thk_h)^2)*x(2)*(r_bat*2)*rho_h*g;

% air weight 
% W4=(pi*(x(1)-thk_h)^2*x(2)*l_bat-x(2)*V_bat)*rho_air*g;

% ballast weight 
% W5=rho_ballast*x(3)*g

fun=@(x)-w_energy*x(2)+...
+w_bn*(rho_f*g*(pi*x(1)^2*x(2)*(r_bat*2))...
-x(2)*m_bat*g...
-2*pi*x(1)^2*thk_h*rho_h*g...
-pi*(x(1)^2-(x(1)-thk_h)^2)*x(2)*(r_bat*2)*rho_h*g...
-(pi*(x(1)-thk_h)^2*x(2)*l_bat-x(2)*V_bat-x(3))*rho_air*g...
-rho_ballast*x(3)*g...
)^2....
+w_ballast*x(3)^2+w_l*x(1)+w_r*x(2);


% starting point for searching minimum
% The second for battery number is quite important, because the local
% optimum is always located near this number 
% 

x0=[0 8 0]; 

% A and b specify the inequality constaints 
A=[]; 
b=[];

% Aeq and beq specify the equality constraints
Aeq=[]; 
beq=[];

% Bounds

% lower bound for decision variables
% the hull should contain at least one battery

lb=[r_constant 1 0]; 

% upper bound for decision variables
ub=[inf inf inf]; 

% This function returns the nonlinear constraints

% budget constraint to ensure the uniqueness of the solution 

nlcon=@(x)x(2)*l_bat*110+x(2)*cost_bat+pi*x(1)^2*3;
nlrhs=2000;
nle = -1; % -1 for <=, 0 for ==, +1 >=  

% Integer Constraints
% The following integer specificaiton means that the radius 
xtype='CIC';

% Options
opts = optiset('solver','nomad','display','iter');

% Create OPTI Object
Opt = opti('fun',fun,'nlmix',nlcon,nlrhs,nle,'ineq',A,b,'bounds',lb,ub,...
           'xtype',xtype);

% Solve the MINLP problem
[x,fval,exitflag,info] = solve(Opt,x0);


sprintf('The radius of the robot r_H: %d m',x(1))
sprintf('The length of the robot l_H: %d m',x(2)*l_bat)
sprintf('The volume of the used ballast material V_ballast: %d m^3',x(3))

%% evaluation of the cost function 

% for construction of the energy consumption 

% sprintf('The total value of the cost function: %d',fval)
% f1=p*((rho_f*g*pi*x(1)^2*x(2)*l_bat)-(pi*x(1)^2*x(2)+4/3*pi*x(1)^3-V_constant-x(3))*rho_air*g-(2*pi*x(1)*x(2)+4*pi*x(1)^2)*t_h*rho_h*g-m_constant*g-x(3)*rho_ballast*g)^2
% f2=q*(x(3))^2
% f3=r*(x(1)-0.12)^20
% f4=w*(x(2)-0.42)^2

%% analysis of the buoyancy and gravity

% Buoyancy Calculation 
B=rho_f*g*(pi*x(1)^2*x(2)*(r_bat*2));
sprintf('The buoyancy is equal to: %d N',B)

% Battery Weight
W_bat=x(2)*m_bat*g;
sprintf('The battery weight is equal to: %d N',W_bat)

% end caps weight
W_caps=2*pi*x(1)^2*thk_h*rho_h*g;
sprintf('The weight of end caps is equal to: %d N',W_caps)

% profile surface weight
W_profile=pi*(x(1)^2-(x(1)-thk_h)^2)*x(2)*(r_bat*2)*rho_h*g;
sprintf('The weight of profile surface is equal to: %d N',W_profile)

% ballast weight 
W_ballast=rho_ballast*x(3)*g;
sprintf('The weight of ballast material is equal to: %d N',W_ballast)

BuoyancyGravityDiffer=B-W_bat-W_caps-W_profile-W_ballast;
sprintf('The difference between buoyancy and gravity is: %d N',BuoyancyGravityDiffer)