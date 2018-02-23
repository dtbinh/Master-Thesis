%% The fin configuration: determined by two variables 
% Assumption: slow manoeuring and small drift angle, the angle of attack

% first we determine the geometric configuration of the fins
% 1.the rotation along the x axia of the body frame, i.e. rotate along the unit vector [1;0;0] 
% 2. the position of the fins: 
% The input should be the mechanical angle of rotation delta

% The fin configuration matrix is time-variant or more precisely
% dynamical state-variant 

% velocities: dynamic states x_dyn
syms u_f v_f w_f p_f q_f r_f

% postion of the fin is characterized as r_f:[r_f_x;r_f_y;r_f_z], the only
% geometrical decision variable 
syms r_f_x r_f_y r_f_z

% orientation of the fin: rotation about unit x-vector 

% syms a_f 
% comment: a_f can be removed, if we make the assumption that the fins
% stick to robot surface

% the input should be 

syms delta

syms rho % the surrounding water density should be a constant

% specify the span length sl and chord length cl
syms sl cl

% position vector should be 
r_f=[r_f_x;r_f_y;r_f_z];

% calculate orientation according to the position of the robot
a_f=-atan2(r_f_y,r_f_z);

% we first calculate velocity of the fin from the robot velocity

v_f_b=VelocityTransformation([u_f;v_f;w_f],[p_f;q_f;r_f],r_f);

% Rotation matrix from flow frame to body frame 
 
R_f_b=R_lambda_delta([1;0;0],a_f)*R_lambda_delta([0;1;0],delta);

% the velocity of fin wrt fin frame {f}
v_f_f=inv(R_f_b)*[u_f;v_f;w_f];

% attack angle calculation in fin frame {f}

alpha=atan2(v_f_f(3),v_f_f(1));

% the drag and lift coefficient

cl=alpha2cl(alpha);

cd=alpha2cd(alpha);

% fin flow rotation matrix

R_flow_f=[cos(alpha) 0 -sin(alpha);0 1 0;sin(alpha) 0 cos(alpha)];

% damping force on fin wrt flow frame {flow}

F_f_flow=-0.5*rho*sl*cl*(v_f_f(1)^2+v_f_f(3)^2)*[cd;0;cl];

% damping force and moment on fin wrt body frame {b}

F_f_b=R_f_b*R_flow_f*F_f_flow;

M_fbb=vp(r_f,F_f_b);






