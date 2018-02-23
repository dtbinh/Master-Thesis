function [c,ceq] = nonl_constraints_dir(x)

% This function outputs the nonlinear equality and inequality constraints
% c_eq(x)=0

% nonlinear constraints

c=[];

% equality constraint to keep the direction vector a unit vector

ceq=x(1)^2+x(2)^2+x(3)^2-1;


end

