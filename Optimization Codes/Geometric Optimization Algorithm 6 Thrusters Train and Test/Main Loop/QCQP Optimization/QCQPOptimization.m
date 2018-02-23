function [x,fval]=QCQPOptimization(Q,f,c,H,k,d)
%% In this function, we formulate the initial guess of the direction of each thruster as QCQP programming problem which has unique solution

% 

% using constrained optimization to formulate this function

options_diropt_qcqp = optimoptions(@fmincon,'Algorithm','interior-point',...
    'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
    'HessianFcn',@(x,lambda)quadhess(x,lambda,Q,H));
fun_diropt = @(x)quadobj(x,Q,f,c);
nonlconstr = @(x)quadconstr(x,H,k,d);
x0 = [0;0;0]; % column vector
[x,fval,eflag,output,lambda] = fmincon(fun_diropt,x0,...
    [],[],[],[],[],[],nonlconstr,options_diropt_qcqp);

% x is the optimized spin direction we should assign it back 

end