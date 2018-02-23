function R_A = R_lambda_delta(lambda,delta)
% This function calculates the rotation matrix for a rotation of angle
% delta
% about the lambda axis

R_A=eye(3)+sin(delta)*vp(lambda)+(1-cos(delta))*vp(lambda)*vp(lambda);


end
