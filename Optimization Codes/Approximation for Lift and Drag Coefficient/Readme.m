%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% approximate the characteristic curve of the lift and drag coefficient 
% The function a2clcd calculate the real value of lift and drag coefficient from the attack angle alpha
% [Cl Cd]=a2clcd(alpha) 

% The file CurveFittingClCd.m approximate the lift and drag coefficient with 9-order polynomial
% two fitted function handles f_cd and f_cl will be generated and stored in "ClCdFitCurve.mat"
% Cl=alpha2cl(alpha)
% Cd=alpha2cd(alpha) calculates the lift and drag force approximately