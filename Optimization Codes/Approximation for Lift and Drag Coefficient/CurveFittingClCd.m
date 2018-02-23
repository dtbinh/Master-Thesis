%% This function is used to find approximate functions to fit the characteristic curve of cl and cd with respect to the attack angle alpha
%% Yongyu Chen 01.05.2017
%% curve fitting for drag and lift coeffient wrt to attack angle 
alpha=-pi:pi/50:pi;
cl=[];
cd=[];
% iteration to avoid mistakes
%%
% generate the weights vector for curve fitting to keep the correctness of
% zero value
% w_cd=ones(1,length(alpha));
% w_cd(1,1)=1;
%% curve fitting with 9-order polynomial
% note that when the attack angle equals zero, Cl is equal to zero and Cd
% is not equal to zero
for i=1:length(alpha)
[cl_1,cd_1]=a2clcd(alpha(i));
cl=[cl;cl_1];
cd=[cd;cd_1];
end
% f_cd=fit(alpha',cd,'smoothingspline')
f_cd=fit(alpha',cd,'poly9','Normalize','on')
figure('Name','Characteristic Curve of Drag Coefficient');
plot(f_cd,alpha',cd);
title('Characteristic Curve of Drag Coefficient');
f_cl=fit(alpha',cl,'poly9','Normalize','on')
% f_cl=fit(alpha',cl,'smoothingspline')
figure('Name','Characteristic Curve of Lift Coefficient');
plot(f_cl,alpha',cl);
title('Characteristic Curve of Lift Coefficient');
% using 'smoothingspline' the curve can be fitted perfectly, but can not be computed using the matlab symbolic tools

 
