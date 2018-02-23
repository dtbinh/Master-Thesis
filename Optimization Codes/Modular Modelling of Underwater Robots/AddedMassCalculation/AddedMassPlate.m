function Ma_P = AddedMassPlate(rho,sp,ch)
% This function calculates the added mass matrix for the fins
% fins will be approximated as a rectangular which determined by two
% parameters:  the span sp and the chord ch
m33=0.25*pi*rho*ch^2*sp;
Ma_P=diag([])


end

