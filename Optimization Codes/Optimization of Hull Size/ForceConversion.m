function F_out=ForceConversion(F_in,ConvertDirection) 
% This function performs force conversion between units pound-force and newton
% when ConversionDirection is 1: Conversion from pound-force to newton
% when ConversionDirection is -1: Conversion from newton to pound-force
% http://www.aqua-calc.com/convert/force/pound-force-to-newton
if ConvertDirection==1
F_out=F_in*4.4482216152605;
end
if ConvertDirection==-1
    F_out=F_in/4.4482216152605;
end