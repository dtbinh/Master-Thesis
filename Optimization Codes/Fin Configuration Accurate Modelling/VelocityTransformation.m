function V_A = VelocityTransformation(v,w,r_A)
% Calculate the generalised velocity V_A in an arbitrary point from the robot body velocity 
% all inputs are three dimensional column vectors 
% input: 1.v:[v_x;v_y;v_z]
%        2.w:[w_roll;w_pitch;w_yaw]
%        3.r_A:[r_A_x;r_A_y;r_A_z]
V_A=v+vp(w,r_A);
end

