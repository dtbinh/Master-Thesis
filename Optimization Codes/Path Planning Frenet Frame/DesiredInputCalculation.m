%% Calculate the desired input
% The desired input is calculated from desired acceleration, desired
% velocity and the inertia matrix based on the underwater robot dynamic
% equation: M*v_dot+C*v=tau

%% first trimming trajectory
tau_d_visual_1=[];
v_d_1=[linear_velocity_x_1,linear_velocity_y_1,linear_velocity_z_1,angular_velocity_roll_1,angular_velocity_pitch_1,angular_velocity_yaw_1];
i=1;
for i=1:size(v_d_1,1)
% The Coriolis term is dependent on the generalised velocity
% velocity transposed: all quanties should be column vector
[CRB1,CA1]=tau_cor(veh,v_d_1(i,:)',v_d_1(i,:)');
D1=veh.D*(v_d_1(i,:)'.*abs(v_d_1(i,:)'));
tau_d_1(:,:,i)=CRB1+CA1+D1;
tau_d_visual_1=[tau_d_visual_1,CRB1+CA1+D1];
i=i+1;
end
tau_d_visual_1=tau_d_visual_1';

%% second trimming trajectory
tau_d_visual_2=[];
v_d_2=[linear_velocity_x_2,linear_velocity_y_2,linear_velocity_z_2,angular_velocity_roll_2,angular_velocity_pitch_2,angular_velocity_yaw_2];
i=1;
for i=1:size(v_d_2,1)
% The Coriolis term is dependent on the generalised velocity
% velocity transposed: all quanties should be column vector
[CRB2,CA2]=tau_cor(veh,v_d_2(i,:)',v_d_2(i,:)');
D2=veh.D*(v_d_2(i,:)'.*abs(v_d_2(i,:)'));
tau_d_2(:,:,i)=CRB2+CA2+D2;
tau_d_visual_2=[tau_d_visual_2,CRB2+CA2+D2];
i=i+1;
end
tau_d_visual_2=tau_d_visual_2';

%% third trimming trajectory
tau_d_visual_3=[];
v_d_3=[linear_velocity_x_3,linear_velocity_y_3,linear_velocity_z_3,angular_velocity_roll_3,angular_velocity_pitch_3,angular_velocity_yaw_3];
i=1;
for i=1:size(v_d_3,1)
% The Coriolis term is dependent on the generalised velocity
% velocity transposed: all quanties should be column vector
[CRB3,CA3]=tau_cor(veh,v_d_3(i,:)',v_d_3(i,:)');
D3=veh.D*(v_d_3(i,:)'.*abs(v_d_3(i,:)'));
tau_d_3(:,:,i)=CRB3+CA3+D3;
tau_d_visual_3=[tau_d_visual_3,CRB3+CA3+D3];
i=i+1;
end
tau_d_visual_3=tau_d_visual_3';

%% fourth trimming trajectory
tau_d_visual_4=[];
v_d_4=[linear_velocity_x_4,linear_velocity_y_4,linear_velocity_z_4,angular_velocity_roll_4,angular_velocity_pitch_4,angular_velocity_yaw_4];
i=1;
for i=1:size(v_d_4,1)
% The Coriolis term is dependent on the generalised velocity
% velocity transposed: all quanties should be column vector
[CRB4,CA4]=tau_cor(veh,v_d_4(i,:)',v_d_4(i,:)');
D4=veh.D*(v_d_4(i,:)'.*abs(v_d_4(i,:)'));
tau_d_4(:,:,i)=CRB4+CA4+D4;
tau_d_visual_4=[tau_d_visual_4,CRB4+CA4+D4];
i=i+1;
end
tau_d_visual_4=tau_d_visual_4';

%% fifth trimming trajectory
tau_d_visual_5=[];
v_d_5=[linear_velocity_x_5,linear_velocity_y_5,linear_velocity_z_5,angular_velocity_roll_5,angular_velocity_pitch_5,angular_velocity_yaw_5];
i=1;
for i=1:size(v_d_5,1)
% The Coriolis term is dependent on the generalised velocity
% velocity transposed: all quanties should be column vector
[CRB5,CA5]=tau_cor(veh,v_d_5(i,:)',v_d_5(i,:)');
D5=veh.D*(v_d_5(i,:)'.*abs(v_d_5(i,:)'));
tau_d_5(:,:,i)=CRB5+CA5+D5;
tau_d_visual_5=[tau_d_visual_5,CRB5+CA5+D5];
i=i+1;
end
tau_d_visual_5=tau_d_visual_5';

%% sixth trimming trajectory
tau_d_visual_6=[];
v_d_6=[linear_velocity_x_6,linear_velocity_y_6,linear_velocity_z_6,angular_velocity_roll_6,angular_velocity_pitch_6,angular_velocity_yaw_6];
i=1;
for i=1:size(v_d_6,1)
% The Coriolis term is dependent on the generalised velocity
% velocity transposed: all quanties should be column vector
[CRB6,CA6]=tau_cor(veh,v_d_6(i,:)',v_d_6(i,:)');
D6=veh.D*(v_d_6(i,:)'.*abs(v_d_6(i,:)'));
tau_d_6(:,:,i)=CRB6+CA6+D6;
tau_d_visual_6=[tau_d_visual_6,CRB6+CA6+D6];
i=i+1;
end
tau_d_visual_6=tau_d_visual_6';

%% seventh trimming trajectory
tau_d_visual_7=[];
v_d_7=[linear_velocity_x_7,linear_velocity_y_7,linear_velocity_z_7,angular_velocity_roll_7,angular_velocity_pitch_7,angular_velocity_yaw_7];
i=1;
for i=1:size(v_d_7,1)
% The Coriolis term is dependent on the generalised velocity
% velocity transposed: all quanties should be column vector
[CRB7,CA7]=tau_cor(veh,v_d_7(i,:)',v_d_7(i,:)');
D7=veh.D*(v_d_7(i,:)'.*abs(v_d_7(i,:)'));
tau_d_7(:,:,i)=CRB7+CA7+D7;
tau_d_visual_7=[tau_d_visual_7,CRB7+CA7+D7];
i=i+1;
end
tau_d_visual_7=tau_d_visual_7';




