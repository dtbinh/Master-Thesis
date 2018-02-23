%% Optimal Configuration Selection 

% we extract the subsystem of the kinematic properties 

% use the smallest singular value of the controllability grammian to
% evaluate the norm 


% 1. Trim Trajectory
A_E_1_sub=double(A_E_1(1:6,1:6));
B_E_1_sub=double(B_E_1(1:6,:));

SVDMin1=min(svd(CtrGram(A_E_1_sub,B_E_1_sub)));

SVDMin1_c{index_loop}=SVDMin1;
% C_E_1_sub=eye(6,6);
% D_E_1_sub=zeros(6,6);


% 2. Trim Trajectory
A_E_2_sub=double(A_E_2(1:6,1:6));
B_E_2_sub=double(B_E_2(1:6,:));

SVDMin2=min(svd(CtrGram(A_E_2_sub,B_E_2_sub)));

SVDMin2_c{index_loop}=SVDMin2;

% 3. Trim Trajectory
A_E_3_sub=double(A_E_3(1:6,1:6));
B_E_3_sub=double(B_E_3(1:6,:));

SVDMin3=min(svd(CtrGram(A_E_3_sub,B_E_3_sub)));

SVDMin3_c{index_loop}=SVDMin3;

% 4. Trim Trajectory
A_E_4_sub=double(A_E_4(1:6,1:6));
B_E_4_sub=double(B_E_4(1:6,:));

SVDMin4=min(svd(CtrGram(A_E_4_sub,B_E_4_sub)));

SVDMin4_c{index_loop}=SVDMin4;

% 5. Trim Trajectory
A_E_5_sub=double(A_E_5(1:6,1:6));
B_E_5_sub=double(B_E_5(1:6,:));

SVDMin5=min(svd(CtrGram(A_E_5_sub,B_E_5_sub)));

SVDMin5_c{index_loop}=SVDMin5;

% 6. Trim Trajectory
A_E_6_sub=double(A_E_6(1:6,1:6));
B_E_6_sub=double(B_E_6(1:6,:));

SVDMin6=min(svd(CtrGram(A_E_6_sub,B_E_6_sub)));

SVDMin6_c{index_loop}=SVDMin6;


% 7. Trim Trajectory
A_E_7_sub=double(A_E_7(1:6,1:6));
B_E_7_sub=double(B_E_7(1:6,:));

SVDMin7=min(svd(CtrGram(A_E_7_sub,B_E_7_sub)));

SVDMin7_c{index_loop}=SVDMin7;

sss=cell2mat(SVDMin1_c)+cell2mat(SVDMin2_c)+cell2mat(SVDMin3_c)+cell2mat(SVDMin4_c)+cell2mat(SVDMin5_c)+cell2mat(SVDMin6_c)+cell2mat(SVDMin7_c)
