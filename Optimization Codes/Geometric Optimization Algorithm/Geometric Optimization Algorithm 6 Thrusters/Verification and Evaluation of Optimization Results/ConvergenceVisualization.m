
% when we change the geometric parameters of all thrusters the system
% matrix and the input matrix will change, the controllability matrix is 
% uniquely determined by the matrix A and B

% we should check the convergence of the controllability matrix
%% check the convergence of the system by means of checking the controllabillity matrix
% we check the norm of the controllability matrix

% trim trajectory 1

% extract from Ctr_E

Ctr_E_norm_1_c=[];
for i=1:index_loop
    Ctr_E_norm_1_c=[Ctr_E_norm_1_c;norm(Ctr_E_1_c{i})];
end

% trim trajectory 2

Ctr_E_norm_2_c=[];
for i=1:index_loop
    Ctr_E_norm_2_c=[Ctr_E_norm_2_c;norm(Ctr_E_2_c{i})];
end

% trim trajectory 3

Ctr_E_norm_3_c=[];
for i=1:index_loop
    Ctr_E_norm_3_c=[Ctr_E_norm_3_c;norm(Ctr_E_3_c{i})];
end

% trim trajectory 4
Ctr_E_norm_4_c=[];
for i=1:index_loop
    Ctr_E_norm_4_c=[Ctr_E_norm_4_c;norm(Ctr_E_4_c{i})];
end


% trim trajectory 5

Ctr_E_norm_5_c=[];
for i=1:index_loop
    Ctr_E_norm_5_c=[Ctr_E_norm_5_c;norm(Ctr_E_5_c{i})];
end


% trim trajectory 6

Ctr_E_norm_6_c=[];
for i=1:index_loop
    Ctr_E_norm_6_c=[Ctr_E_norm_6_c;norm(Ctr_E_6_c{i})];
end

% trim trajectory 7

Ctr_E_norm_7_c=[];
for i=1:index_loop
    Ctr_E_norm_7_c=[Ctr_E_norm_7_c;norm(Ctr_E_7_c{i})];
end


% ploT all the norms in one plot 
figure;
plot(Ctr_E_norm_1_c,'LineWidth',3);
hold on;
plot(Ctr_E_norm_2_c,'LineWidth',3);
hold on;
plot(Ctr_E_norm_3_c,'LineWidth',3);
hold on;
plot(Ctr_E_norm_4_c,'LineWidth',3);
hold on;
plot(Ctr_E_norm_5_c,'LineWidth',3);
hold on;
plot(Ctr_E_norm_6_c,'LineWidth',3);
hold on;
plot(Ctr_E_norm_7_c,'LineWidth',3);
title('Convergence of Error Dynamics Controllability Matrix Norm');
xlabel('iterations');
ylabel('The Norm of the Error Dynamics Controllability Matrix');
legend('C_{E1}','C_{E2}','C_{E3}','C_{E4}','C_{E5}','C_{E6}','C_{E7}');

%% check the convergence of all geometric variables

% check the convergence of all positions of all thrusters

% thruster 1

% extract the position coordinate separately 

r_T1_x_ck=[];

% extract r_x

for i=1:index_loop
   r_T1_x_ck=[r_T1_x_ck;r_T1_ck{i}(1)];
end

r_T1_y_ck=[];

% extract r_y

for i=1:index_loop
   r_T1_y_ck=[r_T1_y_ck;r_T1_ck{i}(2)]; 
end

% extract r_z

r_T1_z_ck=[];

for i=1:index_loop
    r_T1_z_ck=[r_T1_z_ck;d_T1_ck{i}(3)];
end

figure;
plot(r_T1_x_ck,'LineWidth',3);
hold on;
plot(r_T1_y_ck,'LineWidth',3);
hold on;
plot(r_T1_z_ck,'LineWidth',3);
title('Convergence of the thruster 1 Position');
xlabel('iterations');
ylabel('position in body frame {b} (m)');
legend('r_x','r_y','r_z');

% thruster 2

r_T2_x_ck=[];

% extract r_x

for i=1:index_loop
   r_T2_x_ck=[r_T2_x_ck;r_T2_ck{i}(1)];
end


% extract r_y

r_T2_y_ck=[];


for i=1:index_loop
   r_T2_y_ck=[r_T2_y_ck;r_T2_ck{i}(2)]; 
end

% extract r_z

r_T2_z_ck=[];

for i=1:index_loop
   r_T2_z_ck=[r_T2_z_ck;r_T2_ck{i}(3)];
end

figure;
plot(r_T2_x_ck,'LineWidth',3);
hold on;
plot(r_T2_y_ck,'LineWidth',3);
hold on;
plot(r_T2_z_ck,'LineWidth',3);
title('Convergence of the thruster 2 Position');
xlabel('iterations');
ylabel('position in body frame {b} (m)');
legend('r_x','r_y','r_z');

% thruster 3

r_T3_x_ck=[];

% extract r_x

for i=1:index_loop
   r_T3_x_ck=[r_T3_x_ck;r_T3_ck{i}(1)];
end

r_T3_y_ck=[];

% extract r_y

for i=1:index_loop
   r_T3_y_ck=[r_T3_y_ck;r_T3_ck{i}(2)]; 
end

% extract r_z

r_T3_z_ck=[];

for i=1:index_loop
    r_T3_z_ck=[r_T3_z_ck;r_T3_ck{i}(3)];
end

figure;
plot(r_T3_x_ck,'LineWidth',3);
hold on;
plot(r_T3_y_ck,'LineWidth',3);
hold on;
plot(r_T3_z_ck,'LineWidth',3);
title('Convergence of the thruster 3 Position');
xlabel('iterations');
ylabel('position in body frame {b} (m)');
legend('r_x','r_y','r_z');

% thruster 4

r_T4_x_ck=[];

% extract r_x

for i=1:index_loop
   r_T4_x_ck=[r_T4_x_ck;r_T4_ck{i}(1)];
end

r_T4_y_ck=[];

% extract r_y

for i=1:index_loop
   r_T4_y_ck=[r_T4_y_ck;r_T4_ck{i}(2)]; 
end

% extract r_z

r_T4_z_ck=[];

for i=1:index_loop
    r_T4_z_ck=[r_T4_z_ck;r_T4_ck{i}(3)];
end


figure;
plot(r_T4_x_ck,'LineWidth',3);
hold on;
plot(r_T4_y_ck,'LineWidth',3);
hold on;
plot(r_T4_z_ck,'LineWidth',3);
title('Convergence of the thruster 4 Position');
xlabel('iterations');
ylabel('position in body frame {b} (m)');
legend('r_x','r_y','r_z');

% thruster 5

r_T5_x_ck=[];

% extract r_x

for i=1:index_loop
   r_T5_x_ck=[r_T5_x_ck;r_T5_ck{i}(1)];
end

r_T5_y_ck=[];

% extract r_y

for i=1:index_loop
   r_T5_y_ck=[r_T5_y_ck;r_T5_ck{i}(2)]; 
end

% extract r_z

r_T5_z_ck=[];

for i=1:index_loop
    r_T5_z_ck=[r_T5_z_ck;r_T5_ck{i}(3)];
end

figure;
plot(r_T5_x_ck,'LineWidth',3);
hold on;
plot(r_T5_y_ck,'LineWidth',3);
hold on;
plot(r_T5_z_ck,'LineWidth',3);
title('Convergence of the thruster 5 Position');
xlabel('iterations');
ylabel('position in body frame {b} (m)');
legend('r_x','r_y','r_z');

% thruster 6

r_T6_x_ck=[];

% extract r_x

for i=1:index_loop
   r_T6_x_ck=[r_T6_x_ck;r_T6_ck{i}(1)];
end

r_T6_y_ck=[];

% extract r_y

for i=1:index_loop
   r_T6_y_ck=[r_T6_y_ck;r_T6_ck{i}(2)];
end

r_T6_z_ck=[];
% extract r_z

for i=1:index_loop
   r_T6_z_ck=[r_T6_z_ck;r_T6_ck{i}(3)];
end

figure;
plot(r_T6_x_ck,'LineWidth',3);
hold on;
plot(r_T6_y_ck,'LineWidth',3);
hold on;
plot(r_T6_z_ck,'LineWidth',3);
title('Convergence of the thruster 6 Position');
xlabel('iterations');
ylabel('position in body frame {b} (m)');
legend('r_x','r_y','r_z');

%% check the convergence of all directions of all thrusters

% thruster 1

% extract the position coordinate separately 

d_T1_x_ck=[];

% extract r_x

for i=1:index_loop
   d_T1_x_ck=[d_T1_x_ck;d_T1_ck{i}(1)];
end

d_T1_y_ck=[];

% extract r_y

for i=1:index_loop
   d_T1_y_ck=[d_T1_y_ck;d_T1_ck{i}(2)]; 
end

% extract r_z

d_T1_z_ck=[];

for i=1:index_loop
    d_T1_z_ck=[d_T1_z_ck;d_T1_ck{i}(3)];
end

figure;
plot(d_T1_x_ck,'LineWidth',3);
hold on;
plot(d_T1_y_ck,'LineWidth',3);
hold on;
plot(d_T1_z_ck,'LineWidth',3);
title('Convergence of the thruster 1 Direction');
xlabel('iterations');
ylabel('position in body frame {b}');
legend('d_x','d_y','d_z');


% thruster 2
% extract the position coordinate separately 

d_T2_x_ck=[];

% extract r_x

for i=1:index_loop
   d_T2_x_ck=[d_T2_x_ck;d_T2_ck{i}(1)];
end

d_T2_y_ck=[];

% extract r_y

for i=1:index_loop
   d_T2_y_ck=[d_T2_y_ck;d_T2_ck{i}(2)]; 
end

% extract r_z

d_T2_z_ck=[];

for i=1:index_loop
    d_T2_z_ck=[d_T2_z_ck;d_T2_ck{i}(3)];
end

figure;
plot(d_T2_x_ck,'LineWidth',3);
hold on;
plot(d_T2_y_ck,'LineWidth',3);
hold on;
plot(d_T2_z_ck,'LineWidth',3);
title('Convergence of the thruster 2 Direction');
xlabel('iterations');
ylabel('position in body frame {b}');
legend('d_x','d_y','d_z');

% thruster 3
% extract the position coordinate separately 

d_T3_x_ck=[];

% extract r_x

for i=1:index_loop
   d_T3_x_ck=[d_T3_x_ck;d_T3_ck{i}(1)];
end

d_T3_y_ck=[];

% extract r_y

for i=1:index_loop
   d_T3_y_ck=[d_T3_y_ck;d_T3_ck{i}(2)]; 
end

% extract r_z

d_T3_z_ck=[];

for i=1:index_loop
    d_T3_z_ck=[d_T3_z_ck;d_T3_ck{i}(3)];
end

figure;
plot(d_T3_x_ck,'LineWidth',3);
hold on;
plot(d_T3_y_ck,'LineWidth',3);
hold on;
plot(d_T3_z_ck,'LineWidth',3);
title('Convergence of the thruster 3 Direction');
xlabel('iterations');
ylabel('position in body frame {b}');
legend('d_x','d_y','d_z');


% thruster 4
% extract the position coordinate separately 

d_T4_x_ck=[];

% extract r_x

for i=1:index_loop
   d_T4_x_ck=[d_T4_x_ck;d_T4_ck{i}(1)];
end

d_T4_y_ck=[];

% extract r_y

for i=1:index_loop
   d_T4_y_ck=[d_T4_y_ck;d_T4_ck{i}(2)]; 
end

% extract r_z

d_T4_z_ck=[];

for i=1:index_loop
    d_T4_z_ck=[d_T4_z_ck;d_T4_ck{i}(3)];
end

figure;
plot(d_T4_x_ck,'LineWidth',3);
hold on;
plot(d_T4_y_ck,'LineWidth',3);
hold on;
plot(d_T4_z_ck,'LineWidth',3);
title('Convergence of the thruster 4 Direction');
xlabel('iterations');
ylabel('position in body frame {b}');
legend('d_x','d_y','d_z');

% thruster 5
% extract the position coordinate separately 

d_T5_x_ck=[];

% extract r_x

for i=1:index_loop
   d_T5_x_ck=[d_T5_x_ck;d_T5_ck{i}(1)];
end

d_T5_y_ck=[];

% extract r_y

for i=1:index_loop
   d_T5_y_ck=[d_T5_y_ck;d_T5_ck{i}(2)]; 
end

% extract r_z

d_T5_z_ck=[];

for i=1:index_loop
    d_T5_z_ck=[d_T5_z_ck;d_T5_ck{i}(3)];
end

figure;
plot(d_T5_x_ck,'LineWidth',3);
hold on;
plot(d_T5_y_ck,'LineWidth',3);
hold on;
plot(d_T5_z_ck,'LineWidth',3);
title('Convergence of the thruster 5 Direction');
xlabel('iterations');
ylabel('position in body frame {b}');
legend('d_x','d_y','d_z');

% thruster 6

% extract the position coordinate separately 

d_T6_x_ck=[];

% extract r_x

for i=1:index_loop
   d_T6_x_ck=[d_T6_x_ck;d_T6_ck{i}(1)];
end

d_T6_y_ck=[];

% extract r_y

for i=1:index_loop
   d_T6_y_ck=[d_T6_y_ck;d_T6_ck{i}(2)]; 
end

% extract r_z

d_T6_z_ck=[];

for i=1:index_loop
    d_T6_z_ck=[d_T6_z_ck;d_T6_ck{i}(3)];
end

figure;
plot(d_T6_x_ck,'LineWidth',3);
hold on;
plot(d_T6_y_ck,'LineWidth',3);
hold on;
plot(d_T6_z_ck,'LineWidth',3);
title('Convergence of the thruster 6 Direction');
xlabel('iterations');
ylabel('position in body frame {b}');
legend('d_x','d_y','d_z');

%% 