%% in this function, we test the correctness of the functions between coordinate transfer functions

x_Car=0.5;

y=1.0;

z=2.0;

[x_Cyl,phi,r]=CoorTransferCy2Car(x_Car,y,z)

[x_Car_test,y_Car_test,z_Car_test]=CoorTransferCar2Cy(x_Cyl,phi,r)

% The value of x_Car_test,y_Car_test,z_Car_test should be equal to x,y,z
% respectively



