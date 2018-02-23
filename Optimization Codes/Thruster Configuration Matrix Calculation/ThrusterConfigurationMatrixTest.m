%% in this script, the correctness of the function "ThrusterConfigurationMatrix"

T_1={veh.r_T1,veh.b_T1,veh.d_T1};
T_2={veh.r_T2,veh.b_T2,veh.d_T2};
T_3={veh.r_T3,veh.b_T3,veh.d_T3};
T_4={veh.r_T4,veh.b_T4,veh.d_T4};
T_5={veh.r_T5,veh.b_T5,veh.d_T5};
T_6={veh.r_T6,veh.b_T6,veh.d_T6};

B_Test=ThrusterConfigurationMatrix(T_1,T_2,T_3,T_4,T_5,T_6);

rk_test=rank(B_Test);

