% GeometricParaInitialization;
%% Main loop of the Geometric Optimization

% sequential optimization

% define an index to denote whether the new system is controllable or not

index_loop=1;

% two criteria are used to break the loop

% The most important criteria is the satification of the controllability

% If the first criteria is not broken, we check the convergence of the
% controllability matrix

% The second one is the norm of the controllability matrix 

epsilon=0.01;

% define the following 

Ctr_E_1_pre=zeros(12,72);

Ctr_E_2_pre=zeros(12,72);

Ctr_E_3_pre=zeros(12,72);

Ctr_E_4_pre=zeros(12,72);

Ctr_E_5_pre=zeros(12,72);

Ctr_E_6_pre=zeros(12,72);

Ctr_E_7_pre=zeros(12,72);


while (1)
    
    if index_loop>=200
            breakflag0=sprintf('Too many iterations');
            index_loop=index_loop-1;
    
    disp(breakflag0);
    break;
    end
    
    
OptimizationControlInput; 

OptimizationPosition;

OptimizationDirection;

% OptimizationControlInput_old; 
% 
% OptimizationPosition_old;
% 
% OptimizationDirection_old;

% after we perform optimization for the geometric parameters of all
% components


% How do we choose the best geometric configuration configuration



% we must update all the terms related to the geometric center 

%% Check the Controllability 

% in the file ControllabilityCheck we update the configuration vector for
% all the actuators, and then the configuration and the dynamics are also
% updated

% update the configuration vector

% update the error dynamics

ControllabilityCheck;

% OptimalConfigurationSelection;


% How do we choose the best geometric configuration configuration for a



if(CtrlCheck==0)
        
    breakflag1=sprintf('Optimization ends because the controllability is broken');
    
    disp(breakflag1);
    
end




%% store all geometric variables 

% store the result from the first optimization phase 

% u_d_ck{index_loop}=u_d;

% store the optimized diretion of each iteration

d_T1_ck{index_loop}=veh.d_T1;

d_T2_ck{index_loop}=veh.d_T2;

d_T3_ck{index_loop}=veh.d_T3;

d_T4_ck{index_loop}=veh.d_T4;

d_T5_ck{index_loop}=veh.d_T5;

d_T6_ck{index_loop}=veh.d_T6;

%  store the optimized position of each iteration

r_T1_ck{index_loop}=veh.r_T1;

r_T2_ck{index_loop}=veh.r_T2;

r_T3_ck{index_loop}=veh.r_T3;

r_T4_ck{index_loop}=veh.r_T4;

r_T5_ck{index_loop}=veh.r_T5;

r_T6_ck{index_loop}=veh.r_T6;

%

% ConfigurationUpdate;
% 
% %
% 
% DynamicsUpdate;

% the desired input should also be updated 

DesiredInputUpdate;

% define variables to store the geometric configuration in the last
% iteration

% 

% 

% optimization result from the first optimization phase

% u_d_last=u_d;

% optimization result from the second optimization phase

% the location of all thrusters

r_T1_last=veh.r_T1;
r_T2_last=veh.r_T2;
r_T3_last=veh.r_T3;
r_T4_last=veh.r_T4;
r_T5_last=veh.r_T5;
r_T6_last=veh.r_T6;

% the direction of all other thrusters 

d_T1_last=veh.d_T1;
d_T2_last=veh.d_T2;
d_T3_last=veh.d_T3;
d_T4_last=veh.d_T4;
d_T5_last=veh.d_T5;
d_T6_last=veh.d_T6;





%% the current controllability matrices

Ctr_E_1_post=Ctr_E_1;

Ctr_E_2_post=Ctr_E_2;

Ctr_E_3_post=Ctr_E_3;

Ctr_E_4_post=Ctr_E_4;

Ctr_E_5_post=Ctr_E_5;

Ctr_E_6_post=Ctr_E_6;

Ctr_E_7_post=Ctr_E_7;


% use the condition number to choose the best condition 

Ctr_E_1_diff=norm(Ctr_E_1_post-Ctr_E_1_pre);
Ctr_E_2_diff=norm(Ctr_E_2_post-Ctr_E_2_pre);
Ctr_E_3_diff=norm(Ctr_E_3_post-Ctr_E_3_pre);
Ctr_E_4_diff=norm(Ctr_E_4_post-Ctr_E_4_pre);
Ctr_E_5_diff=norm(Ctr_E_5_post-Ctr_E_5_pre);
Ctr_E_6_diff=norm(Ctr_E_6_post-Ctr_E_6_pre);
Ctr_E_7_diff=norm(Ctr_E_7_post-Ctr_E_7_pre);


Ctr_E_av=(Ctr_E_1_diff+Ctr_E_2_diff+Ctr_E_3_diff+Ctr_E_4_diff+Ctr_E_5_diff+Ctr_E_6_diff+Ctr_E_7_diff)/7;

if Ctr_E_av<=epsilon
    
    breakflag1=sprintf('Optimization ends because the average norm %d is smaller than the predefined norm limit %d',Ctr_E_av,epsilon);
    
    disp(breakflag1);
    
    break;
   
end 

Ctr_E_1_pre=Ctr_E_1_post;

Ctr_E_2_pre=Ctr_E_2_post;

Ctr_E_3_pre=Ctr_E_3_post;

Ctr_E_4_pre=Ctr_E_4_post;

Ctr_E_5_pre=Ctr_E_5_post;

Ctr_E_6_pre=Ctr_E_6_post;

Ctr_E_7_pre=Ctr_E_7_post;



index_loop=index_loop+1;
end


% %% when the index variable is equal to 0;
% 
% % the main loop for optimization is broken when the controllability is not
% % satisfied we use the result from the last iteration as our optimal value 
% 
% % optimization is finished 
% 
% % use the geometric configuration parameter in the last iteration 
% 
% % u_d_optimized=u_d_last;
% 
% % in the current situation 
% 
% r_T1_optimized=r_T1_last;
% 
% r_T2_optimized=r_T2_last;
% 
% r_T3_optimized=r_T3_last;
% 
% r_T4_optimized=r_T4_last;
% 
% r_T5_optimized=r_T5_last;
% 
% r_T6_optimized=r_T6_last;
% 
% % the optimal spin direction   
% 
% d_T1_optimized=d_T1_last;
% 
% d_T2_optimized=d_T2_last;
% 
% d_T3_optimized=d_T3_last;
% 
% d_T4_optimized=d_T4_last;
% 
% d_T5_optimized=d_T5_last;
% 
% d_T6_optimized=d_T6_last;
% 
% % new way for choosing the best configuration number use condition number
% % to choose the best one
% 


%%
% 
% u_d_optimized=u_d_ck{min_conindex};
% 
% % in the current situation 
% 
Cond_sum=cell2mat(Cond_E_1_c)+cell2mat(Cond_E_2_c)+cell2mat(Cond_E_3_c)+cell2mat(Cond_E_4_c)+cell2mat(Cond_E_5_c)+cell2mat(Cond_E_6_c)+cell2mat(Cond_E_7_c);

[mincond,min_condindex]=min(Cond_sum);

% min_condindex=index_loop;

r_T1_optimized=r_T1_ck{min_condindex};

r_T2_optimized=r_T2_ck{min_condindex};

r_T3_optimized=r_T3_ck{min_condindex};

r_T4_optimized=r_T4_ck{min_condindex};

r_T5_optimized=r_T5_ck{min_condindex};

r_T6_optimized=r_T6_ck{min_condindex};

% the optimal spin direction   

d_T1_optimized=d_T1_ck{min_condindex};

d_T2_optimized=d_T2_ck{min_condindex};

d_T3_optimized=d_T3_ck{min_condindex};

d_T4_optimized=d_T4_ck{min_condindex};

d_T5_optimized=d_T5_ck{min_condindex};

d_T6_optimized=d_T6_ck{min_condindex};







