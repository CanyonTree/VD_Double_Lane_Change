%% Homework 7
% Bryce Albritton
% MECH 4420 HW 7
clc; clear all; close all;

load_car_params

data1 = readmatrix("HW7_G35_datafile_1.txt");
data2 = readmatrix("HW7_G35_datafile_2.txt");
start_1 = 1; 
cutoff_1 = 390; 
start_2 = 715; 
cutoff_2 = 1138; 
start_3 = 1343; 
cutoff_3 = 1504; 



figure
plot(data2(:,1),data2(:,2))
title('2nd data set velocity vs. time')
ylabel('Velocity (m/s)')
% xline(data2(cutoff,1))
xlabel('Time (s)')
figure
plot(data2(:,1),data2(:,3)*180/pi)
% xline(data2(cutoff_2,1))
title('2nd data set steer angle vs. time')
ylabel('Steer Angle (deg)')
xlabel('Time (s)')

data2_1 = data2(start_1:cutoff_1,:);
data2_2 = data2(start_2:cutoff_2,:);
data2_3 = data2(start_3:cutoff_3,:);

sim1 = car_sim_loop(data1,car_params);
sim2_1 = car_sim_loop(data2_1,car_params);
sim2_2 = car_sim_loop(data2_2,car_params);
sim2_3 = car_sim_loop(data2_3,car_params);

V = data1(:,2);
r = data1(:,4);
B = data1(:,5);
Vx = V .* cos(B);
Vy = V .* sin(B);
data1_alphaF = (Vy + a*r)./Vx - data1(:,3);
data1_alphaR = (Vy - b*r)./Vx;

clear V r B Vx Vy
V = data2(:,2);
r = data2(:,4);
B = data2(:,5);
Vx = V .* cos(B);
Vy = V .* sin(B);
data2_alphaF = (Vy + a*r)./Vx - data2(:,3);
data2_alphaR = (Vy - b*r)./Vx;

% Yaw Rate
figure
subplot(2,1,1)
plot(data1(:,1),data1(:,4))
hold on
plot([sim1.time],[sim1.r],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (rad/s)')
legend('real','sim','Location','best')
title('Data Set 1 Yaw Rate vs. Time')

subplot(2,1,2)
plot(data2(:,1),data2(:,4))
hold on
plot([sim2_1.time],[sim2_1.r],'r--','LineWidth',2)
plot([sim2_2.time],[sim2_2.r],'r--','LineWidth',2)
% plot([sim2_3.time],[sim2_3.r],'r--','LineWidth',2)
% 
xlabel('Time (s)')
ylabel('Yaw Rate (rad/s)')
legend('real','sim','Location','best')
title('Data Set 2 Yaw Rate vs. Time')
ylim([-0.51, 1.1])

% Side Slip
figure
subplot(2,1,1)
plot(data1(:,1),data1(:,5))
hold on
plot([sim1.time],[sim1.B],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 1 Side Slip vs. Time')

subplot(2,1,2)
plot(data2(:,1),data2(:,5))
hold on
plot([sim2_1.time],[sim2_1.B],'r--','LineWidth',2)
plot([sim2_2.time],[sim2_2.B],'r--','LineWidth',2)
xlabel('Time (s)')
ylabel('Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 2 Side Slip vs. Time')


% Front Tire Side Slip
figure
subplot(2,1,1)
plot(data1(:,1),data1_alphaF)
hold on
plot([sim1.time],[sim1.alphaF],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Front Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 1 Front Tire Side Slip vs. Time')

subplot(2,1,2)
plot(data2(:,1),data2_alphaF)
hold on
plot([sim2_1.time],[sim2_1.alphaF],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Front Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 2 Front Tire Side Slip vs. Time')

% Rear Tire Side Slip
figure
subplot(2,1,1)
plot(data1(:,1),data1_alphaR)
hold on
plot([sim1.time],[sim1.alphaR],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Rear Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 1 Rear Tire Side Slip vs. Time')

subplot(2,1,2)
plot(data2(:,1),data2_alphaR)
hold on
plot([sim2_1.time],[sim2_1.alphaR],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Rear Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 2 Rear Tire Side Slip vs. Time')


%% B1

alpha = -pi/2:0.001:pi/2;


for i = 1:length(alpha)
    Fyf(i) = Duggoff_Fy(alpha(i),caf,Wf,mu);
    Fyr(i) = Duggoff_Fy(alpha(i),car,Wr,mu);
end

figure
subplot(2,1,1)
plot(alpha,Fyf)
title('Front Axle Duggoff Model')
xlabel('alpha (rad)')
ylabel('Fyf (N)')
subplot(2,1,2)
plot(alpha,Fyr)
title('Rear Axle Duggoff Model')
xlabel('alpha (rad)')
ylabel('Fyr (N)')

car_params.use_duggoff = 1;
%%
sim1_duggoff = car_sim_loop(data1,car_params);
sim2_duggoff = car_sim_loop(data2,car_params);


% Yaw Rate
figure
subplot(2,1,1)
plot(data1(:,1),data1(:,4))
hold on
plot([sim1_duggoff.time],[sim1_duggoff.r],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (rad/s)')
legend('real','sim','Location','best')
title('Data Set 1 Yaw Rate vs. Time with Duggoff Model')

subplot(2,1,2)
plot(data2(:,1),data2(:,4))
hold on
plot([sim2_duggoff.time],[sim2_duggoff.r],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (rad/s)')
legend('real','sim','Location','best')
title('Data Set 2 Yaw Rate vs. Time with Duggoff Model')


% Side Slip
figure
subplot(2,1,1)
plot(data1(:,1),data1(:,5))
hold on
plot([sim1_duggoff.time],[sim1_duggoff.B],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 1 Side Slip vs. Time with Duggoff Model')

subplot(2,1,2)
plot(data2(:,1),data2(:,5))
hold on
plot([sim2_duggoff.time],[sim2_duggoff.B],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 2 Side Slip vs. Time with Duggoff Model')


% Front Tire Side Slip
figure
subplot(2,1,1)
plot(data1(:,1),data1_alphaF)
hold on
plot([sim1_duggoff.time],[sim1_duggoff.alphaF],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Front Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 1 Front Tire Side Slip vs. Time with Duggoff Model')

subplot(2,1,2)
plot(data2(:,1),data2_alphaF)
hold on
plot([sim2_duggoff.time],[sim2_duggoff.alphaF],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Front Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 2 Front Tire Side Slip vs. Time with Duggoff Model')

% Rear Tire Side Slip
figure
subplot(2,1,1)
plot(data1(:,1),data1_alphaR)
hold on
plot([sim1_duggoff.time],[sim1_duggoff.alphaR],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Rear Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 1 Rear Tire Side Slip vs. Time with Duggoff Model')

subplot(2,1,2)
plot(data2(:,1),data2_alphaR)
hold on
plot([sim2_duggoff.time],[sim2_duggoff.alphaR],'--','LineWidth',2)
xlabel('Time (s)')
ylabel('Rear Tire Side Slip (rad)')
legend('real','sim','Location','best')
title('Data Set 2 Rear Tire Side Slip vs. Time with Duggoff Model')





