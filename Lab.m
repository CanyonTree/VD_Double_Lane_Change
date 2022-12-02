%% Double Lane Change Lab
% MECH 4420

clc; clear all; close all;

g = 9.81;
Steering_Ratio = 14.8;
L = 2.85; % m
b = 1.593;
m = 1756+100+150; % kg
tr = 1.5748;
delta_x = 0.9144;
delta_y = 0.6096;
a = L-b;

t0_velocity = 5;
tfinal_velocity = 2;

file1 = 'brooks.bag';
file2 = 'bryce.bag';
file3 = 'nathan.bag';
file4 = 'noah.bag';


data1 = get_data(file1, t0_velocity, tfinal_velocity, Steering_Ratio);
data2 = get_data(file2, t0_velocity, tfinal_velocity, Steering_Ratio);
data3 = get_data(file3, t0_velocity, tfinal_velocity, Steering_Ratio);
data4 = get_data(file4, t0_velocity, tfinal_velocity, Steering_Ratio);

sim_input1 = [data1.Time', data1.IMU.Vel.X', data1.Steering.SteerAngle'];
sim_input2 = [data2.Time', data2.IMU.Vel.X', data2.Steering.SteerAngle'];
sim_input3 = [data3.Time', data3.IMU.Vel.X', data3.Steering.SteerAngle'];
sim_input4 = [data4.Time', data4.IMU.Vel.X', data4.Steering.SteerAngle'];

% sim_input1(:,2) = max(sim_input1(:,2)-2);
% sim_input2(:,2) = max(sim_input2(:,2)-2);
% sim_input3(:,2) = max(sim_input3(:,2)-2);
% sim_input4(:,2) = max(sim_input4(:,2)-2);

HW8_MKZ_Params

sim1_HW8 = car_sim_loop(sim_input1,car_params);
sim2_HW8 = car_sim_loop(sim_input2,car_params);
sim3_HW8 = car_sim_loop(sim_input3,car_params);
sim4_HW8 = car_sim_loop(sim_input4,car_params);

X0 = [0,0,0,0];
sim1_Pcode = car_sim_loop_set_steer(sim_input1(:,3),sim_input1(:,2),sim_input1(1,1),sim_input1(end,1),X0,car_params);
sim2_Pcode = car_sim_loop_set_steer(sim_input2(:,3),sim_input2(:,2),sim_input2(1,1),sim_input2(end,1),X0,car_params);
sim3_Pcode = car_sim_loop_set_steer(sim_input3(:,3),sim_input3(:,2),sim_input3(1,1),sim_input3(end,1),X0,car_params);
sim4_Pcode = car_sim_loop_set_steer(sim_input4(:,3),sim_input4(:,2),sim_input4(1,1),sim_input4(end,1),X0,car_params);

clear car_params
Lab3_MKZ_Params

sim1_Lab3 = car_sim_loop(sim_input1,car_params);
sim2_Lab3 = car_sim_loop(sim_input2,car_params);
sim3_Lab3 = car_sim_loop(sim_input3,car_params);
sim4_Lab3 = car_sim_loop(sim_input4,car_params);

sim1_Pcode_Duggoff = car_sim_loop_set_steer(sim_input1(:,3),sim_input1(:,2),sim_input1(1,1),sim_input1(end,1),X0,car_params);
sim2_Pcode_Duggoff = car_sim_loop_set_steer(sim_input2(:,3),sim_input2(:,2),sim_input2(1,1),sim_input2(end,1),X0,car_params);
sim3_Pcode_Duggoff = car_sim_loop_set_steer(sim_input3(:,3),sim_input3(:,2),sim_input3(1,1),sim_input3(end,1),X0,car_params);
sim4_Pcode_Duggoff = car_sim_loop_set_steer(sim_input4(:,3),sim_input4(:,2),sim_input4(1,1),sim_input4(end,1),X0,car_params);

%%%% Steer Angle vs. Time %%%%
figure
plot(sim_input1(:,1),sim_input1(:,3)*180/pi)
hold on
plot(sim_input2(:,1),sim_input2(:,3)*180/pi)
plot(sim_input3(:,1),sim_input3(:,3)*180/pi)
plot(sim_input4(:,1),sim_input4(:,3)*180/pi)
% plot(sim_input4(:,1),sim_input4(:,2),'m--')
title('Steer Angle vs. Time')
xlabel('Time (s)')
ylabel('Steer Angle (deg)')
legend('Brooks','Bryce','Nathan','Noah','Location','Best')

%%%% Displacement: Y vs. X %%%%
figure
plot(data1.IMU.Pos.X,data1.IMU.Pos.Y)
hold on
plot(data2.IMU.Pos.X,data2.IMU.Pos.Y)
plot(data3.IMU.Pos.X,data3.IMU.Pos.Y)
plot(data4.IMU.Pos.X,data4.IMU.Pos.Y)
xlabel('X Displacement (m)')
ylabel('Y Displacement (m)')
title('Displacement: Y vs. X')

%%%% Yaw Rate vs. Time %%%%
figure
plot(data1.Time,data1.IMU.AngVel.Yaw)
hold on
plot(data2.Time,data2.IMU.AngVel.Yaw)
plot(data3.Time,data3.IMU.AngVel.Yaw)
plot(data4.Time,data4.IMU.AngVel.Yaw)
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
title('Yaw Rate vs. Time')


%%%% GPS Position %%%%
figure
subplot(2,2,1)
plot([data1.GPS.Pos.Longitude],[data1.GPS.Pos.Latitude])
title('Brooks')
xlabel('East')
ylabel('North')
subplot(2,2,2)
plot([data2.GPS.Pos.Longitude],[data2.GPS.Pos.Latitude])
title('Bryce')
xlabel('East')
ylabel('North')
subplot(2,2,3)
plot([data3.GPS.Pos.Longitude],[data3.GPS.Pos.Latitude])
title('Nathan')
xlabel('East')
ylabel('North')
subplot(2,2,4)
plot([data4.GPS.Pos.Longitude],[data4.GPS.Pos.Latitude])
title('Noah')
xlabel('East')
ylabel('North')

%%%% Sim and Real Yah Rates %%%%
figure
subplot(2,2,1)
plot(data1.Time,data1.IMU.AngVel.Yaw)
hold on
plot([sim1_HW8.time], [sim1_HW8.r]*180/pi,'r--','LineWidth',2)
plot([sim1_Lab3.time], [sim1_Lab3.r]*180/pi,'g--','LineWidth',2)
plot(sim1_Pcode.Time, sim1_Pcode.Yaw_Gyro*180/pi,'m--','LineWidth',2)
plot(sim1_Pcode_Duggoff.Time, sim1_Pcode_Duggoff.Yaw_Gyro*180/pi,'k--','LineWidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
title('Yaw Rate vs. Time')
legend('Experiment','HW8 Model','Lab 3 Model','run_MKZ')

subplot(2,2,2)
plot(data2.Time,data2.IMU.AngVel.Yaw)
hold on
plot([sim2_HW8.time], [sim2_HW8.r]*180/pi,'r--','LineWidth',2)
plot([sim2_Lab3.time], [sim2_Lab3.r]*180/pi,'g--','LineWidth',2)
plot(sim2_Pcode.Time, sim2_Pcode.Yaw_Gyro*180/pi,'m--','LineWidth',2)
plot(sim2_Pcode_Duggoff.Time, sim2_Pcode_Duggoff.Yaw_Gyro*180/pi,'k--','LineWidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
title('Yaw Rate vs. Time')
legend('Experiment','HW8 Model','Lab 3 Model','run_MKZ')

subplot(2,2,3)
plot(data3.Time,data3.IMU.AngVel.Yaw)
hold on
plot([sim3_HW8.time], [sim3_HW8.r]*180/pi,'r--','LineWidth',2)
plot([sim3_Lab3.time], [sim3_Lab3.r]*180/pi,'g--','LineWidth',2)
plot(sim3_Pcode.Time, sim3_Pcode.Yaw_Gyro*180/pi,'m--','LineWidth',2)
plot(sim3_Pcode_Duggoff.Time, sim3_Pcode_Duggoff.Yaw_Gyro*180/pi,'k--','LineWidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
title('Yaw Rate vs. Time')
legend('Experiment','HW8 Model','Lab 3 Model','run_MKZ')

subplot(2,2,4)
plot(data4.Time,data4.IMU.AngVel.Yaw)
hold on
plot([sim4_HW8.time], [sim4_HW8.r]*180/pi,'r--','LineWidth',2)
plot([sim4_Lab3.time], [sim4_Lab3.r]*180/pi,'g--','LineWidth',2)
plot(sim4_Pcode.Time, sim4_Pcode.Yaw_Gyro*180/pi,'m--','LineWidth',2)
plot(sim4_Pcode_Duggoff.Time, sim4_Pcode_Duggoff.Yaw_Gyro*180/pi,'k--','LineWidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
title('Yaw Rate vs. Time')
legend('Experiment','HW8 Model','Lab 3 Model','run_MKZ')
