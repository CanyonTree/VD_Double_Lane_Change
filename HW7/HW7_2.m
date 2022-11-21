%% Homework 7
% Bryce Albritton
% MECH 4420 HW 7 Problem 2
clc; clear all; close all;

load_car_params


%% Problem 2

% b & c
rdes = 50*pi/180;
V1 = 15;
V2 = 30;
tfinal = 1.5;



sim1 = Step_Car_Sim(V1,rdes,tfinal,car_params);
sim2 = Step_Car_Sim(V2,rdes,tfinal,car_params);
car_params.use_tire_relaxation = 1;
sim1_r = Step_Car_Sim(V1,rdes,tfinal,car_params);
sim2_r = Step_Car_Sim(V2,rdes,tfinal,car_params);
car_params.use_tire_relaxation = 0;


car_params.use_duggoff = 1;
tfinal = 10;
sim1_d = Step_Car_Sim(V1,rdes,tfinal,car_params);
sim2_d = Step_Car_Sim(V2,rdes,tfinal,car_params);

%% d
cg_shift = 0.2;
car_params.a = a + cg_shift;
car_params.b = b - cg_shift;
car_params.caf = car;
car_params.car = caf;
car_params.use_duggoff = 0;
Wf = m*g*car_params.b/L;         % N
Wr = m*g*car_params.a/L;         % N
car_params.Wf = Wf;
car_params.Wr = Wr;

kus = Wf/(caf) - Wr/(car);    % rad/g

car_params.kus = kus;
tfinal = 1.5;

sim3 = Step_Car_Sim(V1,rdes,tfinal,car_params);

sim4 = Step_Car_Sim(V2,rdes,tfinal,car_params);

car_params.use_tire_relaxation = 1;
sim3_r = Step_Car_Sim(V1,rdes,tfinal,car_params);
sim4_r = Step_Car_Sim(V2,rdes,tfinal,car_params);
car_params.use_tire_relaxation = 0;


car_params.use_duggoff = 1;
tfinal = 10;
sim3_d = Step_Car_Sim(V1,rdes,tfinal,car_params);
tfinal = 10;
sim4_d = Step_Car_Sim(V2,rdes,tfinal,car_params);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('15 m/s\n Poles: %0.2f+(%0.2fi), %0.2f+(%0.2f)i\n zeta: %0.2f\n omega_n: %0.2f \n\n',real(sim1(1).poles(1)),imag(sim1(1).poles(1)),real(sim1(1).poles(2)),imag(sim1(1).poles(2)),sim1(1).zeta,sim1(1).omega_n)
fprintf('30 m/s\n Poles: %0.2f+(%0.2fi), %0.2f+(%0.2f)i\n zeta: %0.2f\n omega_n: %0.2f \n\n',real(sim2(1).poles(1)),imag(sim2(1).poles(1)),real(sim2(1).poles(2)),imag(sim2(1).poles(2)),sim2(1).zeta,sim2(1).omega_n)
fprintf('15 m/s Shifted CG swapped Calpha\n Poles: %0.2f+(%0.2fi), %0.2f+(%0.2f)i\n zeta: %0.2f\n omega_n: %0.2f \n\n',real(sim3(1).poles(1)),imag(sim3(1).poles(1)),real(sim3(1).poles(2)),imag(sim3(1).poles(2)),sim3(1).zeta,sim3(1).omega_n)
fprintf('30 m/s Shifted CG swapped Calpha\n Poles: %0.2f+(%0.2fi), %0.2f+(%0.2f)i\n zeta: %0.2f\n omega_n: %0.2f \n\n',real(sim4(1).poles(1)),imag(sim4(1).poles(1)),real(sim4(1).poles(2)),imag(sim4(1).poles(2)),sim4(1).zeta,sim4(1).omega_n)

figure('position',[0, 0, 1000, 1000])
orient tall
subplot(4,2,1)
plot([sim1.time],[sim1.r]*180/pi)
title('15 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
subplot(4,2,2)
orient tall
plot([sim1.time],[sim1.B]*180/pi)
title('15 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

subplot(4,2,3)
plot([sim2.time],[sim2.r]*180/pi)
title('30 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
subplot(4,2,4)
orient tall
plot([sim2.time],[sim2.B]*180/pi)
title('30 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

subplot(4,2,5)
plot([sim3.time],[sim3.r]*180/pi)
title('Shifted CG & Swapped Calphas 15 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
subplot(4,2,6)
orient tall
plot([sim3.time],[sim3.B]*180/pi)
title('Shifted CG & Swapped Calphas 15 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

subplot(4,2,7)
plot([sim4.time],[sim4.r]*180/pi)
title('Shifted CG & Swapped Calphas 30 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
orient tall
subplot(4,2,8)
plot([sim4.time],[sim4.B]*180/pi)
title('Shifted CG & Swapped Calphas 30 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure('position',[0, 0, 1000, 1000])
orient tall
subplot(4,2,1)
plot([sim1_d.time],[sim1_d.r]*180/pi)
title('Duggoff 15 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
orient tall
subplot(4,2,2)
plot([sim1_d.time],[sim1_d.B]*180/pi)
title('Duggoff 15 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

subplot(4,2,3)
plot([sim2_d.time],[sim2_d.r]*180/pi)
title('Duggoff 30 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
orient tall
subplot(4,2,4)
plot([sim2_d.time],[sim2_d.B]*180/pi)
title('Duggoff 30 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

subplot(4,2,5)
plot([sim3_d.time],[sim3_d.r]*180/pi)
title('Duggoff Shifted CG & Swapped Calphas 15 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
orient tall
subplot(4,2,6)
plot([sim3_d.time],[sim3_d.B]*180/pi)
title('Duggoff Shifted CG & Swapped Calphas 15 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

subplot(4,2,7)
plot([sim4_d.time],[sim4_d.r]*180/pi)
title('Duggoff Shifted CG & Swapped Calphas 30 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
orient tall
subplot(4,2,8)
plot([sim4_d.time],[sim4_d.B]*180/pi)
title('Duggoff Shifted CG & Swapped Calphas 30 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
orient tall

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[0, 0, 1000, 1000])
orient tall
subplot(4,2,1)
plot([sim1.time],[sim1.r]*180/pi)
hold on
plot([sim1_r.time],[sim1_r.r]*180/pi)
title('15 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall

subplot(4,2,2)
plot([sim1.time],[sim1.B]*180/pi)
hold on
plot([sim1_r.time],[sim1_r.B]*180/pi)
title('15 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall


subplot(4,2,3)
plot([sim2.time],[sim2.r]*180/pi)
hold on
plot([sim2_r.time],[sim2_r.r]*180/pi)
title('30 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall

subplot(4,2,4)
plot([sim2.time],[sim2.B]*180/pi)
hold on
plot([sim2_r.time],[sim2_r.B]*180/pi)
title('30 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall


subplot(4,2,5)
plot([sim3.time],[sim3.r]*180/pi)
hold on
plot([sim3_r.time],[sim3_r.r]*180/pi)
title('Shifted CG & Swapped Calphas 15 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall

subplot(4,2,6)
plot([sim3.time],[sim3.B]*180/pi)
hold on
plot([sim3_r.time],[sim3_r.B]*180/pi)
title('Shifted CG & Swapped Calphas 15 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall


subplot(4,2,7)
plot([sim4.time],[sim4.r]*180/pi)
hold on
plot([sim4_r.time],[sim4_r.r]*180/pi)
title('Shifted CG & Swapped Calphas 30 m/s Yaw Rate vs. Time')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall

subplot(4,2,8)
plot([sim4.time],[sim4.B]*180/pi)
hold on
plot([sim4_r.time],[sim4_r.B]*180/pi)
title('Shifted CG & Swapped Calphas 30 m/s Side Slip vs. Time')
xlabel('Time (s)')
ylabel('Side Slip (deg)')
legend('No Tire Relaxation','With Tire Relaxation','Location','Best')
orient tall

