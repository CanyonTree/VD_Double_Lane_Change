% Tire Relaxation Test
clc; clear all; close all;

A = 0.5;
B = 1;

Calpha = 90000;
alpha = 0.5;
dt = 0.0001;
tfinal = 1;
t = 0:dt:tfinal;
V = 10;

xfinal = V*tfinal;
dx = 0.00001;
x = 0:dx:xfinal;

Fy(1) = 0;
for i = 2:length(x)
    Fy_prime(i-1) = (alpha*Calpha - B*Fy(i-1))/A;
    Fy(i) = Fy(i-1) + Fy_prime(i-1)*dx;
end

figure
plot(x,Fy)


clear Fy_prime Fy
Fy(1) = 0;
for i = 2:length(t)
    Fy_prime(i-1) = (alpha*Calpha - B*Fy(i-1))/(A/V);
    Fy(i) = Fy(i-1) + Fy_prime(i-1)*dt;
end

figure
plot(t,Fy)