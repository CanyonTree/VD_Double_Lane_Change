function [sim_output,zeta,omega_n,poles] = Step_Car_Sim(V,rdes,tfinal,car_params)
%CAR_SIM_STEP Summary of this function goes here
%   Detailed explanation goes here

dt = car_params.dt;
t = 0:dt:tfinal;

caf = car_params.caf;
car = car_params.car;
Iz = car_params.Iz;
a = car_params.a;
b = car_params.b;
m = car_params.m;

c0 = caf + car;
c1 = a*caf - b*car;
c2 = (a^2)*caf + (b^2)*car;

B_r = (a*caf*c0 - c1*caf)/(m*V);
Meff = Iz;
Beff = (c0*Iz + m*c2)/(m*V);
Keff = (c0*c2)/(m*V^2) - c1 - (c1^2/(m*V^2));

s1 = (-Beff + sqrt(Beff^2 - 4 * Meff*Keff))/(2*Meff);
s2 = (-Beff - sqrt(Beff^2 - 4 * Meff*Keff))/(2*Meff);
poles = [s1,s2];
zeta = Beff/(2*sqrt(Keff*Meff));
omega_n = sqrt(Keff/Meff);

del = (Keff/B_r)*rdes;

data = zeros([length(t),5]);
data(:,1) = t;
data(:,2) = V;
data(:,3) = del;

sim_output = car_sim_loop(data,car_params);
sim_output(1).zeta = zeta;
sim_output(1).omega_n = omega_n;
sim_output(1).poles = poles;

end

