function [sim] = car_sim_loop_set_steer(delta,Vel,tinitial,tfinal,X0,specs)
%CAR_SIM Summary of this function goes here
%   Detailed explanation goes here

dt = 0.01; %s
i = 1;
mu = specs.mu;
% X0=[East0 (m)  North0 (m)  Heading0 (rad) yaw_rate0 (rad/s)];
GPS(i,:) = [X0(1), X0(2), X0(3), Vel(i), 0];
East(i) = GPS(i,1);
North(i) = GPS(i,2);
Heading(i) = GPS(i,3);
Speed(i) = GPS(i,4);
Course(i) = GPS(i,5);



yaw_gyro(i) = X0(4);
delta_enc(i) = 0;

t = tinitial:dt:tfinal;


i = 2;

if (specs.use_duggoff == 1)
    [GPS(i,:),yaw_gyro(i),delta_enc(i)] = run_MKZ(delta(i-1), Vel(i), X0, mu);
else
    [GPS(i,:),yaw_gyro(i),delta_enc(i)] = run_MKZ(delta(i-1), Vel(i), X0);
end

East(i) = GPS(i,1); 
North(i) = GPS(i,2); 
Heading(i) = GPS(i,3); 
Speed(i) = GPS(i,4); 
Course(i) = GPS(i,5);

for i = 3:length(t)

    if (specs.use_duggoff == 1)
        [GPS(i,:),yaw_gyro(i),delta_enc(i)] = run_MKZ(delta(i), Vel(i));
    else
        [GPS(i,:),yaw_gyro(i),delta_enc(i)] = run_MKZ(delta(i), Vel(i));
    end

    % GPS=[East (m)  North(m)  Heading(rad) Speed (m/s) Course(rad)]
    East(i) = GPS(i,1); 
    North(i) = GPS(i,2); 
    Heading(i) = GPS(i,3); 
    Speed(i) = GPS(i,4); 
    Course(i) = GPS(i,5);
end


sim = struct;
sim.East = East;
sim.North = North;
sim.Heading = Heading;
sim.Speed = Speed;
sim.Course = Course;
sim.Yaw_Gyro = yaw_gyro;
sim.Delta_Enc = delta_enc;
sim.Delta = delta;
sim.Time = t;

save("sim","sim")
clear all
load("sim")
delete("sim.mat")

end

