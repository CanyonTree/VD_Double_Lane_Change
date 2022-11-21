function [t0] = get_t0(data,t0_velocity)
%GET_T0 Summary of this function goes here
%   Detailed explanation goes here
velocity = data.IMU.Vel.X;
t0 = data.IMU.Vel.Time(1);
for i = 1:length(velocity)
    if velocity(i)>t0_velocity
        t0 = data.IMU.Vel.Time(i);
        return
    end
end

end

