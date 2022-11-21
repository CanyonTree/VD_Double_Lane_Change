function [tfinal] = get_tfinal(data,tfinal_velocity)
%GET_T0 Summary of this function goes here
%   Detailed explanation goes here
velocity = data.IMU.Vel.X;
tfinal = data.IMU.Vel.Time(end);
for i = 1:length(velocity)
    if velocity(end+1-i)>tfinal_velocity
        tfinal = data.IMU.Vel.Time(end+1-i);
        return
    end
end

end

