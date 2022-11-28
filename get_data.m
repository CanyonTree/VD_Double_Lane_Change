function [data_out] = get_data(bag,t0_velocity,tfinal_velocity, Steering_Ratio)
%GET_DATA Summary of this function goes here
%   Detailed explanation goes here

bag = rosbag(bag);

data_out = struct;

[Accel, Vel, Pos, AngVel] = get_IMU_Accelerometer_Data(bag);


data_out.IMU.Accel = Accel;
data_out.IMU.Vel = Vel;
data_out.IMU.Pos = Pos;
data_out.IMU.AngVel = AngVel;

GPSpos = get_GPS(bag);
data_out.GPS.Pos = GPSpos;


%% Get steering angle
topicdel = select(bag,'Topic','/vehicle/steering_report');
structdel = readMessages(topicdel,'DataFormat','struct');

for i = 1:length(structdel)
    del(i) = structdel{i}.SteeringWheelAngle;
    timedel(i) = getTime(structdel,i);
end

data_out.Steering.SteeringWheelAngle = del/Steering_Ratio;
data_out.Steering.Time = timedel;

%% get yaw/pitch/roll
% topicphi = select(bag,'Topic','/etalin/Imu');
% structphi = readMessages(topicphi,'DataFormat','struct');
% for i = 1:length(structphi)
%     [yaw(i),pitch(i),roll(i)] = quat2angle([structphi{i}.Orientation.W,structphi{i}.Orientation.X,structphi{i}.Orientation.Y,structphi{i}.Orientation.Z]);
%     timephi(i) = getTime(structphi,i);
% end
% 
% data_out.IMU.Orient.Yaw = yaw;
% data_out.IMU.Orient.Pitch = pitch;
% data_out.IMU.Orient.Roll = roll;
% data_out.IMU.Orient.Time = timephi;

topicang = select(bag,'Topic','/etalin/attitude');
structang = readMessages(topicang,'DataFormat','struct');
for i = 1:length(structang)
%     [yaw(i),pitch(i),roll(i)] = [structang{i}.BodyAttitude.Yaw, structang{i}.BodyAttitude.Pitch, structang{i}.BodyAttitude.Roll];
    Yaw(i) = structang{i}.BodyAttitude.Yaw;
    Pitch(i) = structang{i}.BodyAttitude.Pitch;
    Roll(i) = structang{i}.BodyAttitude.Roll;
    timephi(i) = getTime(structang,i);
end

Orient = struct;
Orient.Yaw = Yaw;
Orient.Pitch = Pitch;
Orient.Roll = Roll;
Orient.Time = timephi;

    t0 = get_t0(data_out,t0_velocity);
    tfinal = get_tfinal(data_out,tfinal_velocity);
    data_out.t0 = t0;


    
    del = interp1(timedel-data_out.t0, del, AngVel.Time - data_out.t0);
    
    Steering = struct;
    Steering.SteeringWheelAngle = del;
    Steering.Time = timedel;

    Accel.X = interp1(Accel.Time - data_out.t0, Accel.X, AngVel.Time-data_out.t0); % m/s^2
    Accel.Y = interp1(Accel.Time - data_out.t0, Accel.Y, AngVel.Time-data_out.t0); % m/s^2
    Accel.Z = interp1(Accel.Time - data_out.t0, Accel.Z, AngVel.Time-data_out.t0); % m/s^2

    Vel.X = interp1(Vel.Time - data_out.t0, Vel.X, AngVel.Time-data_out.t0); % m/s^2
    Vel.Y = interp1(Vel.Time - data_out.t0, Vel.Y, AngVel.Time-data_out.t0); % m/s^2
    Vel.Z = interp1(Vel.Time - data_out.t0, Vel.Z, AngVel.Time-data_out.t0); % m/s^2

    Pos.X = interp1(Pos.Time - data_out.t0, Pos.X, AngVel.Time-data_out.t0); % m/s^2
    Pos.Y = interp1(Pos.Time - data_out.t0, Pos.Y, AngVel.Time-data_out.t0); % m/s^2
    Pos.Z = interp1(Pos.Time - data_out.t0, Pos.Z, AngVel.Time-data_out.t0); % m/s^2
    
    AngVel.Yaw = cut_data(AngVel.Yaw,AngVel.Time,t0,tfinal);
    AngVel.Pitch = cut_data(AngVel.Pitch,AngVel.Time,t0,tfinal);
    AngVel.Roll = cut_data(AngVel.Roll,AngVel.Time,t0,tfinal);

    Accel.X = cut_data(Accel.X,AngVel.Time,t0,tfinal);
    Accel.Y = cut_data(Accel.Y,AngVel.Time,t0,tfinal);
    Accel.Z = cut_data(Accel.Z,AngVel.Time,t0,tfinal);

    Vel.X = cut_data(Vel.X,AngVel.Time,t0,tfinal);
    Vel.Y = cut_data(Vel.Y,AngVel.Time,t0,tfinal);
    Vel.Z = cut_data(Vel.Z,AngVel.Time,t0,tfinal);

    Pos.X = cut_data(Pos.X,AngVel.Time,t0,tfinal);
    Pos.Y = cut_data(Pos.Y,AngVel.Time,t0,tfinal);
    Pos.Z = cut_data(Pos.Z,AngVel.Time,t0,tfinal);
    
    Steering.SteeringWheelAngle = cut_data(Steering.SteeringWheelAngle,AngVel.Time,t0,tfinal);
    Steering.SteerAngle = Steering.SteeringWheelAngle/Steering_Ratio;
    
    Time = AngVel.Time-data_out.t0;
    Time = cut_data(Time,AngVel.Time,t0,tfinal);
    
    Accel.Units = "m/s^2";
    Vel.Units = "m/s";
    Pos.Units = "m";
    AngVel.Units = "deg/s";
    Orient.Units = "deg";
    Steering.Units = "rad";

    data_out.IMU.Accel = Accel;
    data_out.IMU.Vel = Vel;
    data_out.IMU.Pos = Pos;
    data_out.IMU.AngVel = AngVel;
    data_out.IMU.Orient = Orient;
    data_out.Steering = Steering;
    data_out.Time = Time;

end


function [Accel, Vel, Pos, AngVel] = get_IMU_Accelerometer_Data(bag)

    topic_IMU_V = select(bag,'Topic','/etalin/velocity');
    struct_IMU_V = readMessages(topic_IMU_V,'DataFormat','struct');
    topic_IMU_a = select(bag,'Topic','/etalin/acceleration');
    struct_IMU_a = readMessages(topic_IMU_a,'DataFormat','struct');
    topic_IMU_AngVel = select(bag,'Topic','/etalin/angular_velocity');
    struct_IMU_AngVel = readMessages(topic_IMU_AngVel,'DataFormat','struct');


    topic_steering_report = select(bag,'Topic','/vehicle/ulc_report');
    struct_steering_report = readMessages(topic_steering_report,'DataFormat','struct');
    
    
    Accel = struct;
    Vel = struct;
    Pos = struct;

    for i = 1:length(struct_IMU_a)
        acc_x_IMU(i) = struct_IMU_a{i}.Acceleration.X;
        acc_y_IMU(i) = struct_IMU_a{i}.Acceleration.Y;
        acc_z_IMU(i) = struct_IMU_a{i}.Acceleration.Z;
        time_acc_IMU(i) = getTime(struct_IMU_a,i);
    end
    
    Accel.X = acc_x_IMU;
    Accel.Y = acc_y_IMU;
    Accel.Z = acc_z_IMU;
    Accel.Time = time_acc_IMU;

    for i = 1:length(struct_IMU_AngVel)
        roll_vel(i) = struct_IMU_AngVel{i}.AngularVelocity.Roll;
        yaw_vel(i) = struct_IMU_AngVel{i}.AngularVelocity.Yaw;
        pitch_vel(i) = struct_IMU_AngVel{i}.AngularVelocity.Pitch;
        time_Ang_Vel(i) = getTime(struct_IMU_AngVel,i);
    end
    
    AngVel.Roll = roll_vel;
    AngVel.Yaw = yaw_vel;
    AngVel.Pitch = pitch_vel;
    AngVel.Time = time_Ang_Vel;

    for i = 1:length(struct_IMU_V)
        vel_x_IMU(i) = struct_IMU_V{i}.Velocity.X;
        vel_y_IMU(i) = struct_IMU_V{i}.Velocity.Y;
        vel_z_IMU(i) = struct_IMU_V{i}.Velocity.Z;
        time_V_IMU(i) = getTime(struct_IMU_V,i);
    end
    
    Vel.X = vel_x_IMU;
    Vel.Y = vel_z_IMU;
    Vel.Z = vel_y_IMU;
    Vel.Time = time_V_IMU;


    pos_x_IMU = 0;
    pos_y_IMU = 0;
    pos_z_IMU = 0;
    for i = 2:length(vel_x_IMU)
        pos_x_IMU(i) = pos_x_IMU(i-1) + vel_x_IMU(i)*(time_V_IMU(i)-time_V_IMU(i-1));
        pos_y_IMU(i) = pos_y_IMU(i-1) + vel_y_IMU(i)*(time_V_IMU(i)-time_V_IMU(i-1));
        pos_z_IMU(i) = pos_z_IMU(i-1) + vel_z_IMU(i)*(time_V_IMU(i)-time_V_IMU(i-1));
    end
    Pos.X = pos_x_IMU;
    Pos.Y = pos_y_IMU;
    Pos.Z = pos_z_IMU;
    Pos.Time = time_V_IMU;
    
    






    pos_y_IMU = 0;
    for i = 2:length(vel_y_IMU)
        pos_y_IMU(i) = pos_y_IMU(i-1) + vel_y_IMU(i)*(time_V_IMU(i)-time_V_IMU(i-1));
    end
    Pos.Y = pos_y_IMU;

end

function [Pos] = get_GPS(bag)
    topicGPSp = select(bag,'Topic','vehicle/gps/fix');
    structGPSp = readMessages(topicGPSp,'DataFormat','struct');

    Pos = struct;
    

    for i = 1:length(structGPSp)
        Lat(i) = structGPSp{i}.Latitude;
        Long(i) = structGPSp{i}.Longitude;
        time(i) = getTime(structGPSp,i);
    end

    Pos.Latitude = Lat;
    Pos.Longitude = Long;
    Pos.Time = time;
end
