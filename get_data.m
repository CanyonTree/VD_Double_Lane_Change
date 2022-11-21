function [data_out] = get_data(bag,t0_velocity, Steering_Ratio)
%GET_DATA Summary of this function goes here
%   Detailed explanation goes here

bag = rosbag(bag);

data_out = struct;

[Accel, Vel, Pos, AngVel] = get_IMU_Accelerometer_Data(bag);
data_out.IMU.Accel = Accel;
data_out.IMU.Vel = Vel;
data_out.IMU.Pos = Pos;
data_out.IMU.AngVel = AngVel;





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

data_out.IMU.Orient.Yaw = Yaw;
data_out.IMU.Orient.Pitch = Pitch;
data_out.IMU.Orient.Roll = Roll;
data_out.IMU.Orient.Time = timephi;

data_out.t0 = get_t0(data_out,t0_velocity);


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
    for i = 2:length(vel_x_IMU)
        pos_x_IMU(i) = pos_x_IMU(i-1) + vel_x_IMU(i)*(time_V_IMU(i)-time_V_IMU(i-1));
    end
    
    Pos.X = pos_x_IMU;

    i = 1;
    while vel_x_IMU(i) < 0.22
        i = i+1;
    end




end

