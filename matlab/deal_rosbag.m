rosbag_path = "/home/takeshi/drone_data/rosbag/20210222/no_interference/2321.bag";
bag = rosbag(rosbag_path);

mixed_duty = select(bag, 'Topic', '/mixed_duty');
pose = select(bag, 'Topic', '/pose');

mixed_duty_struct = readMessages(mixed_duty, 'DataFormat', 'struct');
pose_struct = readMessages(pose, 'DataFormat', 'struct');

init_sec = pose_struct{1}.Header.Stamp.Sec;
init_nsec = pose_struct{1}.Header.Stamp.Nsec;

mixed_length = size(mixed_duty_struct);
mixed_length = mixed_length(1);

pose_length = size(pose_struct);
pose_length = pose_length(1);

mixed_time_mat = zeros(mixed_length, 1);
mixed_z_mat = zeros(mixed_length, 1);
mixed_roll_mat = zeros(mixed_length, 1);
mixed_pitch_mat = zeros(mixed_length, 1);
mixed_yaw_mat = zeros(mixed_length, 1);

pose_time_mat = zeros(pose_length, 1);
pose_z_mat = zeros(pose_length, 1);
pose_q_w_mat = zeros(pose_length, 1);
pose_q_x_mat = zeros(pose_length, 1);
pose_q_y_mat = zeros(pose_length, 1);
pose_q_z_mat = zeros(pose_length, 1);

roll_mat = zeros(pose_length, 1);
pitch_mat = zeros(pose_length, 1);
yaw_mat = zeros(pose_length, 1);

for i = 1:mixed_length
   mixed_time_mat(i, 1) = cast(mixed_duty_struct{i}.Header.Stamp.Sec - init_sec, 'double') + (cast(mixed_duty_struct{i}.Header.Stamp.Nsec, 'double') - cast(init_nsec, 'double')) * 10e-10;
   mixed_z_mat(i, 1) = mixed_duty_struct{i}.Quaternion.W;
   mixed_roll_mat(i, 1) = mixed_duty_struct{i}.Quaternion.X;
   mixed_pitch_mat(i, 1) = mixed_duty_struct{i}.Quaternion.Y;
   mixed_yaw_mat(i, 1) = mixed_duty_struct{i}.Quaternion.Z;
end

for i = 1:pose_length
   pose_time_mat(i, 1) = cast(pose_struct{i}.Header.Stamp.Sec - init_sec, 'double') + (cast(pose_struct{i}.Header.Stamp.Nsec, 'double') - cast(init_nsec, 'double')) * 10e-10;
   pose_q_w_mat(i, 1) = pose_struct{i}.Pose.Orientation.W;
   pose_q_x_mat(i, 1) = pose_struct{i}.Pose.Orientation.X;
   pose_q_y_mat(i, 1) = pose_struct{i}.Pose.Orientation.Y;
   pose_q_z_mat(i, 1) = pose_struct{i}.Pose.Orientation.Z;
   
   q0 = pose_struct{i}.Pose.Orientation.W;
   q1 = pose_struct{i}.Pose.Orientation.X;
   q2 = pose_struct{i}.Pose.Orientation.Y;
   q3 = pose_struct{i}.Pose.Orientation.Z;
   
   q0q0 = q0 * q0;
   q1q1q2q2 = q1 * q1 - q2 * q2;
   q3q3 = q3 * q3;
   roll_mat(i, 1)  = (atan2(2.0 * (q0 * q1 + q2 * q3), q0q0 - q1q1q2q2 + q3q3));
   pitch_mat(i, 1) = (-asin(2.0 * (q1 * q3 - q0 * q2)));
   yaw_mat(i, 1)   = (atan2(2.0 * (q1 * q2 + q0 * q3), q0q0 + q1q1q2q2 - q3q3));
end

plot(pose_time_mat, roll_mat)
hold on
plot(mixed_time_mat, mixed_roll_mat)
legend('roll', 'duty')

figure()
plot(pose_time_mat, pitch_mat)
hold on
plot(mixed_time_mat, mixed_pitch_mat)
legend('pitch', 'duty')

