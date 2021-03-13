% rosbag_path = "/home/takeshi/drone_data/matlab/20210303/1456.bag";
rosbag_path = "/home/takeshi/drone_data/rosbag/20210222/no_interference/2322.bag";
bag = rosbag(rosbag_path);

% input = select(bag, 'Topic', '/control_input');
input = select(bag, 'Topic', '/mixed_duty');
output = select(bag, 'Topic', '/pose');

input_struct = readMessages(input, 'DataFormat', 'struct');
output_struct = readMessages(output, 'DataFormat', 'struct');

init_sec = output_struct{1}.Header.Stamp.Sec;
init_nsec = output_struct{1}.Header.Stamp.Nsec;

input_length = size(input_struct);
input_length = input_length(1);

output_length = size(output_struct);
output_length = output_length(1);

input_time = zeros(input_length, 1);
input_z = zeros(input_length, 1);
input_roll = zeros(input_length, 1);
input_pitch = zeros(input_length, 1);
input_yaw = zeros(input_length, 1);

output_time = zeros(output_length, 1);
output_z_mat = zeros(output_length, 1);
pose_q_w_mat = zeros(output_length, 1);
pose_q_x_mat = zeros(output_length, 1);
pose_q_y_mat = zeros(output_length, 1);
pose_q_z_mat = zeros(output_length, 1);

output_x = zeros(output_length, 1);
output_y = zeros(output_length, 1);
output_z = zeros(output_length, 1);

for i = 1:input_length
   input_time(i, 1) = cast(input_struct{i}.Header.Stamp.Sec - init_sec, 'double') + (cast(input_struct{i}.Header.Stamp.Nsec, 'double') - cast(init_nsec, 'double')) * 10e-10;
   input_z(i, 1) = input_struct{i}.Quaternion.W;
   input_roll(i, 1) = input_struct{i}.Quaternion.X;
   input_pitch(i, 1) = input_struct{i}.Quaternion.Y;
   input_yaw(i, 1) = input_struct{i}.Quaternion.Z;
end

for i = 1:output_length
   pose_time_mat(i, 1) = cast(output_struct{i}.Header.Stamp.Sec - init_sec, 'double') + (cast(output_struct{i}.Header.Stamp.Nsec, 'double') - cast(init_nsec, 'double')) * 10e-10;
   pose_q_w_mat(i, 1) = output_struct{i}.Pose.Orientation.W;
   pose_q_x_mat(i, 1) = output_struct{i}.Pose.Orientation.X;
   pose_q_y_mat(i, 1) = output_struct{i}.Pose.Orientation.Y;
   pose_q_z_mat(i, 1) = output_struct{i}.Pose.Orientation.Z;
   
   q0 = output_struct{i}.Pose.Orientation.W;
   q1 = output_struct{i}.Pose.Orientation.X;
   q2 = output_struct{i}.Pose.Orientation.Y;
   q3 = output_struct{i}.Pose.Orientation.Z;
   
   q0q0 = q0 * q0;
   q1q1q2q2 = q1 * q1 - q2 * q2;
   q3q3 = q3 * q3;
   output_x(i, 1)  = (atan2(2.0 * (q0 * q1 + q2 * q3), q0q0 - q1q1q2q2 + q3q3));
   output_y(i, 1) = (-asin(2.0 * (q1 * q3 - q0 * q2)));
   output_z(i, 1)   = (atan2(2.0 * (q1 * q2 + q0 * q3), q0q0 + q1q1q2q2 - q3q3));
   output_time(i, 1) = cast(output_struct{i}.Header.Stamp.Sec - init_sec, 'double') + (cast(output_struct{i}.Header.Stamp.Nsec, 'double') - cast(init_nsec, 'double')) * 10e-10;
%     output_x(i, 1) = output_struct{i}.Vector.X;
%     output_y(i, 1) = output_struct{i}.Vector.Y;
%     output_z(i, 1) = output_struct{i}.Vector.Z;
end

plot(output_time, output_x)
hold on
plot(input_time, input_roll)
legend('roll', 'duty')

figure()
plot(output_time, output_y)
hold on
plot(input_time, input_pitch)
legend('pitch', 'duty')

