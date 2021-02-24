rosbag_dir = "/home/takeshi/drone_data/rosbag/20210222/no_interference/";

rosbag_path_list = ["2320.bag" "2321.bag" "2322.bag" "2325.bag" "2329.bag" "2330.bag" "2331.bag"];

number_of_file = size(rosbag_path_list);
number_of_file = number_of_file(2);
% number_of_size = 1

all_roll_duty = [];
all_roll = [];
all_pitch_duty = [];
all_pitch = [];

all_duty_time = [0.0];
all_pose_time = [0.0];

for num=1:number_of_file
    rosbag_path = rosbag_dir + rosbag_path_list(num);
    
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
    
    mixed_time_mat = zeros(mixed_length-1, 1);
    mixed_z_mat = zeros(mixed_length-1, 1);
    mixed_roll_mat = zeros(mixed_length-1, 1);
    mixed_pitch_mat = zeros(mixed_length-1, 1);
    mixed_yaw_mat = zeros(mixed_length-1, 1);

    pose_time_mat = zeros(pose_length-1, 1);
    pose_z_mat = zeros(pose_length-1, 1);
    pose_q_w_mat = zeros(pose_length-1, 1);
    pose_q_x_mat = zeros(pose_length-1, 1);
    pose_q_y_mat = zeros(pose_length-1, 1);
    pose_q_z_mat = zeros(pose_length-1, 1);

    roll_mat = zeros(pose_length-1, 1);
    pitch_mat = zeros(pose_length-1, 1);
    yaw_mat = zeros(pose_length-1, 1);
    
    duty_last_sec = all_duty_time(end, 1);
    pose_last_sec = all_pose_time(end, 1);
    
    duty_time_from_init = zeros(mixed_length-1, 1);
    pose_time_from_init = zeros(pose_length-1, 1);

    for i = 1:mixed_length-1
       mixed_time_mat(i, 1) = cast(mixed_duty_struct{i+1}.Header.Stamp.Sec - init_sec, 'double') + (cast(mixed_duty_struct{i+1}.Header.Stamp.Nsec, 'double') - cast(init_nsec, 'double')) * 10e-10;
       mixed_z_mat(i, 1) = mixed_duty_struct{i+1}.Quaternion.W;
       mixed_roll_mat(i, 1) = mixed_duty_struct{i+1}.Quaternion.X;
       mixed_pitch_mat(i, 1) = mixed_duty_struct{i+1}.Quaternion.Y;
       mixed_yaw_mat(i, 1) = mixed_duty_struct{i+1}.Quaternion.Z;
       
       duty_time_from_init(i, 1) = mixed_time_mat(i, 1) + duty_last_sec;
    end

    for i = 1:pose_length-1
       pose_time_mat(i, 1) = cast(pose_struct{i}.Header.Stamp.Sec - init_sec, 'double') + (cast(pose_struct{i}.Header.Stamp.Nsec, 'double') - cast(init_nsec, 'double')) * 10e-10;
       pose_q_w_mat(i, 1) = pose_struct{i+1}.Pose.Orientation.W;
       pose_q_x_mat(i, 1) = pose_struct{i+1}.Pose.Orientation.X;
       pose_q_y_mat(i, 1) = pose_struct{i+1}.Pose.Orientation.Y;
       pose_q_z_mat(i, 1) = pose_struct{i+1}.Pose.Orientation.Z;
       
       pose_time_from_init(i, 1) = pose_time_mat(i, 1) + pose_last_sec;

       q0 = pose_struct{i+1}.Pose.Orientation.W;
       q1 = pose_struct{i+1}.Pose.Orientation.X;
       q2 = pose_struct{i+1}.Pose.Orientation.Y;
       q3 = pose_struct{i+1}.Pose.Orientation.Z;

       q0q0 = q0 * q0;
       q1q1q2q2 = q1 * q1 - q2 * q2;
       q3q3 = q3 * q3;
       roll_mat(i, 1)  = (atan2(2.0 * (q0 * q1 + q2 * q3), q0q0 - q1q1q2q2 + q3q3));
       pitch_mat(i, 1) = (-asin(2.0 * (q1 * q3 - q0 * q2)));
       yaw_mat(i, 1)   = (atan2(2.0 * (q1 * q2 + q0 * q3), q0q0 + q1q1q2q2 - q3q3));
    end
    all_roll = [all_roll; roll_mat];
    all_pitch = [all_pitch; pitch_mat];
    all_roll_duty = [all_roll_duty; mixed_roll_mat];
    all_pitch_duty = [all_pitch_duty; mixed_pitch_mat];
    all_duty_time = [all_duty_time; duty_time_from_init];
    all_pose_time = [all_pose_time; pose_time_from_init];
    
end

min_size = 2274;

all_pose_time = all_pose_time(2: min_size+1, 1);
all_roll = all_roll(1: min_size, 1);
all_duty_time = all_duty_time(1: min_size, 1);
all_roll_duty = all_roll_duty(1: min_size, 1);
all_pose_time = all_pose_time(1: min_size, 1);
all_pitch = all_pitch(1: min_size, 1);
all_duty_time = all_duty_time(1: min_size, 1);
all_pitch_duty = all_pitch_duty(1: min_size, 1);

%plot(pose_time_mat, roll_mat)
plot(all_pose_time, all_roll)
hold on
%plot(mixed_time_mat, mixed_roll_mat)
plot(all_duty_time, all_roll_duty)
legend('roll', 'duty')

figure()
%plot(pose_time_mat, pitch_mat)
plot(all_pose_time, all_pitch)
hold on
%plot(mixed_time_mat, mixed_pitch_mat)
plot(all_duty_time, all_pitch_duty)
legend('pitch', 'duty')


