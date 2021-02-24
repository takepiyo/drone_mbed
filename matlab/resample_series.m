reference_freq = 13;

delta_t = 1 / 13

[output_roll, resampled_pose_time] = resample(all_roll, all_pose_time, reference_freq);
[output_pitch, resampled_pose_time] = resample(all_pitch, all_pose_time, reference_freq);

[input_roll, resampled_duty_time] = resample(all_roll_duty, all_duty_time, reference_freq);
[input_pitch, resampled_duty_time] = resample(all_pitch_duty, all_duty_time, reference_freq);

plot(resampled_pose_time,output_roll,resampled_duty_time, input_roll)
figure()
plot(resampled_pose_time,output_pitch,resampled_duty_time, input_pitch)

min_size = 2125;
input_roll = input_roll(1: min_size, 1);
input_pitch = input_pitch(1: min_size, 1);