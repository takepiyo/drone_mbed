reference_freq = 13;

delta_t = 1 / 13

[resampled_output_roll, resampled_output_time] = resample(output_x, output_time, reference_freq);
[resampled_output_pitch, resampled_output_time] = resample(output_y, output_time, reference_freq);

[resampled_input_roll, resampled_input_time] = resample(input_roll, input_time, reference_freq);
[resampled_input_pitch, resampled_input_time] = resample(input_pitch, input_time, reference_freq);

plot(resampled_output_time,resampled_output_roll,resampled_input_time, resampled_input_roll)
legend('roll', 'duty')
figure()
plot(resampled_output_time,resampled_output_pitch,resampled_input_time, resampled_input_pitch)
legend('pitch', 'duty')
% 
% min_size = 243;
% resampled_input_roll = resampled_input_roll(1: min_size, 1);
% resampled_input_pitch = resampled_input_pitch(1: min_size, 1);