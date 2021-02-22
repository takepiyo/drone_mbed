
duty_low = 0.20;
duty_high = 0.43;
length = 25;

pnSequence = comm.PNSequence('SamplesPerFrame',length);

duty0 = (duty_high - duty_low) * pnSequence() + duty_low;
duty1 = (duty_high - duty_low) * pnSequence() + duty_low;
duty2 = (duty_high - duty_low) * pnSequence() + duty_low;
duty3 = (duty_high - duty_low) * pnSequence() + duty_low;

rosinit('http://localhost:11311');

duty_0_pub = rospublisher('/input_duty_0', 'std_msgs/Float32');
duty_1_pub = rospublisher('/input_duty_1', 'std_msgs/Float32');
duty_2_pub = rospublisher('/input_duty_2', 'std_msgs/Float32');
duty_3_pub = rospublisher('/input_duty_3', 'std_msgs/Float32');

msg_0 = rosmessage(duty_0_pub);
msg_1 = rosmessage(duty_1_pub);
msg_2 = rosmessage(duty_2_pub);
msg_3 = rosmessage(duty_3_pub);

for i = 1:length
    msg_0.Data = 0.0;
    msg_1.Data = 0.0;
    msg_2.Data = 0.0;
    msg_3.Data = 0.0;
    send(duty_0_pub, msg_0);
    send(duty_1_pub, msg_1);
    send(duty_2_pub, msg_2);
    send(duty_3_pub, msg_3);
    pause(0.1);
end

for i = 1:length
    msg_0.Data = 0.3;
    msg_1.Data = 0.3;
    msg_2.Data = 0.3;
    msg_3.Data = 0.3;
    send(duty_0_pub, msg_0);
    send(duty_1_pub, msg_1);
    send(duty_2_pub, msg_2);
    send(duty_3_pub, msg_3);
    pause(0.1);
end

for i = 1:length
    msg_0.Data = duty0(i,1);
    msg_1.Data = duty1(i,1);
    msg_2.Data = duty2(i,1);
    msg_3.Data = duty3(i,1);
    send(duty_0_pub, msg_0);
    send(duty_1_pub, msg_1);
    send(duty_2_pub, msg_2);
    send(duty_3_pub, msg_3);
    pause(0.1);
end
for i = 1:length
    msg_0.Data = 0.0;
    msg_1.Data = 0.0;
    msg_2.Data = 0.0;
    msg_3.Data = 0.0;
    send(duty_0_pub, msg_0);
    send(duty_1_pub, msg_1);
    send(duty_2_pub, msg_2);
    send(duty_3_pub, msg_3);
    pause(0.1);
end

rosshutdown;