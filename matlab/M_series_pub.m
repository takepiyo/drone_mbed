
duty_max = 0.2;
length = 50;

pnSequence = comm.PNSequence('SamplesPerFrame',length);

duty1 = pnSequence() * duty_max;
duty2 = pnSequence() * duty_max;
duty3 = pnSequence() * duty_max;
duty4 = pnSequence() * duty_max;

rosinit('http://localhost:11311');

% duty_pub = rospublisher('/input_duty', 'std_msgs/Float32');
duty_pub = rospublisher('/input_duty', 'std_msgs/Float32');
for i = 1:length
    msg = rosmessage(duty_pub);
    msg.Data = duty1(i,1);
%     msg.Data(2) = duty2(i,1);
%     msg.Data(3) = duty3(i,1);
%     msg.Data(4) = duty4(i,1);
    send(duty_pub, msg);
    pause(0.1);
end
for i = 1:length
    msg = rosmessage(duty_pub);
    msg.Data = 0.0;
%     msg.Data(2) = 0.0;
%     msg.Data(3) = 0.0;
%     msg.Data(4) = 0.0;
    send(duty_pub, msg);
    pause(0.1);
end

rosshutdown;