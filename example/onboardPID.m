% use the onboard PID node (position_pid_control_node) to control the quad,
% set setpoints from matlab

% if the ROS core does not run on this PC, add the appropriate IP to the
% below rosinit calls
disp('Starting ROS node')
try
    rosinit
catch
    rosshutdown
    rosinit
end

%%
% change the state of the drone. first arm, then set to controller mode
state_pub = rospublisher(['/mav_core/command/state'], 'std_msgs/UInt8');
pause(1)
state_msg = rosmessage(state_pub);
state_msg.Data = 1;
state_pub.send(state_msg);
pause(1)
state_msg.Data = 2;
state_pub.send(state_msg);

%% create the setpoint publisher
pos_setp_pub = rospublisher(['/mav_core/command/setpoint'], 'std_msgs/Float64MultiArray','IsLatching', false);
pause(1)
pos_setp_msg = rosmessage(pos_setp_pub);

%% send a position reference to the position controller
setp_pos = [0 0 1]';
setp_vel = [0 0 0]';
setp_acc = [0 0 14]'; % assume the drone is about 0.5 kg
setp_yaw = 0;

pos_setp_msg.Data = [setp_pos; setp_vel; setp_acc; setp_yaw];
pos_setp_pub.send(pos_setp_msg);
%%
pause(3)

return
%% fly a square
for i = 1:1
    idx = mod(i, 4);
    if idx == 1
        setp_pos = [1 1 1]';
    elseif idx == 2
        setp_pos = [1 -1 1]';
    elseif idx == 3
        setp_pos = [-1 -1 1]';
    else
        setp_pos = [-1 1 1]';
    end
    setp_yaw = setp_yaw + pi/4;
%     setp_yaw = pi/2;
%     setp_yaw = mod(setp_yaw, 2*pi)
    pos_setp_msg.Data = [setp_pos; setp_vel; setp_acc; setp_yaw];
    pos_setp_pub.send(pos_setp_msg);
    pause(3);
end
%%
setp_pos = [0 0 1]';
setp_yaw = pi/2;
pos_setp_msg.Data = [setp_pos; setp_vel; setp_acc; setp_yaw];
pos_setp_pub.send(pos_setp_msg);

%% return to starting position
setp_pos = [0 0 1]';
pos_setp_msg.Data = [setp_pos; setp_vel; setp_acc; setp_yaw];
pos_setp_pub.send(pos_setp_msg);


%% 
return

%% land
setp_pos = [0 0 1]';
setp_vel = [0 0 0]';
setp_acc = [0 0 10]'; % assume the drone is about 0.5 kg
setp_yaw = 0;

pos_setp_msg.Data = [setp_pos; setp_vel; setp_acc; setp_yaw];
pos_setp_pub.send(pos_setp_msg);
pause(1)

setp_pos = [0 0 0]';
setp_vel = [0 0 0]';
setp_acc = [0 0 0]'; % assume the drone is about 0.5 kg
setp_yaw = 0;

pos_setp_msg.Data = [setp_pos; setp_vel; setp_acc; setp_yaw];
pos_setp_pub.send(pos_setp_msg);

%% switch to manual control
state_msg.Data = 1;
state_pub.send(state_msg);

%% disarm
state_msg.Data = 0;
state_pub.send(state_msg);