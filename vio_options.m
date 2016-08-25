%% Options

% Use external trajectory
use_external_trajectory = false;

% Use manual gimbal control
manual_gimbal_control = true;

% Use local roscore?
roscore_local = false;

% Which joystick port to use

joystick_index = 1;

% Offboard options
offboard_options = struct();
offboard_options.quad_frame_id = 'body';


offboard_options.dummy_transform = false;
offboard_options.use_gimbal = false;
if roscore_local
    offboard_options.dummy_transform = true;
else
    offboard_options.dummy_transform = false;
end
offboard_options.dummy_transform = false;

% Visualization options
vis_options = struct();
vis_options.plot_gimbal = true;

% Define ROS parameters
if roscore_local
    ROS_IP = '127.0.0.1';
    ROS_MASTER_IP = '127.0.0.1';
else
%     [~, ROS_IP] = system('ifconfig | grep -Eo ''inet (addr:)?([0-9]*\.){3}[0-9]*'' | grep -Eo ''([0-9]*\.){3}[0-9]*'' | grep -v ''127.0.0.1''');
%     ROS_IP = '192.168.1.2';
%     ROS_IP = '192.168.1.249';
    ROS_IP = getenv('ROS_IP');
    ROS_MASTER_IP = '192.168.1.101';
end


%% define cost parameters for controller tuning
dt=1/25;

qx_p=2;
qx_v=4;
qx_i=0.1;
rx=8;

% z state and input costs
qz_p=3;
qz_v=3;
qz_i=0.5;
rz=5;
qy_p=qx_p;
qy_v=qx_v;
qy_i=qx_i;
ry=rx;

useGui=0;
