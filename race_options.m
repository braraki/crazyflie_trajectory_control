%% Options

% Use external trajectory
use_external_trajectory = false;

% Use manual gimbal control
manual_gimbal_control = true;

% Use local roscore?
roscore_local = false;

% Which joystick port to use

joystick_index = 1;

% define the quad ID
quad_id = 1;

% define race or camera
race_flag=1;

% Offboard options
offboard_options = struct();
offboard_options.quad_frcame_id = sprintf('vicon/Race%d/Race%d', quad_id, quad_id);


offboard_options.dummy_transform = false;
offboard_options.race_flag=race_flag;
offboard_options.use_gimbal = false;
if roscore_local
    offboard_options.dummy_transform = true;
else
    offboard_options.dummy_transform = false;
end
% offboard_options.dummy_transform = false;

% Visualization options
vis_options = struct();
vis_options.plot_gimbal = true;

% Define ROS parameters
if roscore_local
    ROS_IP = '127.0.0.1';
    ROS_MASTER_IP = '127.0.0.1';
else
    [~, ROS_IP] = system('ifconfig | grep -Eo ''inet (addr:)?([0-9]*\.){3}[0-9]*'' | grep -Eo ''([0-9]*\.){3}[0-9]*'' | grep -v ''127.0.0.1''');
%     ROS_IP = '192.168.1.2';
    ROS_IP = '192.168.1.249';
    ROS_MASTER_IP = '192.168.1.101';
end

gazebo=1;
%% define cost parameters for controller tuning
dt=1/100;
% xy state and input costs
qx_p=5;
qx_v=4;
qx_i=1;
rx=8;

%% for racer1
if offboard_options.race_flag
qx_p=2;
qx_v=2;
qx_i=0.1;
rx=8;
end

% z state and input costs
qz_p=3;
qz_v=3;
qz_i=0.5;
rz=5;
if gazebo
qx_p=5;
qx_v=5;
qx_i=0.1;
rx=1;

qz_p=1;
qz_v=1;
qz_i=0.01;
rz=5;
end


qy_p=qx_p;
qy_v=qx_v;
qy_i=qx_i;
ry=rx;

useGui=0;

try
    race_options_local;
catch
end

