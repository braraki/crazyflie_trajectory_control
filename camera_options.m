%% Options

% Use external trajectory
use_external_trajectory = true;

% Receive gimbal transform?
receive_gimbal_position = false;
% Use manual gimbal control
manual_gimbal_control = false;

% Use local roscore?
roscore_local = false;

% Which joystick port to use

joystick_index = -1;

% define the quad ID
quad_id = 1;
quad_name = 'Race';
% quad_name = 'Solo';
% Offboard options
offboard_options = struct();
offboard_options.quad_frame_id = sprintf('vicon/%s%d/%s%d', quad_name, quad_id, quad_name, quad_id);

offboard_options.race_flag = false;
offboard_options.use_gimbal = true;
offboard_options.dummy_transform = false;

offboard_options.use_gimbal = true;
if roscore_local
    offboard_options.dummy_transform = true;
else
    offboard_options.dummy_transform = false;
end


% Visualization options
vis_options = struct();
vis_options.plot_gimbal = true;

% Define ROS parameters
if roscore_local
    ROS_IP = '127.0.0.1';
    ROS_MASTER_IP = '127.0.0.1';
else
    ROS_IP = '192.168.1.200';
    ROS_MASTER_IP = '192.168.1.200';
end


%% define cost parameters for controller tuning
dt=1/100;
% xy state and input costs
qx_p=2;
qx_v=2;
qx_i=0.1;
rx=8;

%% for racer1
if strcmp(quad_name, 'Race')
    display('Using race gains.');
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

qy_p=qx_p;
qy_v=qx_v;
qy_i=qx_i;
ry=rx;

useGui=0;
