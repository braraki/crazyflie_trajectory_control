function vio_offboard_control()
% Offboard control based on VIO pose estimates, with VICON backup
roscore_local = false;
use_vio   = true; % set to false to not wait for vio pose estimate
use_vicon = false; % set to false to not wait for vio pose estimate

clear PositionControllerLQRrtN

t1 = clock;
% [~, ROS_IP] = system('ifconfig | grep -Eo ''inet (addr:)?([0-9]*\.){3}[0-9]*'' | grep -Eo ''([0-9]*\.){3}[0-9]*'' | grep -v ''127.0.0.1''');
% ROS_IP = ROS_IP(1:end-1);
% setenv('ROS_IP', ROS_IP);

if roscore_local
    ROS_MASTER_IP = getenv('ROS_IP');
else
    ROS_MASTER_IP = '192.168.1.101';
end
ROS_MASTER_URI = sprintf('http://%s:11311', ROS_MASTER_IP);

rosshutdown
fprintf('ROS_MASTER_URI: %s\n', ROS_MASTER_URI)
rosinit(ROS_MASTER_URI);
global vio_vel
vio_vel = zeros(3,1);

if use_vio
    vio_tf = RosTransformListener('world', 'camera');
    vio_body_tf = RosTransformListener('world', 'body');
    [vio_pos, vio_att] = vio_body_tf.getPose();
    vio_vel_sub = rossubscriber('/vio_ros/vel',@vio_vel_cb, 'BufferSize', 1);
else
    vio_pos = zeros(3,1);
    vio_att = [0 0 0 1]';
end

R = RotFromQuatJ(vio_att);
vio_yaw = -atan2(R(1,2), R(1,1));

if use_vicon
    vicon_tf = RosTransformListener('world', 'vicon/Race1/Race1');
    [vicon_pos, vicon_att] = vicon_tf.getPose();
    vicon_att = quatInv(vicon_att); % vicon sends hamiltonians?
    R = RotFromQuatJ(vicon_att);
    vicon_yaw = -atan2(R(1,2), R(1,1));
else
    vicon_pos = zeros(3,1);
    vicon_att = [0 0 0 1]';
end
R = RotFromQuatJ(vicon_att);
vicon_yaw = -atan2(R(1,2), R(1,1));

if use_vicon
    setpoint.p = vicon_pos;
    setpoint.yaw = vicon_yaw;
else
    setpoint.p = vio_pos;
    setpoint.yaw = vio_yaw;
end
setpoint.v = zeros(3,1);

vicon_origin.pos = zeros(3,1); % transform from vicon world frame to control world frame
vicon_origin.att = eye(3);
    
if use_vicon
   
    % transform from vio world frame to control world frame
    vio_origin.att = getYawRotFromRot(RotFromQuatJ(vio_att))' * vicon_origin.att * getYawRotFromRot(RotFromQuatJ(vicon_att));
    vio_origin.pos = vicon_origin.att' * vicon_pos + vicon_origin.pos - vio_origin.att' * vio_pos;
    
else
    vio_origin.pos = zeros(3,1);
    vio_origin.att = eye(3);
end

vio_tf_age = 0;
vicon_tf_age = 0;

loop_rate = Rate(100);

if use_vicon
    control_with_vio = false;
else
    control_with_vio = true;
end

VIO_BUTTON = 5;
VICON_BUTTON = 6;
AXIS_YAW = 1;
AXIS_Z = 2;
AXIS_X = 5;
AXIS_Y = 4;

dead_zone = 0.2;

ref_change_gain = 0.02;

race_options;

q=[qx_p,qy_p,qz_p,qx_v,qy_v,qz_v,qx_i,qy_i,qz_i]; %state costs
r=[rx,ry,rz];% input costs
m=0.5;
K_LQRp_vicon=genController(dt,0.5,1,q,r);


K_LQRp_vio =...
    [0.6         0         0    0.3        0         0    0.1107         0         0;
     0    0.6         0         0    0.3         0         0    0.1107         0;
     0         0    1.1894         0         0    0.5         0         0    0.3120];

control_broadcaster = OffboardControlBroadcaster();

joy = vrjoystick(joystick_index);

%% set up the figure
figure(1); clf
% Pose(2, [], [], 'world');~

if use_vio
    vio_pose_vis = Pose(2, [], [], 'cam');
    vio_body_pose_vis = Pose(2, [], [], 'vio');
%     vio_setpoint_vis = Pose(1, [], 'rcc', 'vio setpoint');
end
if use_vicon
    vicon_pose_vis = Pose(2, [], [], 'VICON');
%     vicon_setpoint_vis = Pose(1, [], 'rbb', 'vicon setpoint');
end

% setpoint_vis = Pose(1, [], 'rbb', 'setpoint');

axis square
axis([-5 5 -5 5 -0.5 3])
grid on
set(gca,'DataAspectRatio', [1 1 1])
labels = {{'Loop rate', ''}, {'vio age', 'vicon age'}, {'vicon vel x', 'vio vel x'}, {'vicon vel y', 'vio vel y'}, {'vicon vel z', 'vio vel z'}};
colors = {lines(2), lines(2), lines(2), lines(2)};

% scope = Scope([2, 2, 2, 2, 2],  2, labels);
scope = Scope([2, 2],  2, {{'Loop rate', ''}, {'vicon x', 'vio x'}});
% scope = Scope([2, 3],  2);

%% loop
if control_with_vio
    ros_warn('Using VIO for control')
else
    ros_warn('Using VICON for control')
end

loop_cnt = 1;
while true
    t2 = clock;
    elapsed = etime(t2, t1);
    t1 = t2;
    
    % get the pose estimates and update plot
    
    if use_vio
        [vio_cam_pos, vio_cam_att] = vio_tf.getPose();
        vio_cam_att = quatInv(vio_cam_att); % tfs are hamiltonian?
        [vio_pos, vio_att, vio_tf_age] = vio_body_tf.getPose();
        vio_att = quatInv(vio_att); % tfs are hamiltonian?
        R = RotFromQuatJ(vio_att);
        vio_yaw = -atan2(R(1,2), R(1,1));
        if any(isnan(vio_pos)) || any(isnan(vio_att))
            fprintf('GOT NANS!')
        end
    end
    
    if use_vicon
        [vicon_pos, vicon_att, vicon_tf_age] = vicon_tf.getPose();
        vicon_att = quatInv(vicon_att); % vicon sends hamiltonian?
        R = RotFromQuatJ(vicon_att);
        vicon_yaw = -atan2(R(1,2), R(1,1));
    end
    
    % change the reference point according to the joystick
    dx = -axis(joy, AXIS_X);
    if abs(dx) < dead_zone
        dx = 0;
    end
    dy = -axis(joy, AXIS_Y);
    if abs(dy) < dead_zone
        dy = 0;
    end
    dz = -axis(joy, AXIS_Z);
    if abs(dz) < dead_zone
        dz = 0;
    end
    dyaw = -axis(joy, AXIS_YAW);
    if abs(dyaw) < dead_zone
        dyaw = 0;
    end
    
    if control_with_vio
        R_wb = [cos(vio_yaw) -sin(vio_yaw), 0;
            sin(vio_yaw), cos(vio_yaw), 0;
            0, 0, 1];
        pos_change = R_wb * [dx; dy; dz] * ref_change_gain;
        setpoint.p = setpoint.p + pos_change;
        setpoint.v = pos_change;
        setpoint.yaw = setpoint.yaw + dyaw * ref_change_gain;
    else
        R_wb = [cos(vicon_yaw), -sin(vicon_yaw), 0;
            sin(vicon_yaw), cos(vicon_yaw), 0;
            0, 0, 1];
        pos_change = R_wb * [dx; dy; dz] * ref_change_gain;
        setpoint.p = setpoint.p + pos_change;
        setpoint.v = pos_change;
        setpoint.yaw = setpoint.yaw + dyaw * ref_change_gain;
    end
%     disp(setpoint.v)
    
    % check if pose estimate source is being switched
    if control_with_vio
        if button(joy, VICON_BUTTON) && use_vicon
            ros_warn('Switching to VICON for control')
            control_with_vio = false;
            vicon_origin.att = getYawRotFromRot(RotFromQuatJ(vicon_att))' * vio_origin.att * getYawRotFromRot(RotFromQuatJ(vio_att));
            vicon_origin.pos = vio_origin.att' * vio_pos + vio_origin.pos - vicon_origin.att' * vicon_pos;
        end
    else
        if button(joy, VIO_BUTTON) && use_vio
            ros_warn('Switching to VIO for control')
            control_with_vio = true;
            vio_origin.att = getYawRotFromRot(RotFromQuatJ(vio_att))' * vicon_origin.att * getYawRotFromRot(RotFromQuatJ(vicon_att));
            vio_origin.pos = vicon_origin.att' * vicon_pos + vicon_origin.pos - vio_origin.att' * vio_pos;
        end
    end
    
    vio_pos_ctrl = vio_origin.att' * vio_pos + vio_origin.pos;
    vio_att_ctrl = QuatFromRotJ(RotFromQuatJ(vio_att) * vio_origin.att);
    
    vicon_pos_ctrl = vicon_origin.att' * vicon_pos + vicon_origin.pos;
    vicon_att_ctrl = QuatFromRotJ(RotFromQuatJ(vicon_att) * vicon_origin.att);
    
    if control_with_vio
        pos = vio_pos_ctrl;
        att = [-vio_att_ctrl(4); vio_att_ctrl(1:3)];
        K_LQRp = K_LQRp_vio;
    else
        pos = vicon_pos_ctrl;
        att = [-vicon_att_ctrl(4); vicon_att_ctrl(1:3)];
        K_LQRp = K_LQRp_vicon;
    end
    
    % controller wants hamiltonian?
    [~,F_des_c,~,x_out,~,~,yawspeed,~,~,~,~] = PositionControllerLQRrtN(att, pos, setpoint, setpoint.yaw, [0;0;0],[],m,[],K_LQRp,[],dt,6);

    control_broadcaster.sendJoy(joy);

    F_des_c(3) = max(0.05, F_des_c(3));
    control_broadcaster.sendBodyForceAndYaw(F_des_c, yawspeed);
    
    if mod(loop_cnt, 8) == 0
        vicon_vel_ctrl = vicon_origin.att' * x_out(4:6);
        vio_vel_ctrl = vio_origin.att' * vio_vel;
%         scope.plot(loop_cnt, [1/elapsed; 0; vio_tf_age; vicon_tf_age; vicon_vel_ctrl(1); vio_vel_ctrl(1); vicon_vel_ctrl(2); vio_vel_ctrl(2); vicon_vel_ctrl(3); vio_vel_ctrl(3)]);
        scope.plot(loop_cnt, [1/elapsed; 0; vicon_vel_ctrl(1); vio_vel_ctrl(1)]);
%         scope.plot(loop_cnt, [1/elapsed; 0; F_des_c]);
        
        if use_vio
            vio_pose_vis.setPose(vio_cam_pos, quatInv(vio_cam_att));
            vio_body_pose_vis.setPose(vio_pos_ctrl, quatInv(vio_att_ctrl));
        end
        if use_vicon
            vicon_pose_vis.setPose(vicon_pos_ctrl, quatInv(vicon_att_ctrl));
        end
        
        R_setpoint = [cos(setpoint.yaw), -sin(setpoint.yaw), 0; sin(setpoint.yaw), cos(setpoint.yaw), 0; 0, 0, 1];
%         setpoint_vis.setPose(setpoint.p, R_setpoint');

        drawnow limitrate
    end
    loop_rate.sleep()
    loop_cnt = loop_cnt + 1;
    
end
end

function vio_vel_cb(~, callbackdata)
global vio_vel

vio_vel(1) = callbackdata.X;
vio_vel(2) = callbackdata.Y;
vio_vel(3) = callbackdata.Z;
end

function R_yaw = getYawRotFromRot(R)
yaw = -atan2(R(1,2), R(1,1));
R_yaw = [cos(yaw) -sin(yaw), 0;
    sin(yaw), cos(yaw), 0;
    0, 0, 1];
end