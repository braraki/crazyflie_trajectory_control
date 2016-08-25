% close all;
clear all;

%% Constants
TAKEOFF = 0;
FLYING = 1;
LANDING = 2;
OFF = 3;
% Height at which we switch from takeoff to flying or flying to landing
FLIGHT_SWITCH_HEIGHT = 0.1;

%% include paths for helper functions
addpath(genpath([fileparts(pwd),'/tools']));
addpath(genpath([fileparts(pwd)]));
vio_options();

q=[qx_p,qy_p,qz_p,qx_v,qy_v,qz_v,qx_i,qy_i,qz_i]; %state costs
r=[rx,ry,rz];% input costs
K_LQRp=genController(1/25,0.5,1,q,r);
% K_LQRp =...
%     [0.6         0         0    0.3        0         0    0.1107         0         0;
%      0    0.6         0         0    0.3         0         0    0.1107         0;
%      0         0    1.1894         0         0    0.5         0         0    0.3120];

% construct the offboard object
quadrotor = offboardControl(ROS_IP, ROS_MASTER_IP, offboard_options);


%% Define trajectory

    oc=1;
    % define the Quadrotor trajectory with a polygone
    %QuadrotorTray = impoly('Closed',oc);
    
    %xy_pos=getPosition(QuadrotorTray);
    xoff=0;
    yoff=0;
    zoff=0;
   
    
   
    
    height = 1;
    height_curve = 1;
    
    xyz_pos=[0,0,height;
    -1,1,height;
    0,1.5,height_curve;
    1,1,height;
    0,0,height;
    -1,-1,height;
    0,-1.5,height_curve;
    1,-1,height];

%     % rectangle
%     height = 0.8;
%     height_curve = height;
%     xyz_pos=[1,-1,height;
%     1,0,height;
%     1,1,height_curve;
%     0,1,height;
%     -1,1,height;
%     -1,0,height;
%     -1,-1,height_curve;
%     0,-1,height];
    xyz_pos = xyz_pos(:, [2,1,3]);

    % generate new trajectory
    quad_trajectory = trajectory(1,xyz_pos);


t1=clock;
cnt=0;

vis=visualizatio();


vis.plotPolyAndInter(quad_trajectory.traj_inter)

% % keyboard input
% kb_joy = KeyboardJoystick(gcf());
% joystick input
if joystick_index < 0
    joy = DummyJoystick();
else
    joy = vrjoystick(joystick_index);
end

% display('Press any key to start flight...');
% drawnow();
% pause();

%quadrotor.calibrate_quad_offset();

%% control loop
scope = Scope(3, 2);
[z_w, rot] = quadrotor.wait_for_quad_position();

flight_mode = TAKEOFF;
flight_mode = FLYING;
ramp_factor = 0;
thrust_offset_z = 0;
T_quad = 0;
T_target = 0;
T_end = inf;
display('Switching to takeoff mode.');
while flight_mode ~= OFF
    t2 = clock;
    e = etime(t2,t1);
    if e>dt
        t1=t2;
        freq=1/e;
        
        cnt=cnt+1;
        quadrotor.sendJoy(joy);
        
        % Get position and rotation
        [z_w, rot] = quadrotor.get_quad_position();
%         rot(1) = -rot(1); % vio sends JPL quaternions
        
       
        R_bw=RotFromQuatH(rot);
        
   
            a_ff=[0;0;0];
            m=0.5;
            F_lim=[3,3,10];
            speed_quad = quadrotor.speed*2.8;
            speed_target=1;
            T_quad=T_quad+e*speed_quad; % time intetration for trajectory race
            T_target=T_target+e*speed_target; % time intetration for trajectory race
            [inter_quad,setp_quad]=quad_trajectory.evaluate(T_quad,speed_quad); % quadrotor setpoints
            setp_yaw = pi/2;
            [F_des,F_des_c,yaw,x_out,Fi,Fp,yawspeed,q_out,w_out,YawLin,R_cw] = PositionControllerLQRrtN(rot,z_w,setp_quad,setp_yaw,[0;0;0],R_bw,m,F_lim,K_LQRp,'Vicon',dt,6);
           
            % sending
            % F_des(3) = F_des(3) + thrust_offset_z;
            F_des(3)=0.1;
           
            quadrotor.sendBodyForceAndYaw(F_des_c, yawspeed);
            disp(F_des_c)
            loop_flag=1;

        
        % Visualization
        if mod(cnt,8) == 1
            vis.updateVisualization(R_bw,z_w,setp_quad.p,[0;0;0],[0;0;0],[]);    

            scope.plot(cnt, [freq; 1; 2]);
            drawnow limitrate;
        end
        
    end
    
end