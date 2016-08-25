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
race_options();

q=[qx_p,qy_p,qz_p,qx_v,qy_v,qz_v,qx_i,qy_i,qz_i]; %state costs
r=[rx,ry,rz];% input costs
K_LQRp=genController(dt,0.5,1,q,r);

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
   
    
   
    
    height = 0.8;
    height_curve = 1;
    
    xyz_pos=[0,0,height;
    -1.5,1.5,height;
    0,2.5,height_curve;
    1.5,1.5,height;
    0,0,height;
    -1.5,-1.5,height;
    0,-2.5,height_curve;
    1.5,-1.5,height];
    xyz_pos = xyz_pos(:, [2,1,3]);

    % generate new trajectory
    quad_trajectory = trajectory(1,xyz_pos);
    target_trajectory = trajectory(1,[1,1,1;
        2,2,3;
        3,3,2]);


t1=clock;
cnt=0;

vis=visualizatio();


vis.plotPolyAndInter(quad_trajectory.traj_inter)
vis.plotPolyAndInter(target_trajectory.traj_inter)

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
display('Trying to grab quad transform ...');
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
        
       
        R_bw=RotFromQuatH(rot);
        
   
            a_ff=[0;0;0];
            m=0.5;
            F_lim=[3,3,10];
            speed_quad = quadrotor.speed*2.8;
            speed_target=1;
            speed_quad = 0;
            T_quad=T_quad+e*speed_quad; % time intetration for trajectory race
            T_target=T_target+e*speed_target; % time intetration for trajectory race
            [inter_quad,setp_quad]=quad_trajectory.evaluate(T_quad,speed_quad); % quadrotor setpoints
            [inter_target,setp_target]=target_trajectory.evaluate(T_target,speed_target);  % target setpoints
            setp_yaw = pi/2;
            [F_des,F_des_c,yaw,x_out,Fi,Fp,yawspeed,q_out,w_out,YawLin,R_cw] = PositionControllerLQRrtN(rot,z_w,setp_quad,setp_yaw,[0;0;0],R_bw,m,F_lim,K_LQRp,'Vicon',dt,6);
           
            % sending
            % F_des(3) = F_des(3) + thrust_offset_z;
            F_des(3)=0.1;
            disp(F_des_c)
           
            quadrotor.sendBodyForceAndYaw(F_des_c, yawspeed);
            loop_flag=1;
           
            

        setp_gimbal.theta = 0;
        setp_gimbal.phi = 0;
        manual_gimbal_control=0;
        if offboard_options.use_gimbal
            if manual_gimbal_control
                roll = kb_joy.axis_values(1);
                pitch = kb_joy.axis_values(2);
                
                setp_gimbal.theta=quadrotor.x_set;
                setp_gimbal.phi=quadrotor.y_set;
                yaw = kb_joy.axis_values(3);
                str = sprintf('roll=%d, pitch=%d, yaw=%d', roll, pitch, yaw);
                display(str);
            else
                setp_gimbal.theta = 0;
                setp_gimbal.phi = 0;
                rel_postion = x_out(1:3) - 0;
                x_out(1:3)
                % todo: hack with angles
                setp_gimbal.phi = atan2(rel_postion(2), rel_postion(1)) + pi;
                setp_gimbal.theta = atan2(rel_postion(3), norm(rel_postion(1:2))) ;
                setp_gimbal.psi=atan2(rel_postion(2), rel_postion(1)) + pi;
                rel_postion = x_out(1:3) - 0;
                [ yaw_speed_gimbal ] = Gimbal_LQG( rot_gimbal,R_cw,rel_postion,dt );
                quadrotor.sendGimbalSetpoint(0, -setp_gimbal.theta, yaw_speed_gimbal*500);
            end
            roll=0; pitch=0; yaw=0;
            rot_gimbal = eul2rotm([yaw, pitch, roll]);
            vis.updateGimbal(rot_gimbal);
            
        end
        
        
        
        
        % Visualization
        if mod(cnt,8) == 1
            vis.updateVisualization(R_bw,z_w,setp_quad.p,setp_target.p,[0;0;0],setp_gimbal);
            % cam visualization
            % todo: hack with angles
            rel_postion = setp_quad.p - setp_target.p;
            
            % todo: hack with angles
            setp_gimbal.psi = atan2(rel_postion(2), rel_postion(1)) + pi;
            setp_gimbal.theta = atan2(rel_postion(3), norm(rel_postion(1:2))) + pi/2;
            
            R_cam = angle2dcm(0,  setp_gimbal.theta+pi/2, -setp_gimbal.psi+pi, 'XYZ');
            vis.updateCamera(R_cam,setp_quad.p);
            scope.plot(cnt, [freq; 1; 2]);
            drawnow limitrate;
        end
        
    end
    
end
