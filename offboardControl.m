classdef offboardControl < handle
    %Helper Class for ROS offboard control for the quadrotor
    %   Detailed explanation goes here

    properties
        WINDOWS_THRUST_AXIS = 3;
        LINUX_NEGATIVE_THRUST_AXIS = 3;
        LINUX_POSITIVE_THRUST_AXIS = 6;

        MAX_CALIBRATION_STD = 0.05;
        DEFAULT_GIMBAL_MODE = 1;

        joy
        tftree
        joy_msg
        ref_msg
        force_msg
        gimbal_msg
        game_msg
        
        joy_pub
        traj_pub
        force_pub
        gimbal_pub
        game_pub
        
        y_set
        x_set
        speed = 0;
        back
        start

        world_frame_id = 'world';
        quad_frame_id = 'vicon/Race1/Race1';
        gimbal_frame_id = 'vicon/Gimbal1/Gimbal1';
        quad_offset_trans = [0, 0, 0];
        dummy_transform = false;
    end

    methods
        function obj= offboardControl(ROS_IP, ROS_MASTER_IP, options)
            % constructor
            if nargin < 1
                ROS_IP = '192.168.1.3';
            end
            if nargin < 2
                ROS_MASTER_IP = '192.168.1.3';
            end
            if nargin < 3
                options = struct();
            end
            if ~isfield(options, 'use_gimbal')
                options.use_gimbal = false;
            end
            if isfield(options, 'quad_frame_id')
                obj.quad_frame_id = options.quad_frame_id;
            end
            if isfield(options, 'dummy_transform')
                obj.dummy_transform = options.dummy_transform;
            end

            setenv(['ROS_MASTER_URI', 'http://',ROS_MASTER_IP,':11311']);
            setenv('ROS_IP', ROS_IP);
            try
                rosinit(['http://',ROS_MASTER_IP, ':11311']);
            catch
                display('Ros is already started');
                rosshutdown
                rosinit(['http://',ROS_MASTER_IP, ':11311']);
            end
            fprintf('Ros master at %s, host IP: %s\n', ROS_MASTER_IP, ROS_IP)

            obj.tftree= rostf();
            pause(1);

            % publisher and for joypad and trajectory follower
            obj.game_pub=rospublisher('/game','std_msgs/Float32MultiArray','IsLatching', false);
            obj.joy_pub = rospublisher('/joy', 'sensor_msgs/Joy', 'IsLatching', false);
            %obj.traj_pub = rospublisher('/lqr_controller/reference', 'reference_trajectory/Reference', 'IsLatching', false);
            %obj.force_pub = rospublisher('/onboard_localization/controller_output', 'onboard_localization/ControllerOut', 'IsLatching', false);
            if options.use_gimbal
                obj.gimbal_pub = rospublisher('/onboard_localization/gimbal_setpoint', 'onboard_localization/GimbalSetpoint', 'IsLatching', false);
            end
            
            obj.game_msg=rosmessage(obj.game_pub);
            %obj.force_msg = rosmessage(obj.force_pub);
            obj.joy_msg = rosmessage(obj.joy_pub);
            %obj.ref_msg = rosmessage(obj.traj_pub);
            if options.use_gimbal
                obj.gimbal_msg = rosmessage(obj.gimbal_pub);
            end
        end

        function [trans, rot] = wait_for_quad_position(obj)
            if ~obj.dummy_transform
                waitForTransform(obj.tftree, obj.world_frame_id, obj.quad_frame_id);
            end
            [trans, rot] = obj.get_quad_position();
        end

        function [trans, rot] = get_quad_position(obj)
            %% receive position data from VICON
            if obj.dummy_transform
                trans = [0, 0, 0];
                rot = [0, 1, 0, 0];
            else
                tr = getTransform(obj.tftree, obj.world_frame_id, obj.quad_frame_id);

                trans = tr.Transform.Translation;
                rot = tr.Transform.Rotation;

                trans = [trans.X, trans.Y, trans.Z];
                rot = [rot.W, rot.X, rot.Y, rot.Z];
            end
            % Remove offset
            trans = trans - obj.quad_offset_trans;
        end

        function [trans, rot] = get_gimbal_position(obj)
            %% receive position data from VICON
            if obj.dummy_transform
                trans = [0, 0, 0];
                rot = [0, 1, 0, 0];
            else
                tr = getTransform(obj.tftree, obj.world_frame_id, obj.gimbal_frame_id);

                trans = tr.Transform.Translation;
                rot = tr.Transform.Rotation;

                trans = [trans.X, trans.Y, trans.Z];
                rot = [rot.W, rot.X, rot.Y, rot.Z];
            end
            % Remove offset
            trans = trans - obj.quad_offset_trans;
        end

        function calibrate_quad_offset(obj, nSamples)
            if nargin < 2
                nSamples = 10;
            end

            samples = zeros(nSamples, 3);
            for i = 1:nSamples
                % TODO: Also get offset on rotation?
                [trans, ~] = obj.wait_for_quad_position();
                samples(i, :) = trans;
            end
            offset_std = std(samples, 1);
            if any(offset_std > obj.MAX_CALIBRATION_STD)
                error(['Calibration failed. Standard deviation of measurements is too high: ', num2str(offset_std)]);
            end
            display(['Calibration succeeded. Standard deviation of measurements: ', num2str(offset_std)]);
            obj.quad_offset_trans = mean(samples, 1);
        end

        function sendJoy(obj,joy)
            %% send joy message to drone
            %% Joystick inputs
            X_axis=5;
            Y_axis=4;
            Z_axis=2;
            Yaw_axis=1;
            
            % sample the continuous joystick inputs
            obj.joy_msg.Axes=zeros(8,1);
            obj.joy_msg.Buttons=zeros(11,1);
            yaw = axis(joy, 1);    % left-right left
            if abs(axis(joy, 2)) > 0.05
                obj.speed=axis(joy, 2);  % x up-down left
            else
                obj.speed=0;
            end
            
            % HACK for Linux
            if isunix()
                neg_thrust= axis(joy, obj.LINUX_NEGATIVE_THRUST_AXIS); % thrust
                pos_thrust= axis(joy, obj.LINUX_POSITIVE_THRUST_AXIS); % thrust
                neg_thrust = (neg_thrust + 1) / 2.1;
                pos_thrust = (pos_thrust + 1) / 2.1;
                neg_thrust = 1 - neg_thrust;
                pos_thrust = 1 - pos_thrust;
            else
                thrust= axis(joy, obj.WINDOWS_THRUST_AXIS); % thrust
            end
            
            if abs(axis(joy, 4)) > 0.05
                obj.y_set=axis(joy, 4);%  left-right right
            else
                obj.y_set = 0;
            end
            
            if abs(axis(joy, 5)) > 0.05
                obj.x_set=axis(joy, 5); % up-down right
            else
                obj.x_set=0;
            end
            
            % sample the discrete joystick inputs
            A= button(joy, 1);     % A button
            B=  button(joy, 2);     % B button
            X=   button(joy, 3);     %x button
            Y=   button(joy, 4);     %y button
            
            button(joy, 5);     % LB left
            button(joy, 6);     % RB right
            
           obj.back= button(joy, 7);     % Back
            obj.start=button(joy, 8);     % start
            
            button(joy, 9);     % stick-push left
            button(joy, 10);    % stick-push right
                    
            obj.joy_msg.Buttons(1)=A;
            obj.joy_msg.Buttons(2)=B;
            
            obj.joy_msg.Buttons(3)=X;
            obj.joy_msg.Buttons(4)=Y;
            
            obj.joy_msg.Buttons(6)=button(joy, 6);
            obj.joy_msg.Buttons(5)=button(joy, 5);

            % HACK for Linux
            if isunix()
                thrust_left = neg_thrust;
                thrust_right = pos_thrust;
            else
                if thrust>=0
                    thrust_left=1-2*thrust;
                else
                    thrust_left=1;
                end
                if thrust<=0
                    thrust_right=1+2*thrust;
                else
                    thrust_right=1;
                end
            end
 
            obj.joy_msg.Axes(3)=thrust_left;
            obj.joy_msg.Axes(6)=thrust_right;
            
            obj.joy_msg.Axes(X_axis)=-obj.x_set;
            obj.joy_msg.Axes(Y_axis)=-obj.y_set;
            %
            obj.joy_msg.Axes(Yaw_axis)=-yaw;
            % on drone: yaw on axis(4); and xy on axis 1,2
            send(obj.joy_pub,obj.joy_msg); % sending the message
        end

              function sendGame(obj,vect)

            obj.game_msg.Data=vect;
            send(obj.game_pub,obj.game_msg); % sending the message
        end

        function sendRefPos(obj,setp)
            %% send joy message to drone
            %% Joystick inputs
            % sample the continuous joystick inputs
            obj.ref_msg.X = setp(1);
            obj.ref_msg.Y = setp(2);
            obj.ref_msg.Z = setp(3);
            obj.ref_msg.Yaw = 0;
            obj.ref_msg.Vx = setp(4);
            obj.ref_msg.Vy = setp(5);
            obj.ref_msg.Vz = setp(6);
            obj.ref_msg.Vyaw = 0;
            obj.ref_msg.Ax = setp(7);
            obj.ref_msg.Ay = setp(8);
            obj.ref_msg.Az = setp(9);
            obj.ref_msg.Ayaw = 0;
            
            send(obj.traj_pub,obj.ref_msg); % sending the message
        end
        
        function sendBodyForceAndYaw(obj, F_des, Yaw)
            %% send joy message to drone
            %% Joystick inputs
            % sample the continuous joystick inputs
            obj.force_msg.X = F_des(1);
            obj.force_msg.Y = F_des(2);
            obj.force_msg.Z = F_des(3);
            obj.force_msg.Yaw = Yaw;
            
            send(obj.force_pub,obj.force_msg); % sending the message
        end

        function sendGimbalSetpoint(obj, roll, pitch, yaw, gimbal_mode)
            if isempty(obj.gimbal_msg)
                warning('Offboard gimbal is not setup.');
                return;
            end
            if nargin < 5
                gimbal_mode = obj.DEFAULT_GIMBAL_MODE;
            end
            obj.gimbal_msg.Roll = roll;
            obj.gimbal_msg.Pitch = pitch;
            obj.gimbal_msg.Yaw = yaw;
            obj.gimbal_msg.GimbalMode = gimbal_mode;
            send(obj.gimbal_pub, obj.gimbal_msg);
        end
    end
    
end

