classdef OffboardControlBroadcaster < handle
    %OffboardControlBroadcaster
    
    properties
        WINDOWS_THRUST_AXIS = 3;
        LINUX_NEGATIVE_THRUST_AXIS = 3;
        LINUX_POSITIVE_THRUST_AXIS = 6;
        
        MAX_CALIBRATION_STD = 0.05;
        DEFAULT_GIMBAL_MODE = 1;
        
        joy
        joy_msg
        ref_msg
        force_msg
        
        joy_pub
        force_pub
        
        y_set
        x_set
        speed = 0;
        back
        start
    end
    
    methods
        function obj = OffboardControlBroadcaster()
            obj.joy_pub = rospublisher('/joy', 'sensor_msgs/Joy', 'IsLatching', false);
            obj.force_pub = rospublisher('/onboard_localization/controller_output', 'onboard_localization/ControllerOut', 'IsLatching', false);
            obj.force_msg = rosmessage(obj.force_pub);
            obj.joy_msg = rosmessage(obj.joy_pub);
        end
        
        function sendJoy(obj,joy)
            %% send joy message to drone
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
                b = 2/(1-0.0476);
                a = 1-b;
                thrust_left = a + b*neg_thrust;
                thrust_right = a + b*pos_thrust;
%                 disp([thrust_left thrust_right])
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
        
        function sendBodyForceAndYaw(obj, F_des, Yaw)
            %% send joy message to drone
            obj.force_msg.X = F_des(1);
            obj.force_msg.Y = F_des(2);
            obj.force_msg.Z = F_des(3);
            obj.force_msg.Yaw = Yaw;
            
            send(obj.force_pub,obj.force_msg); % sending the message
        end
        
    end
    
end

