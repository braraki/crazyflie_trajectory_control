classdef CCrazyfly<handle
    %CCRAZYFLY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pos = [0;0;0];
        vel = [0;0;0];
        q = [0;0;0;1]
        pos_Setpoint = [0;0;0]
        
        ie = [0;0;0];
        tflistener
        KF
        
        setp_Force_pub
        setp_Force_msg
        
        setp_pos_sub
        
        pos_pub
        pos_msg
        
        id
        
        start
        flying
        land
    end
    
    methods
        function obj = CCrazyfly(id,dt)
            obj.id = id;
            if id == 6
                obj.pos_Setpoint=[1,;0;1];
            end
            if id == 2
                obj.pos_Setpoint=[-1,;0;1];
            end
            name = ['vicon/CF',num2str(id),'/CF',num2str(id)];
            obj.tflistener = RosTransformListener('world',name);
            obj.KF = kalmanTarget(dt);
            
            % obj.setp_Force_pub = rospublisher(['/q',num2str(quadID),'/bebop/camera_control'], 'geometry_msgs/Twist','IsLatching', false);
            obj.setp_Force_pub = rospublisher(['/q',num2str(id),'/cmd_vel'], 'geometry_msgs/Twist','IsLatching', false);
            obj.setp_Force_msg = rosmessage(rostype.geometry_msgs_Twist);
            
            obj.setp_pos_sub = rossubscriber(['/q',num2str(id),'/bebop/cmd_pos'], 'nav_msgs/Odometry','BufferSize', 1);
            %obj.setp_pos_msg = rosmessage(rostype.nav_msgs_Odometry);
            
            obj.pos_pub = rospublisher(['/q',num2str(id),'/bebop/state'], 'nav_msgs/Odometry','IsLatching', false);
            obj.pos_msg = rosmessage(rostype.nav_msgs_Odometry);
            
            [pos, q, age]  = obj.tflistener.getPose();
           % obj.pos_Setpoint = pos;
        end
        
        function  getPositionSetpoint(obj)
            msg= obj.setp_pos_sub.LatestMessage;
            if ~isempty(msg)
                x = msg.Pose.Pose.Position.X;
                y = msg.Pose.Pose.Position.Y;
                z = msg.Pose.Pose.Position.Z;
                obj.pos_Setpoint = [x;y;z];
            end
        end
        
         function  sendState(obj)
        
            obj.pos_msg.Pose.Pose.Position.X=obj.pos(1);
            obj.pos_msg.Pose.Pose.Position.Y=obj.pos(2);
            obj.pos_msg.Pose.Pose.Position.Z=obj.pos(3);
          
            obj.pos_msg.Twist.Twist.Linear.X = obj.vel(1);
            obj.pos_msg.Twist.Twist.Linear.Y = obj.vel(2);
            obj.pos_msg.Twist.Twist.Linear.Z = obj.vel(3);
            obj.pos_pub.send(obj.pos_msg);
         end
        
         function filterAndSendState(obj)
            [pos, q, age]  = obj.tflistener.getPose();
            obj.getPositionSetpoint();
            [pos,vel]=obj.KF.step(pos);
            
            obj.pos = pos;
            obj.vel = vel;
            obj.q = q;
            obj.sendState(); 
         end
        function step(obj,yaw,thrust,K_LQRp)
            pos = obj.pos;
            vel = obj.vel;
            %pos_Setpoint = obj.pos_Setpoint;
%             if obj.start == 1 
%                 if pos(3)<1
%                     if pos_Setpoint(3)<1
%                     pos_Setpoint = pos_Setpoint+0.001;
%                     end
%                 else
%                     obj.start = 0;
%                     obj.flying = 1;
%                 end
%             end
%             
%            if obj.land == 1 
%                 if pos(3)>0
%                     if pos_Setpoint(3)>=0
%                         pos_Setpoint = pos_Setpoint-0.001;
%                     end
%                 else
%                     obj.land = 0;
%                     obj.flying = 0;
%                 end
%             end
           %obj.pos_Setpoint=pos_Setpoint;
            q = obj.q;
            obj.sendState();
            Kp = K_LQRp(:,1:3)*diag([1;1;0.3]);
            Kd = K_LQRp(:,4:6)*diag([1;1;0.3]);
            Ki = K_LQRp(:,7:9);
            %% error in pos       
            pos_d = obj.pos_Setpoint;
            e_earth = (pos_d-pos);
            
           
     
            F_des = Kp*e_earth+Kd*([0;0;0]-vel);
            
            R_bw = RotFromQuatJ(q);
            R_updown = diag([1,1,1]);
            phi=atan2(R_bw(2,3),R_bw(3,3));
            theta=-asin(R_bw(1,3));
            psi=atan2(R_bw(1,2),R_bw(1,1));
            R_c = angle2dcm(0, 0, psi, 'XYZ');
            
            if pos(3)<0.3
                intReset = 1;
            else
                intReset = 0;
            end
            IntLim = [10;10;10];
            %% integrator
            dt = 1/50;
             e_body = R_c*e_earth;
            ie = obj.ie
           
            F_ff = [2.4;-0.3;0];
            if intReset==0
                if abs(ie(1))<IntLim(1)
                    ie(1)=ie(1)+e_body(1)*dt;
                else
                    ie(1)=ie(1);
                end
                if abs(ie(2))<IntLim(2)
                    ie(2)=ie(2)+e_body(2)*dt;
                else
                    ie(2)=ie(2);
                end
                if abs(ie(3))<IntLim(3)
                    ie(3)=ie(3)+e_body(3)*dt;
                else
                    ie(3)=ie(3);
                end
                
            
               
                
            else
                % If below height limit keep integrals cosntant
                ie=zeros(3,1);
            end
           obj.ie = ie;
            Fp=R_updown*R_c*F_des+[0;0;thrust] + Ki*ie*0 + Ki*F_ff;
%             obj.id
%            e_body
%             
%             pos_d
            if abs(Fp(1))>=2
                Fp(1)=sign(Fp(1))*2;
            end
            if abs(Fp(2))>=2
                Fp(2)=sign(Fp(2))*2;
            end
            if abs(Fp(3))>=2
                Fp(3)=sign(Fp(3))*2;
            end
            obj.SendAttitude(Fp(2),Fp(1),yaw,Fp(3))
        end
        
        function SendAttitude(obj,roll,pitch,yaw,thrust_scaled)
            
            thrust = thrust_scaled*50000 +5000;
            
            obj.setp_Force_msg.Linear.X = -roll*20;
            obj.setp_Force_msg.Linear.Y = -pitch*20;
            obj.setp_Force_msg.Angular.Z = -yaw*50;
            obj.setp_Force_msg.Linear.Z = thrust;
            obj.setp_Force_pub.send(obj.setp_Force_msg);
            
            
        end
    end
    
end

