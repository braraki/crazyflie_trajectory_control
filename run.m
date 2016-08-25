% close all;
clear all;
clear all;
clear classes

%% add some paths
addpath(genpath(fullfile((fileparts((pwd))), 'tools')))
m=0.0408;
dt = 1/50;
q=[1,1,1,1,1,1,0.1,0.1,0.1];
r=[1,1,1];
[K_LQRp]= genController(dt,m,0,q,r);
%% Constants
TAKEOFF = 0;
FLYING = 1;
LANDING = 2;
OFF = 3;

START_BUTTON=1;
LAND_BUTTON=2;
MANUAL_BUTTON=4;
AUTO_BUTTON=3;
RESET_BUTTON=7;
X_VEL_AXIS=4;
Y_VEL_AXIS=5;
Z_VEL_AXIS=2;
YAW_SPEED_AXIS=7;
THRUST_UP = 6;
THRUST_DOWN = 3;
ROLL_AXIS=5;
PITCH_AXIS=3;

CROSS_LR = 1;
CROSS_UD = 2;
% Height at which we switch from takeoff to flying or flying to landing
FLIGHT_SWITCH_HEIGHT = 0.1;

%% include paths for helper functions
% CFrace_options();
%
% q=[qx_p,qy_p,qz_p,qx_v,qy_v,qz_v,qx_i,qy_i,qz_i]; %state costs
% r=[rx,ry,rz];% input costs
% K_LQRp=genController(dt,0.5,1,q,r);


%% setup ros
ROS_MASTER_IP = '192.168.1.3';
ROS_IP = '192.168.1.3'
setenv('ROS_MASTER_URI', ['http://',ROS_MASTER_IP,':11311']);
setenv('ROS_IP', ROS_IP);
try
    rosinit;
    
catch
    display('Ros is already started');
    rosshutdown
    rosinit;
end

%% define the publisher and listener
Joy_sub = rossubscriber('/joy','sensor_msgs/Joy', 'BufferSize', 1);



t1=clock;
cnt=0;

numDrones = 2;
Drone(1) = CCrazyfly(6,dt);
Drone(2) = CCrazyfly(2,dt);
t1=clock;
thrust = [0;0];
auto = [1,1,0,0];

scp1 = Scope(3, 2);

cnt = 0;

activeDrone = 1;
setp_start = [0;0;0];
while(true)
    t2 = clock;
    e = etime(t2,t1);
    if e>dt
        t1=t2;
        freq=1/e;
        msg=Joy_sub.LatestMessage;
        for cntDrone = 1:numDrones
            Drone(cntDrone).filterAndSendState();
        end
        %pos
        if ~isempty(msg)
            if msg.Buttons(START_BUTTON)==1
                Drone(activeDrone).start = 1;
            end
            if msg.Buttons(LAND_BUTTON)==1
                Drone(activeDrone).land = 1;
            end
            %
            %             if msg.Buttons(LAND_BUTTON)==1
            %                 land_pub.send(land_msg);
            %             end
            
            if msg.Axes(CROSS_UD)==1
                activeDrone = 1;
            end
            if msg.Axes(CROSS_LR)==-1
                activeDrone = 2;
            end
%             if msg.Axes(CROSS_UD)==-1
%                 activeDrone = 3;
%             end
%             if msg.Axes(CROSS_LR)==1
%                 activeDrone = 4;
%             end
            %activeDrone
            if msg.Buttons(AUTO_BUTTON)==1
                auto(activeDrone) = 1;
            end
            if msg.Buttons(MANUAL_BUTTON)==1
                auto(activeDrone)= 0;
            end
            
            thrust_up = -(msg.Axes(THRUST_UP)-1)/2;
            thrust_down = (msg.Axes(THRUST_DOWN)-1)/2;
            thrust(activeDrone) = thrust(activeDrone) + 0.01*(thrust_up+thrust_down);
            if thrust(activeDrone) >=1
                thrust(activeDrone) = 1;
            else if thrust(activeDrone)<=0
                    thrust(activeDrone) = 0;
                end
            end
            
            roll    = msg.Axes(X_VEL_AXIS);
            pitch   = msg.Axes(Y_VEL_AXIS);
            yaw     = msg.Axes(YAW_SPEED_AXIS)
            
            thrust
            
        end
        
        
%             if auto(1) ==0
%                 Drone(1).SendAttitude(roll,pitch,yaw,thrust(2));
%             else
                Drone(1).step(yaw,thrust(1),K_LQRp);
           % end
           % if auto(2) ==0
            %    Drone(2).SendAttitude(roll,pitch,yaw,thrust(1));
          %  else
                Drone(2).step(yaw,thrust(2),K_LQRp);
         %   end
            
        
        cnt=cnt+1;
        scp1.plot(cnt, Drone(2).vel);
        Drone(1).pos_Setpoint;
        Drone(2).pos_Setpoint;
%         display('drone 1')
%         Drone(1).pos
%         display('drone 2')
%         Drone(2).pos
         drawnow limitrate
    end;
    %    cnt=cnt+1;
end