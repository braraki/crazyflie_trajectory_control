classdef kalmanTarget<handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x
        P
        F
        H
        
        R
        Q
        
    end
    
    methods
        function obj = kalmanTarget(dt)
            qx = 10;
            qy = 10;
            qz = 10;
            
            rx = 0.01;
            ry = 0.01;
            rz = 0.01;
            
            obj.x =[0;0;0;0;0;0];
            obj.F = [1,0,0,dt,0,0;
                    0,1,0,0,dt,0;
                    0,0,1,0,0,dt;
                    0,0,0,1,0,0;
                    0,0,0,0,1,0;
                    0,0,0,0,0,1];
            obj.P = 10*eye(6);
            obj.H = [1,0,0,0,0,0;
                    0,1,0,0,0,0;
                    0,0,1,0,0,0];
             
             obj.Q = diag([0,0,0,qx,qy,qz]);
             obj.R = diag([rx,ry,rz]);
        end
        
        function prediction(obj)
           obj.x = obj.F*obj.x;
           obj.P = obj.F*obj.P*obj.F'+obj.Q;
        end
        
        function update(obj,z)
            x = obj.x;
            F = obj.F;
            H = obj.H;
            P = obj.P;
            R = obj.R;
            
            y = z - H*x;
            S = H*P*H'+R;
            K = P*H'/S;
            x = x + K*y;
            P = (eye(6) - K*H)*P;
            obj.x = x;
            obj.P = P;
        end
        
        function [pos,vel] = step(obj,z)
            obj.prediction();
            obj.update(z);
            pos = obj.x(1:3);
            vel = obj.x(4:6);
        end
        
        function [pos_N,vel_N] = N_steps(obj,z,N)
            obj.prediction();
            obj.update(z);
            x = zeros(6,N);
            
            x(:,1) = obj.x;
            for i = 1:(N-1)
                x(:,i+1) = obj.F*x(:,i);
            end
            pos_N = x(1:3,:);
            vel_N = x(4:6,:);
        end
    end
    
end

