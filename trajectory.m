classdef trajectory
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        traj_inter
        traj_ppObj
        close_trajectory
    end

    methods
        function obj=trajectory(close_trajectory,points)
            % points have to be in the format:
            %points: [x1,y1,z1;
            %         x2,y2,z2;
            %         ........]
            
            if nargin<1
                close_trajectory=1;
            end
            if nargin<2
                points=[0;0;0];
            end
            
            if close_trajectory==0
                obj.traj_ppObj=cscvn(points');
                obj.traj_inter=fnplt(obj.traj_ppObj);
            else
                %If the first and last point coincide (and there are no other
                %repeated points), then a periodic cubic spline curve is constructed. However, double points result in corners.
                obj.traj_ppObj=cscvn([points;points(1,:)]');
                obj.traj_inter=fnplt(obj.traj_ppObj);
                obj.close_trajectory=close_trajectory;
            end
        end

        function [inter,setp]=evaluate(obj,T,velocityset)
            % INPUTS:
            % T: time where the spline has to be evaluated
            % velocityset: setpoint velocity on the spline
            %
            % OUTPUT:
            % inter: interpolated values on the trajectories [pos;pos';pos'']
            % inter: interpolated values on the trajectories [pos;pos';pos'']
            
            if nargin< 1
                T=0;
            end
            if nargin< 2
                velocityset=0;
            end
            coefs=obj.traj_ppObj.coefs; breaks=obj.traj_ppObj.breaks;  pieces=obj.traj_ppObj.pieces;
            
            seg=1;
            findT=0;
            if obj.close_trajectory==0
                % if trajectory is open
                if T<=0
                    T=0;
                end
                if T>=breaks(end)
                    T=breaks(end);
                end
                for i=1:(pieces)
                    a=breaks(i);
                    b=breaks(i+1);
                    if (T>=a && T<=b)
                        seg=i;
                        break
                    end
                    
                end
                findT=T;
            end
            
            if obj.close_trajectory==1
                % if trajectory is closed
                searchTc=mod(T,breaks(end));
                % find the right polynome segment
                for i=1:(pieces)
                    a=breaks(i);
                    b=breaks(i+1);
                    if (searchTc>=a && searchTc<=b)
                        seg=i;
                        break
                    end
                    
                end
                findT=searchTc;
            end
            T=(findT-breaks(seg));
            %% patameter "time" for evaluation
            % polynome coefs for x:
            kx3=coefs(3*seg-2,1); kx2=coefs(3*seg-2,2); kx1=coefs(3*seg-2,3); kx0=coefs(3*seg-2,4);
            % polynome coefs for y:
            ky3=coefs(3*seg-1,1); ky2=coefs(3*seg-1,2); ky1=coefs(3*seg-1,3); ky0=coefs(3*seg-1,4);
            % polynome coefs for z:
            kz3=coefs(3*seg,1); kz2=coefs(3*seg,2); kz1=coefs(3*seg,3); kz0=coefs(3*seg,4);
            
            % ploynom interpretation and 1st and 2nd derivatives
            x=[0;0;0]; v=[0;0;0]; a=[0;0;0];
            x(1)=kx3*T^3+kx2*T^2+kx1*T+kx0;
            x(2)=ky3*T^3+ky2*T^2+ky1*T+ky0;
            x(3)=kz3*T^3+kz2*T^2+kz1*T+kz0;
            
            v(1)=3*kx3*T^2+2*kx2*T+kx1;
            v(2)=3*ky3*T^2+2*ky2*T+ky1;
            v(3)=3*kz3*T^2+2*kz2*T+kz1;
            
            a(1)=6*kx3*T+2*kx2;
            a(2)=6*ky3*T+2*ky2;
            a(3)=6*kz3*T+2*kz2;
            
            setv=v*velocityset;
            seta=a*velocityset;
            % integrate time
            %T=T+dt*velocityset;
            
            setp.p=x; setp.v=setv;  setp.a=seta;
            inter.p=x; inter.p_d=v; inter.p_dd=a; inter.tangent=v;
        end
        
        
    end
    
    
    
    
end




