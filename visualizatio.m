classdef visualizatio<handle
    %VISUALIZATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        plot_3d
        plot_2d
        q1
        q2
        q3
        h_x
        plot_human
        plot_gimbal
        q1_gimbal
        q2_gimbal
        q3_gimbal
        target_setp
        quad_setp
        q_yaw
        
        nCols
        nRows
        
        % interpolated traj
        traj_inter_plot2d
        traj_inter_plot3d
        
        % camera plot
        camera
        % parameters
        sim
    end
    
    methods

        function obj = visualizatio(fig, sim, scale, options)
            if nargin < 1 || isempty(fig)
                fig = figure(1); clf;
            end
            if nargin < 2 || isempty(sim)
                sim = 1;
            end
            if nargin<3 || isempty(scale)
                scale = 5;
            end
            if nargin < 4
                options = struct();
            end
            if isfield(options, 'plot_gimbal')
                plot_gimbal = options.plot_gimbal;
            else
                plot_gimbal = false;
            end
            if plot_gimbal
                obj.nRows = 1;
                obj.nCols = 3;
            else
                obj.nRows = 1;
                obj.nCols = 2;
            end

            % plotting init stuff
            obj.sim=sim;
            
            obj.plot_3d=subplot(obj.nRows, obj.nCols, 1);
           % obj.plot_3d=fig;
            hold(obj.plot_3d,'on');
            grid on
            obj.q1=quiver3(obj.plot_3d,0,0,0,1,0,0,'r'); % x axis of quad
            
            obj.q2=quiver3(obj.plot_3d,0,0,0,0,1,0,'g'); % y axis of quad
            obj.q3=quiver3(obj.plot_3d,0,0,0,0,0,1,'b'); % z axis of quad
            
            obj.h_x=quiver3(obj.plot_3d,0,0,0,1,0,0,'m'); % tangent vector of reference
            xlabel(obj.plot_3d,'X-Axis');
            ylabel(obj.plot_3d,'Y-Axis');
            zlabel(obj.plot_3d,'Z-Axis');
            plot_camera=1;
            if plot_camera
                % define the trajectory using a figure

                depth=3*0.2;
                width=1*0.2;
                height=1;
                r1=[depth;width;width];
                r2=[depth;width;-width];
                r3=[depth;-width;-width];
                r4=[depth;-width;width];
                obj.camera.r1=r1;
                obj.camera.r2=r2;
                obj.camera.r3=r3;
                obj.camera.r4=r4;
                
%                 obj.camera.hr1=quiver3(0,0,0,r1(1),r1(2),r1(3),'b') % x axis of quad
%                 obj.camera.hr2=quiver3(0,0,0,r2(1),r2(2),r2(3),'b') % y axis of quad
%                 obj.camera.hr3=quiver3(0,0,0,r3(1),r3(2),r3(3),'b') % z axis of quad
%                 obj.camera.hr4=quiver3(0,0,0,r4(1),r4(2),r4(3),'b') % z axis of quad
                obj.camera.hr1=line([0,r1(1)],[0,r1(2)],[0,r1(3)]); % x axis of quad
                obj.camera.hr2=line([0,r2(1)],[0,r2(2)],[0,r2(3)]); % x axis of quad
                obj.camera.hr3=line([0,r3(1)],[0,r3(2)],[0,r3(3)]); % x axis of quad
                obj.camera.hr4=line([0,r4(1)],[0,r4(2)],[0,r4(3)]); % x axis of quad

                obj.camera.h1=line([r1(1),r2(1)],[r1(2),r2(2)],[r1(3),r2(3)]); % x axis of quad
                obj.camera.h2=line([r2(1),r3(1)],[r2(2),r3(2)],[r2(3),r3(3)]); % y axis of quad
                obj.camera.h3=line([r3(1),r4(1)],[r3(2),r4(2)],[r3(3),r4(3)]); % z axis of quad
                obj.camera.h4=line([r4(1),r1(1)],[r4(2),r1(2)],[r4(3),r1(3)]); % z axis of quad
            end
            
            % plot ref. point
            obj.target_setp=plot3(obj.plot_3d,0,0,0,'+r'); % pos. setpoint
            obj.quad_setp=plot3(obj.plot_3d,0,0,0,'+r'); % pos. setpoint
            
            obj.q_yaw=quiver3(obj.plot_3d,0,0,0,1,0,0,'m');
            
            grid on;
            grid minor;
            axis(obj.plot_3d,scale*[-1 1 -1 1 -1 1]);
            axis manual;
            
 
            
            % define the trajectory using a figure
            obj.plot_2d=subplot(obj.nRows, obj.nCols, 2);
            axis(scale*[-1 1 -1 1]);
            grid on;
            grid minor;

            if plot_gimbal
                % define the trajectory using a figure
                obj.plot_gimbal = subplot(obj.nRows, obj.nCols, 3);

                grid on
                obj.q1_gimbal=quiver3(0,0,0,1,0,0,'r'); % x axis of quad
                hold on
                obj.q2_gimbal=quiver3(0,0,0,0,1,0,'g'); % y axis of quad
                obj.q3_gimbal=quiver3(0,0,0,0,0,1,'b'); % z axis of quad

                xlabel(obj.plot_gimbal,'X-Axis');
                ylabel(obj.plot_gimbal,'Y-Axis');
                zlabel(obj.plot_gimbal,'Z-Axis');

                grid on
                grid minor
                axis(scale*[-1 1 -1 1 -1 1]);
                axis manual
            end
            

            
             
        end
        
        function updateVisualization(obj,R,state,refpoint_quad,refpoint_target,tangent,setp_gimbal)
%             if nargin<1
%                 R=eye(3);
%             end
%             if nargin<2
%                 state=zeros(3,1);
%             end
%             if nargin<3
%                 refpoint=zeros(3,1);
%             end
%             if nargin<4
%                 tangent=[0;0;0];
%             end
            set(obj.q1,'XData',state(1),'YData',state(2),'ZData',state(3))
            set(obj.q2,'XData',state(1),'YData',state(2),'ZData',state(3))
            set(obj.q3,'XData',state(1),'YData',state(2),'ZData',state(3))
            set(obj.q1,'UData',R(1,1),'VData',R(2,1),'WData',R(3,1))
            set(obj.q2,'UData',R(1,2),'VData',R(2,2),'WData',R(3,2))
            set(obj.q3,'UData',R(1,3),'VData',R(2,3),'WData',R(3,3))
            set(obj.h_x,'XData',refpoint_quad(1),'YData',refpoint_quad(2),'ZData',refpoint_quad(3));
            set(obj.h_x,'UData',tangent(1),'VData',tangent(2),'WData',tangent(3));
            
            set(obj.quad_setp,'XData',refpoint_quad(1),'YData',refpoint_quad(2),'ZData',refpoint_quad(3));
            if ~isempty(refpoint_target)
                set(obj.target_setp,'XData',refpoint_target(1),'YData',refpoint_target(2),'ZData',refpoint_target(3));
            end
            
            set(obj.q_yaw,'XData',refpoint_quad(1),'YData',refpoint_quad(2),'ZData',refpoint_quad(3));
            
            if ~isempty(setp_gimbal)
                ux=sin(setp_gimbal.theta)*cos(setp_gimbal.phi);
                uy=sin(setp_gimbal.theta)*sin(setp_gimbal.phi);
                uz=cos(setp_gimbal.theta);
                set(obj.q_yaw,'UData',ux,'VData',uy,'WData',uz);
            end
        end

        function updateCamera(obj,R,pos)
   
                
                r1=pos+R*obj.camera.r1;
                r2=pos+R*obj.camera.r2;
                r3=pos+R*obj.camera.r3;
                r4=pos+R*obj.camera.r4;

%            set(obj.camera.r1,'UData',r1(1),'VData',r1(2),'WData',r1(3)) 
%            set(obj.camera.r1,'UData',r2(1),'VData',r2(2),'WData',r2(3)) 
%            set(obj.camera.r1,'UData',r3(1),'VData',r3(2),'WData',r3(3)) 
%            set(obj.camera.r1,'UData',r4(1),'VData',r4(2),'WData',r4(3)) 
           
           set(obj.camera.hr1,'XData',[pos(1),r1(1)],'YData',[pos(2),r1(2)],'ZData',[pos(3),r1(3)]) 
           set(obj.camera.hr2,'XData',[pos(1),r2(1)],'YData',[pos(2),r2(2)],'ZData',[pos(3),r2(3)]) 
           set(obj.camera.hr3,'XData',[pos(1),r3(1)],'YData',[pos(2),r3(2)],'ZData',[pos(3),r3(3)]) 
           set(obj.camera.hr4,'XData',[pos(1),r4(1)],'YData',[pos(2),r4(2)],'ZData',[pos(3),r4(3)]) 
           
             
           set(obj.camera.h1,'XData',[r1(1),r2(1)],'YData',[r1(2),r2(2)],'ZData',[r1(3),r2(3)]) 
           set(obj.camera.h2,'XData',[r2(1),r3(1)],'YData',[r2(2),r3(2)],'ZData',[r2(3),r3(3)]) 
           set(obj.camera.h3,'XData',[r3(1),r4(1)],'YData',[r3(2),r4(2)],'ZData',[r3(3),r4(3)]) 
           set(obj.camera.h4,'XData',[r4(1),r1(1)],'YData',[r4(2),r1(2)],'ZData',[r4(3),r1(3)]) 
               
        end
        function updateGimbal(obj, R)
            if ~isempty(obj.plot_gimbal)
                set(obj.q1_gimbal,'UData',R(1,1),'VData',R(2,1),'WData',R(3,1))
                set(obj.q2_gimbal,'UData',R(1,2),'VData',R(2,2),'WData',R(3,2))
                set(obj.q3_gimbal,'UData',R(1,3),'VData',R(2,3),'WData',R(3,3))
            end
        end
        
        function plotPolyAndInter(obj,traj_inter)
            
            hold on
            obj.traj_inter_plot2d= plot(obj.plot_2d,traj_inter(1,:),traj_inter(2,:));
            hold off
            uistack(obj.traj_inter_plot2d, 'bottom')
           
        
            hold on
            obj.traj_inter_plot3d= plot3(obj.plot_3d,traj_inter(1,:),traj_inter(2,:),traj_inter(3,:));
       
            hold off 
        end
    end
    
end

