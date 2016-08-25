classdef RosTransformListener < handle
    %RosTransformListener Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        tftree
        from_frame_id
        to_frame_id
    end
    
    methods
        function obj = RosTransformListener(from_frame_id, to_frame_id)
            obj.from_frame_id = from_frame_id;
            obj.to_frame_id = to_frame_id;
            obj.tftree= rostf();
            fprintf('Waiting for transform from %s to %s ...', obj.from_frame_id, obj.to_frame_id);
            obj.tftree.waitForTransform(obj.from_frame_id, obj.to_frame_id);
            fprintf(' got transform.\n')
        end
        
        function [pos, rot, age] = getPose(obj)
            % Get the position and orientation (as a JPL quaternion)
            tf = obj.tftree.getTransform(obj.from_frame_id, obj.to_frame_id);
            
            pos = tf.Transform.Translation;
            rot = tf.Transform.Rotation;
            
            pos = [pos.X; pos.Y; pos.Z];
            rot = [rot.X; rot.Y; rot.Z; rot.W];
            
            lastUpdateTime = obj.tftree.LastUpdateTime;
            lastUpdateTime = double(lastUpdateTime.Sec) + double(lastUpdateTime.Nsec) / 1e9;
            currentTime = rostime('now');
            currentTime = double(currentTime.Sec) + double(currentTime.Nsec) / 1e9;
            age = currentTime - lastUpdateTime;
        end
    end
    
end

