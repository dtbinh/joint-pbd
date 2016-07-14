classdef RobotPlant
    %ROBOTPLANT A class that encodes the robot's kinematic model, and also
    %a method for generating its task description
    %   Detailed explanation goes here
    
    properties
        robot % a robotics-toolbox type robot to generate kinematics
        forward_kinematics % takes as input q, and gives corresponding x as output
        jacobian % takes as input q, qd, and gives corresponding xd as output
        dimx % the dimension of x
    end
    
    methods
        function obj = RobotPlant(robot, task_type, custom_forward_kinematics, custom_jacobian, dimx)
            % task_type can be one of the following:
            % 'end_trans' = end xyz, 'end_full' = end xyz + rpy angles, or 'custom'
            % if 'custom' must specify the forward kinematics and custom
            % jacobian using the 3rd and 4th arguments
            % dimx is the dimensionality of the task space
            % custom_qd_basis takes as input the object, q, and dt,
            % and outputs the basis for our velocity policy
            obj.robot = robot;
                
            if nargin<5
                if strcmp(task_type, 'end_trans')
                    obj.forward_kinematics = @obj.end_trans_forward_kinematics;
                    obj.jacobian = @obj.end_trans_jacobian;
                    obj.dimx = 3;
                elseif strcmp(task_type, 'end_full')
                    obj.forward_kinematics = @obj.end_full_forward_kinematics;
                    obj.jacobian = @obj.end_full_jacobian;
                    obj.dimx = 6;
                else
                    error('Custom kinematics/jacobian functions were not specified');
                end
            else
                obj.forward_kinematics = custom_forward_kinematics;
                obj.jacobian = custom_jacobian;
                obj.dimx = dimx;
            end
        end
        
        function basis = qd_basis(obj, q, xt)
            basis = -1*transpose(obj.jacobian(q))*(obj.forward_kinematics(q) - xt);
        end
        
        function x = end_trans_forward_kinematics(obj,q)
            T = obj.robot.fkine(q);
            x = T(1:3, 4);
        end
        
        function j = end_trans_jacobian(obj, q)
            j = obj.robot.jacob0(q, 'trans');
        end
        
        function x = end_full_forward_kinematics(obj, q)
            T = obj.robot.fkine(q);
            x(1:3) = T(1:3, 4);
            x(4:6) = dcm2angle(T(1:3, 1:3));
        end
        
        function j = end_full_jacobian(obj, q)
            j = obj.robot.jacob0(q, 'rpy');
        end
    end
    
end

