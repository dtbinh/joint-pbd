function planar_robot_sim()
    l = [.5;.4;.3];
    num_DOF = 3;
    robot = create_simple_robot(num_DOF, l);
%     robot_aug = create_simple_robot(num_DOF, l);
    fig = initialize_robot_figure(robot);
    title('Classic Lyapunov controller');
%     fig_aug = initialize_robot_figure(robot_aug);
%     title('Augmented Lyapunov controller');
   
    disp('Pick a point for the robot to start at.');
    xi = get_end_position(fig);
    Ti = eye(4); Ti(1:3,4) = xi;
    qi = robot.ikine(Ti, ones(1, num_DOF), [1 1 0 0 0 0]);
    figure(fig);
    robot.animate(qi);
%     figure(fig_aug);
%     robot_aug.animate(qi);
    %%
    while(1)
        disp('Pick a point for the robot to end at.');
        xf = get_end_position(fig);
        plot(xf(1), xf(2), 'rx', 'markersize', 20);
%         figure(fig_aug);
%         plot(xf(1), xf(2), 'rx', 'markersize', 20);
        disp('Simulating policy motion');
        dt= .2;
        qf = simulation(qi);
        qi = qf;
    end


    function qf = simulation(qi)
        q = qi;
%        q_aug = qi;
        %h = plot(x_ref(1),x_ref(2),'go');
        while(1)
%             % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:3,4);
            qd = get_velocity(q, xf);
            q = q+qd*dt;
            
%             x_aug = robot.fkine(q_aug);
%             x_aug = x_aug(1:3,4);
%             qd_aug = get_velocity_aug(q_aug, xf);
%             q_aug = q_aug+qd_aug*dt;
            
            if (norm(x - xf)<0.03 )%&& norm(x_aug - xf)<.01)
                qf= q;
                break
            end
            robot.delay = dt;
            
            figure(fig);
            robot.animate(q);
            plot(x(1), x(2), 'm.','markersize',10);
%             figure(fig_aug);
%             robot_aug.animate(q_aug);
%            plot(x_aug(1), x_aug(2), 'b.','markersize',10);
%             set(h,'XData',x(1));
%             set(h,'YData',x(2));
            %ht = [ht, plot(x(1), x(2), 'm.','markersize',10)];
        end
    end
    
    function qdot = get_velocity(q, x_t)
        if size(q, 2) == 2
            qdot = [0; l(1)*l(2)*sin(q(2))];
        elseif size(q,2) ==3
            qdot = -3*[...
                (l(1)*x_t(1)*sin(q(1)) - l(1)*x_t(2)*cos(q(1)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) + l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
                (l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) - l(1)*l(2)*sin(q(2)) - l(1)*l(3)*sin(q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
                (-l(3)*(l(1)*sin(q(2) + q(3)) + l(2)*sin(q(3)) + x_t(2)*cos(q(1) + q(2) + q(3)) - x_t(1)*sin(q(1) + q(2) + q(3))))];
            %qdot = [0, l(1)*l(2)*sin(q(2)) + l(1)*l(2)*sin(q(2) + q(3)), l(2)*l(3)*sin(q(3)) + l(1)*l(3)*sin(q(2) + q(3))];
        end
     end
%     function qdot = get_velocity_aug(q, x_t)
%         if size(q, 2) == 2
%             qdot = [0; l(1)*l(2)*sin(q(2))];
%         elseif size(q,2) ==3
%             qdot = -1*[...
%                 2*(l(1)*x_t(1)*sin(q(1)) - l(1)*x_t(2)*cos(q(1)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) + l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
%                 0.2*(l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) - l(1)*l(2)*sin(q(2)) - l(1)*l(3)*sin(q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
%                 3*(-l(3)*(l(1)*sin(q(2) + q(3)) + l(2)*sin(q(3)) + x_t(2)*cos(q(1) + q(2) + q(3)) - x_t(1)*sin(q(1) + q(2) + q(3))))];
%             %qdot = [0, l(1)*l(2)*sin(q(2)) + l(1)*l(2)*sin(q(2) + q(3)), l(2)*l(3)*sin(q(3)) + l(1)*l(3)*sin(q(2) + q(3))];
%         end
%     end
end