function planar_robot_limited_sim()
    l = [.5;.4;.3];
    task_dim = 3;
    joint_dim = 3;
    A = 3*eye(joint_dim);
    
    qmax = [.6*pi, .6*pi, .6*pi];%1/2.*pi*ones(1, joint_dim);
    qmin = -1*[.6*pi, .6*pi, .6*pi];%[-1/2.*pi*ones(1, joint_dim);
    
    robot = create_simple_robot(joint_dim, l, qmin, qmax);
%     robot_aug = create_simple_robot(joint_dim, l);
    fig = initialize_robot_figure(robot);
    title('Joint-Limited Lyapunov controller');
%     fig_aug = initialize_robot_figure(robot_aug);
%     title('Augmented Lyapunov controller');
   
    disp('Pick a point for the robot to start at.');
    xi = get_end_position(fig);
    Ti = eye(4); Ti(1:3,4) = xi;
    qi = robot.ikine(Ti, ones(1, joint_dim), [1 1 0 0 0 0]);
    figure(fig);
    robot.animate(qi);
%     figure(fig_aug);
%     robot_aug.animate(qi);
    %%
    while(1)
        disp('Pick a point for the robot to end at.');
        xf = get_end_position(fig);
        plot(xf(1), xf(2), 'bo', 'markersize', 20);
%         figure(fig_aug);
%         plot(xf(1), xf(2), 'rx', 'markersize', 20);
        disp('Simulating policy motion');
        dt= .05;
        qf = simulation(qi);
        qi = qf;
    end
   


    function qf = simulation(qi)
        q = qi;
        while(1)
%             % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:3,4);
            qd = get_vstyle_velocity(q, xf);
            if (~isreal(qd) || ~(all(q >= qmin) && all(q <= qmax)))
                disp('Infeasible trajectory, pick a different starting point')
                qf = qi;
                break
            end
            q_next = q+qd*dt;
            for j = 1:joint_dim
                if q_next(j) - qmin(j) < 0
                    q_next(j) = qmin(j);% - (q_next(j) - qmin(j));
                elseif q_next(j) - qmax(j) > 0
                    q_next(j) = qmax(j); %- (q_next(j) - qmax(j));
                end
            end
            q = q_next;
            if (norm(x - xf)<0.05 )%&& norm(x_aug - xf)<.01)
                qf= q;
                disp('Success!');
                break
            end
            robot.delay = dt;
            
            figure(fig);
            robot.animate(q);
            plot(x(1), x(2), 'm.','markersize',10);
        end
    end
    
    function qdot = get_velocity(q, x_t)
        qdot =  -1*lyap_control_vector(q, x_t);
    end

    function qdot = lyap_control_vector(q, x_t)
        if size(q,2) ==3
            qdot = [...
                (l(1)*x_t(1)*sin(q(1)) - l(1)*x_t(2)*cos(q(1)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) + l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
                (l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) - l(1)*l(2)*sin(q(2)) - l(1)*l(3)*sin(q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
                (-l(3)*(l(1)*sin(q(2) + q(3)) + l(2)*sin(q(3)) + x_t(2)*cos(q(1) + q(2) + q(3)) - x_t(1)*sin(q(1) + q(2) + q(3))))];
            %qdot = [0, l(1)*l(2)*sin(q(2)) + l(1)*l(2)*sin(q(2) + q(3)), l(2)*l(3)*sin(q(3)) + l(1)*l(3)*sin(q(2) + q(3))];
        end
    end

    function basis = get_vstyle_basis(q, x_t)
        cosv = 1 - 2*(q - qmin)./(qmax-qmin);
        dqdv_squared = ((qmax-qmin)/2.).^2.*(1-cosv.^2);
        basis = -(dqdv_squared.*lyap_control_vector(q, x_t));
    end

    function dqdt = get_vstyle_velocity(q, x_t)
        dqdt = transpose(A*transpose(get_vstyle_basis(q,x_t)));
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

