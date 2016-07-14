function [q, qd] = Simulation(q0, xt, motion_generator, dt)
%SIMULATION Summary of this function goes here
%   Detailed explanation goes here

    if nargin < 4
        dt = .05;
    end
    goal_threshold = .05;
    
    robot = motion_generator.plant.robot;
    fig = initialize_robot_figure(robot);
    q = q0;
    robot.animate(q');
    plot(xt(1), xt(2), 'ro', 'markersize', 20);
    x = motion_generator.plant.forward_kinematics(q0);
    while(norm(x - xt) > goal_threshold)
        qd = motion_generator.get_next_motion(q, xt);
        
        q_candidate = q + qd*dt;
        % bounds-checking
        % (current implementing bound-hugging)
        for j = 1:robot.n
            qlim = robot.qlim(j, :); qmin = qlim(1); qmax = qlim(2);
            if q_candidate(j) - qmin < 0
                q_candidate(j) = qmin;
            elseif q_candidate(j) - qmax > 0
                q_candidate(j) = qmax;
            end
        end
        q = q_candidate;
        x = motion_generator.plant.forward_kinematics(q);
        robot.animate(q');
        plot(x(1), x(2), 'b.', 'markersize', 10);
    end
    disp('Reached goal.');
end

