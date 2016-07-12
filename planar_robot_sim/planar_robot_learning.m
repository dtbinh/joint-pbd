function planar_robot_learning()

ret_flag = 0;
sdpsettings('solver','sedumi');
    l = [.5;.4;.3];
    task_dim = 3;
    joint_dim = 3;
    
    limitless = false;
    
    if limitless
        qmax = 2*pi*ones(1, joint_dim);
        qmin = -2*pi*ones(1, joint_dim);
        get_basis = @get_standard_basis;
    else
        qmax = [.8*pi, .8*pi, .8*pi];%1/2.*pi*ones(1, joint_dim);
        qmin = -1*[.8*pi, .8*pi, .8*pi];%[-1/2.*pi*ones(1, joint_dim);
        get_basis = @get_vstyle_basis;
    end
    
    close all
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% open parameters %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


nb_demo = 4;
dt= .2;
    
robot = create_simple_robot(joint_dim, l, qmin, qmax);
fig = initialize_robot_figure(robot);

%%
disp('You will now provide a few demonstrations of the motion in task space.')
disp('The robot will find accompanying joint configurations using inverse kinematics, and use these as its training data.')
% get a demonstration

nb_knots = 10;
nb_clean_data_per_demo = 50;
Data = [];
qd_lyap_basis = [];
target = zeros(1, task_dim);
hpp = [];
for i=1:nb_demo
    [task_data, hp] = get_demonstration(fig,0); task_data = transpose(task_data);
    try
        q_data = task_to_joint_trajectory(task_data);
    catch
        disp('Suggested trajectory was infeasible')
        continue
    end
    nb_data = size(q_data,1);
    skip = floor(nb_data/nb_knots);
    q_knots = q_data(1:skip:end, :);
    ppx = spline(transpose(q_knots(:, joint_dim + 1)),transpose(q_knots(:, 1:joint_dim)));
    % % and get first derivative
    ppxd = differentiate_spline(ppx);
    tt = linspace(q_knots(1,joint_dim + 1), q_knots(end, joint_dim + 1), nb_clean_data_per_demo);
    q_pos = transpose(ppval(ppx,tt));
    x_d = task_data(end, 1:task_dim);
    target = target + x_d;
    q_pos = [q_pos repmat(x_d, nb_clean_data_per_demo, 1)];
    %pos = pos - repmat(pos(:,end), 1, nb_clean_data_per_demo);
    q_vel = transpose(ppval(ppxd,tt));
    q_vel(end,:) = zeros(1,joint_dim);
    Data = [Data; [q_pos,q_vel]];
    hpp = [hpp, hp];
    qd_lyap_basis_tmp = zeros(nb_clean_data_per_demo, joint_dim);
    for j = 1:nb_clean_data_per_demo
        qd_lyap_basis_tmp(j, :) =  get_basis(q_pos(j, 1:joint_dim), x_d);
    end
    [q_vel,qd_lyap_basis_tmp]
    qd_lyap_basis = [qd_lyap_basis;qd_lyap_basis_tmp];
end
% Data vector is: joint values, end position values, joint velocities
% qd_lyap_basis = zeros(size(Data, 1), joint_dim);
% for i = 1:size(Data, 1)
%     qd_lyap_basis(i, :) = vstyle_lyap_vector(Data(i, 1:joint_dim), Data(i,joint_dim + 1:joint_dim + task_dim));
% end
delete(hpp);
target = target/nb_demo;
plot(target(1), target(2), 'bo','markersize',20);
%% Define the optimization
A = sdpvar(joint_dim);%,joint_dim,'diag');

qd_max = ones(joint_dim,1);
Objective = norm(A*transpose(qd_lyap_basis) - transpose(Data(:, joint_dim + task_dim + 1: 2*joint_dim + task_dim)));
Constraints = A>0;

optimize(Constraints, Objective);
A = value(A);
A = 3*A/norm(A);
p = all(eig(A)>0);
eps = 10^-6;
while p==0
    disp('Matrix was not yet PSD');
    A = A+eye(3)*eps;
    eps = 2*eps;
    p = all(eig(A)>0);
end


%% Loop through results
xf = transpose(target);
    while(1)
        disp('--------------------------');
        disp('Pick a point for the robot to start at.');
        xi = get_end_position(fig);
        Ti = eye(4); Ti(1:3,4) = xi;
        qi = robot.ikine(Ti, ones(1, joint_dim), [1 1 0 0 0 0]);
        plot(xi(1), xi(2), 'rx', 'markersize', 20);
        disp('Simulating...');
        simulation(qi, @get_velocity);
    end


    function qf = simulation(qi, policy)
        q = qi;
        set(gcf,'WindowButtonDownFcn',@set_ret_flag);
        while(1)
%             % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:3,4);
            qd = policy(q, xf, A);
            q_next = inf;
            dt_tmp = dt;
            if (~isreal(qd) || ~(all(q >= qmin) && all(q <= qmax)))
                disp('Infeasible trajectory, pick a different starting point')
                return
            end
            % make sure we don't exceed our boundaries
%             while ~(all(q_next > qmin) && all(q_next < qmax))
%                 q_next = q + dt_tmp*qd;
%                 dt_tmp = dt_tmp/2;
%                 if dt_tmp < 10^-2
%                     disp('Timestep grew too small, likely infeasible')
%                     %return
%                     break
%                 end
%             end
            % if we are near a boundary, jump past the boundary
            q_next = q + dt_tmp*qd;
            for j = 1:joint_dim
                if q_next(j) - qmin(j) < 0
                    q_next(j) = qmin(j);% - (q_next(j) - qmin(j));
                elseif q_next(j) - qmax(j) > 0
                    q_next(j) = qmax(j); %- (q_next(j) - qmax(j));
                end
            end
                
            q = q_next;
            if ret_flag
                ret_flag = 0;
                set(gcf,'WindowButtonDownFcn',[]);
                disp('Trajectory terminated prematurely by user');
                return
            end
            if (norm(x - xf)<0.05 )
                qf= q;
                disp('Success!')
                break
            end
            robot.delay = dt;
            
            figure(fig);
            robot.animate(q);
            plot(x(1), x(2), 'm.','markersize',10);
        end
    end
    function set_ret_flag(h,e)
        ret_flag = 1;
    end

    function q_traj = task_to_joint_trajectory(x_traj, q_init_guess) % may throw error
        q_traj = zeros(size(x_traj, 1), joint_dim);
        if nargin < 2 
            q_init_guess = ones(1, joint_dim);
        end
        mask = [1 1 0 0 0 0];
        Ti = eye(4); Ti(1:3,4) = x_traj(1:3,1);
        q_traj(1, :) = robot.ikine(Ti, q_init_guess, mask); % This mask specifies that we only care about first 2 dim
        for i = 2:size(x_traj,1)
            Ti(1:3,4) = transpose(x_traj(i, 1:3));
            q_traj(i, :) = robot.ikine(Ti, q_traj(i-1, :), mask);
        end
        q_traj = [q_traj x_traj(:,task_dim + 1)];
    end
    
    function basis = get_standard_basis(q, x_t)
        basis = -lyap_control_vector(q, x_t);
    end

    function basis = get_vstyle_basis(q, x_t)
        cosv = 1 - 2*(q - qmin)./(qmax-qmin);
        dqdv_squared = ((qmax-qmin)/2.).^2.*(1-cosv.^2);
        basis = -(dqdv_squared.*lyap_control_vector(q, x_t));
    end

    function dqdt = get_velocity(q, x_t, A)
        dqdt = transpose(A*transpose(get_basis(q,x_t)));
    end

    function output = lyap_control_vector(q, x_t)
        % computes J(q)^T * (H(q) - x_t), which defines the basis of our control vector
        if joint_dim ~= 3
            error('Does not support non-3DOF systems yet!');
        end
        sin_q1 = sin(q(1)); cos_q1 = cos(q(1)); sin_q2 = sin(q(2)); sin_q3 = sin(q(3));
        cos_q1pq2pq3 = cos(q(1) + q(2) + q(3)); sin_q1pq2pq3 = sin(q(1) + q(2) + q(3));
        cos_q1pq2 = cos(q(1) + q(2)); sin_q1pq2 = sin(q(1) + q(2));
        sin_q2pq3 = sin(q(2) + q(3));
        output = [...
        (l(1)*x_t(1)*sin_q1 - l(1)*x_t(2)*cos_q1) - l(3)*x_t(2)*cos_q1pq2pq3 + l(3)*x_t(1)*sin_q1pq2pq3 - l(2)*x_t(2)*cos_q1pq2 + l(2)*x_t(1)*sin_q1pq2,...
        (l(3)*x_t(1)*sin_q1pq2pq3 - l(3)*x_t(2)*cos_q1pq2pq3 - l(1)*l(2)*sin_q2 - l(1)*l(3)*sin_q2pq3 - l(2)*x_t(2)*cos_q1pq2 + l(2)*x_t(1)*sin_q1pq2),...
        (-l(3)*(l(1)*sin_q2pq3 + l(2)*sin_q3 + x_t(2)*cos_q1pq2pq3 - x_t(1)*sin_q1pq2pq3))];
%         output = -1*[...
%                 (l(1)*x_t(1)*sin(q(1)) - l(1)*x_t(2)*cos(q(1)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) + l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
%                 (l(3)*x_t(1)*sin(q(1) + q(2) + q(3)) - l(3)*x_t(2)*cos(q(1) + q(2) + q(3)) - l(1)*l(2)*sin(q(2)) - l(1)*l(3)*sin(q(2) + q(3)) - l(2)*x_t(2)*cos(q(1) + q(2)) + l(2)*x_t(1)*sin(q(1) + q(2))),...
%                 (-l(3)*(l(1)*sin(q(2) + q(3)) + l(2)*sin(q(3)) + x_t(2)*cos(q(1) + q(2) + q(3)) - x_t(1)*sin(q(1) + q(2) + q(3))))];
    end
end