function fig = initialize_robot_figure(robot,varargin)

if nargin>1
    % figure exists and we should activate axes to plot in
    axes(varargin{1})
else
    % create plot in new figure window
    fig = figure();
end
robot.plot(zeros(1, robot.n));
view([0 90])
hold on
% plot workspace limitation
rad = sum(robot.a);
if(1)
%if (isempty(robot.links(1).qlim))   % assuming the first joint is representative of whether the whole bot has joint limits
    rad = sum(robot.a);
    x_data = rad*sin(0:0.01:2*pi);
    y_data = rad*cos(0:0.01:2*pi);
    plot(x_data, y_data, 'k--','linewidth',4);
    axis([-0.5 1.5 -0.5 1.5])
else
    for i = 1:length(robot.links)
        qmin(i) = robot.links(i).qlim(1);
        qmax(i) = robot.links(i).qlim(2);
    end
    
end

end
