
function theRobot=create_simple_robot(varargin)

qmin = [];
qmax = [];
if(nargin>0)
    num_DOF = varargin{1};
    a = varargin{2};
%     for i = 1:num_DOF
%         a(i) = varargin{i+1};
%     end
    if nargin>2
        qmin = varargin{3};
        qmax = varargin{4};
    end
else
    num_DOF = 2;
    a(1) = .8;
    a(2) = .6;
end
%% creating the links

%   theta d a alpha
for i = 1:num_DOF
    L(i) = Link([ 0 0 a(i) 0], 'standard');
end

link_radius = 0.1 ;
link_length = 0.5;
Ix = 1/12 * L(1).m*(3*link_radius.^2+link_length.^2);
Iy = Ix;
Iz = L(1).m*link_radius.^2/2;

for i = 1:num_DOF
    % mass
    L(i).m = 1;
    % center of gravity
    L(i).r = [-0.5 0 0];
    % inertia matrix
    L(i).I = diag([Ix,Iy,Iz]);
    % gear ratio
    L(i).G = 0;
    % motor inertia
    L(i).Jm = 0;
    % viscous friction
    L(i).B = 0;
    if (~isempty(qmin) && ~isempty(qmax))
        L(i).qlim = [qmin(i), qmax(i)];
    end
end
% L(1).I = diag([Ix,Iy,Iz]);
% % gear ratio
% L(1).G = 0;
% % motor inertia
% L(1).Jm = 0;
% % viscous friction
% L(1).B = 0;
% 
% % mass
% L(2).m = 1;
% % center of gravity
% L(2).r = [-0.5 0 0];
% %intertia matrix (around cog)
% L(2).I =diag([Ix,Iy,Iz]);
% % gear ration
% L(2).G = 0;
% % motor inertia
% L(2).Jm = 0;
% % viscous friction
% L(2).B = 0;
name = sprintf('simple %d-DOF robot', num_DOF);

theRobot = SerialLink(L, 'name', name, ...
    'comment', 'simple multi-link robot');
theRobot.plotopt = {'noshadow','nojaxes', 'nowrist','noname','linkcolor',0.7*[1,1,1], 'ortho','noshading','notiles','jointcolor',0.6*[1,1,1]};

