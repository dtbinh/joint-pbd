% % Define variables
% x = sdpvar(2,1);
% % Define constraints and objective
% Constraints = [sum(x) <= 1, x(1)==0, x(2) >= 0.5];
% Objective = x'*x+norm(x);
% % Set some options for YALMIP and solver
% options = sdpsettings('verbose',1);
% % Solve the problem
% sol = optimize(Constraints,Objective,options);
% % Analyze error flags
% if sol.problem == 0
%  % Extract and display value
%  solution = value(x)
% else
%  display('Hmm, something went wrong!');
%  sol.info
%  yalmiperror(sol.problem)
% end


% single-matrix optimization
joint_dim = 3;
task_dim = 3;
% define the variables
A = sdpvar(joint_dim,'diag');

% Define the opt. parameters
q_max = ones(1,joint_dim);
q_min = ones(1,joint_dim);
q_data = ones(joint_dim, n);
x_data = ones(task_dim, n);
qd_data = ones(joint_dim, n);
q_basis = ones(joint_dim, n);

Objective = norm(A*q_data - qd_data);
Constraints = [A<0];
for i = 1:joint_dim
    Constraints = [Constraints A(i, i)*q_max(i) < 0];
    Constraints = [Constraints A(i, i)*q_min(i) > 0];
    
    

