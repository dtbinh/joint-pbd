

qdim = 7;
% Approximate parameters for the KUKA robot
A = [0 0 0 0 0 0 0];
Alpha = pi*[1 -1 -1 1 1 -1 0];
D = [.34 0 .4 0 .4 0 .126];
Qmin = pi/180*[-85, -90, -100, -110, -140, -90, -120];
Qmax = pi/180*[85, 90, 100, 110, 140, 90, 120];
robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
robotplant = RobotPlant(robot, 'end_trans');
dt = .1;

time = dt;
num_gaussians = 6;
tol_cutting = 1;

% A set of options that will be passed to the SEDS solver. Please type 
% 'doc preprocess_demos' in the MATLAB command window to get detailed
% information about other possible options.
seds_options.tol_mat_bias = 10^-6; % A very small positive scalar to avoid
                              % instabilities in Gaussian kernel [default: 10^-15]
                              
seds_options.display = 1;          % An option to control whether the algorithm
                              % displays the output of each iterations [default: true]
                              
seds_options.tol_stopping=10^-10;  % A small positive scalar defining the stoppping
                              % tolerance for the optimization solver [default: 10^-10]

seds_options.max_iter = 500;       % Maximum number of iteration for the solver [default: i_max=1000]

seds_options.objective = 'mse';    % 'likelihood': use likelihood as criterion to
                              % optimize parameters of GMM
                              % 'mse': use mean square error as criterion to
                              % optimize parameters of GMM
                              % 'direction': minimize the angle between the
                              % estimations and demonstrations (the velocity part)
                              % to optimize parameters of GMM                              
                              % [default: 'mse']
                              
jseds_options.include_joint_vel_in_GMM = false;
jseds_options.include_task_pos_in_GMM = false;
jseds_options.include_task_vel_in_GMM = false;
jseds_options.include_target_pos_in_GMM = false;




[xT, Data, index] = preprocess_demos_JSEDS(robotplant, demos, time, tol_cutting, jseds_options);
[Priors, Mu, Sigma, As] = JSEDS_learning(Data,num_gaussians,robotplant,seds_options,jseds_options);
motion_generator = MotionGenerator(robotplant);%, Mu, Sigmas, Priors, As);

Simulation([0;0;0;0;0;0;0], [1;1;1], motion_generator);