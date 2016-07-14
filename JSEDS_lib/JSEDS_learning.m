function [Priors, Mu, Sigma, As] = JSEDS_learning(Data, k, robotplant, seds_options, jseds_options)
%JSEDS_LEARNING Train a JSEDS model, and its associated A augmentation
%matrices
% Inputs -----------------------------------------------------------------
%
%   o Data:    A (2dimq + dimx) x N_Total matrix containing all demonstration data points.
%              The rows are assembled as follows:
%              1:dimq - joint angle values throughout the trajectories
%              dimq+1:2dimq - first time derivative of joint angles
%              2dimq+1:dimq - joint angle values throughout the trajectories
%              dimq+1:2dimq - first time derivative of joint angles
%              1:dimq - joint angle values throughout the trajectories
%              dimq+1:2dimq - first time derivative of joint angles
%              are their first time derivatives. Each column of Data stands
%              for a datapoint. All demonstrations are put next to each other 
%              along the second dimension. For example, if we have 3 demos
%              D1, D2, and D3, then the matrix Data is:
%                               Data = [[D1] [D2] [D3]]
%
%   o K:       1 x K array representing the prior probabilities of the K GMM 
%
%   o seds_options: A structure to set the optional parameters of the solver, as defined in "SEDS".
%              Providing 'options' for initialization forces the function
%              to compute a nice initial guess for the main optimization.
%              It is highly recommended to call the function with this
%              variable. Please type
%                               doc SEDS_Solver
%
%              in MATLAB command window to get a list of available
%              parameters that can be passed to the solver via 'options'.

%   o jseds_options: A structure to set the optional parameters of the
%               joint-specific optimization (CREATE DOCUMENTATION
%               SOMEWHERE).
%
%
%
%   ------------------- OUTPUTS ------------------
%   Piors, Mu, Sigma are the parameters of the computed GMM
%   As are the respective augmentation matrices of each mode in the GMM
    dimq = robotplant.robot.n;
    dimx = robotplant.dimx;
    n = size(Data, 2);
    
    Xt = Data(end - dimx + 1:end, :);
    Qd = Data(dimq + 1: 2*dimq, :);
    % now remove the parts of the data matrix that should not be learned on
    % but which we have included to extract things like xT, Qd
    if ~jseds_options.include_target_pos_in_GMM
        Data = Data(1:end - dimx, :); % cut off the end of our matrix
    end
    if ~jseds_options.include_joint_vel_in_GMM
        Data = Data([1:dimq, 2*dimq + 1:end]); % cut out the second qdim elements
    end
    [Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,K); %finding an initial guess for GMM's parameter
    [Priors, Mu, Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,seds_options); %running SEDS optimization solver
    Priors = Priors(1:dimq);
    Mu = Mu(1:dimq, :);
    Sigma = Sigma(1:dimq, 1:dimq, :);
    
    % Now that we have the GMM parameters, we need to find As to best fit
    % the data
    options = sdpsettings('verbose', 1);
    Constraints = [];
    % Ensure all A matrices are PSD
    for i = 1:k
        A_vars{i} = sdpvar(dimq, dimq, 'symmetric');
        Constraints = [Constraints A_vars{i} > 0];
    end
    % Find the GMM mixture distribution h for each point
    Qd_basis = zeros(size(Qd));
    for i = 1:n
        q = Data(1:dimq, i); xt = Xt(:, i);
        Qd_basis(:, i) = robotplant.qd_basis(q, xt);
        for j = 1:k
            h_raw(j, i) = Priors(j).*gaussPDF(q, Mu(:, j), Sigma(:, :, j));
        end
        htotal = sum(h_raw(:,i));
        h(:, i) = h_raw(:,i)/htotal;
    end
    % Then calculate the difference between estimated joint vel
    % h*A*qd_basis and true qd
    Qd_error = -Qd(:, :);
    for j = 1:k
        Qd_error = Qd_error + h(j, :).*(A_vars{j}*Qd_basis);
    end
    % define the norm of this error as the objective, and solve for A
    Objective = sqrt(sum(Qd_error.^2,1));
    sol = optimize(Constraints, Objective, options);
    if sol.problem ~= 0
        yalmiperror(sol.problem);
    end
    As = cell2mat(value(A_vars));
end

