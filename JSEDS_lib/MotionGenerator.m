classdef MotionGenerator
    %MOTIONGENERATOR The class which implements the dynamical system
    %specified using our model
    %   Detailed explanation goes here
    
    properties
        plant % a RobotPlant used to compute kinematics properties
        Mu % the means of the GMM
        Sigma % the sigmas of the GMM
        Priors % the scaling of relative Gaussians of the GMM
        As % the augmentation matrices of the components of the GMM
    end
    
    methods
        function obj = MotionGenerator(robotplant, mu, sigma, priors, A)
            obj.plant = robotplant;
            if nargin == 1
                dimq = robotplant.robot.n;
                obj.Mu = zeros(dimq, 1);
                obj.Sigma = eye(dimq, dimq, 1);
                obj.Priors = 1;
                obj.As = eye(4);
            else
                obj.Mu = mu;
                obj.Sigma = sigma;
                obj.Priors = priors;
                obj.As = A;
            end
        end
        
        function qd = get_next_motion(obj, q, xt) % Generates the next joint velocity given the joint position,
            % desired position, a GMM model, and an augmentation matrix for
            % each Gaussian
            k = length(obj.Priors);
            qd_basis = obj.plant.qd_basis(q, xt);
            for i = 1:k
                h_raw(i) = obj.Priors(i).*gaussPDF(q, obj.Mu(:, i), obj.Sigma(:, :, i));
            end
            htotal = sum(h_raw);
            h = h_raw/htotal;
            A_total = zeros(size(obj.As(:,:,1)));
            for i = 1:k
                A_total = A_total + h(i)*obj.As(:, :, i);
            end
            A_total
            qd = A_total*qd_basis;
        end
    end
    
end

