% To run this toolbox you need to provide the variable demos composed of all
% demosntration trajectories. The trajecotry can be only position or position and velocity.

clc
clear
close all
% %% User Parameters and Setting
load('DATA/Multi_Demostrations/CShape','Demos') 
% the variable 'demos' composed of 3 demosntrations. Each demonstrations is
% recorded from Tablet-PC at 50Hz. Datas are in millimeters.

%% Initialization
options.dt = 0.01; %The time step of the demonstrations
options.tol_cutting = 0.001; % A threshold on velocity that will be used for trimming demos
 
% Training parameters
options.K =2; %Number of Gaussian
options.d=2; % Dimention  of Demostrations
options.I=1; % It's a reduced factor. Leave it one if you want the dynamical system is trainded accordign to all the data.
options.tol_mat_bias = 10^-18; % A very small positive scalar to avoid  instabilities in Gaussian kernel              
options.display = 1;          % An option to control whether the algorithm displays the output of each iterations [default: true] 
options.tol_stopping=10^(-8);  % A small positive scalar defining the stoppping tolerance for the optimization solver [default: 10^-10]
options.max_iter = 1000; % Maximum number of iteration for the solver [default: i_max=1000]
options.TolCon = 1e-2;   % Tolerance on the constraint violation, a positive scalar. The default is 1e-6.
options.Normilizing='False'; % If the number of the training data points need to be normalized along the path.
options.Velocity='False'; % Velocity and Position of the demostrations are provided. 
                           % If you put it false, make sure that the name of demostrations are demo_P, and demo_V with same lengh.
options.method='SYM'; % APA to construct J-SD
                       % SYM to constract S-SD
options.smoothing='False'; % Recunstracting the demostrations from velocity profile.  
options.Method=1; % Two types for initilizing are provied. First one is based on a one mapping from velocities and Positions to accelerartions. 
                  % Second one is based on a seperate mapping from velocities and Positions to accelerartions 
options.Method_For_APA='NSYM'; % SYM to use the result of S-DS to coustract a J-SD
%% SESODS
addpath([pwd, '/GMR_lib']);    % add GMR dir to path
addpath([pwd, '/SEDS_lib']);    % add SEDS dir to path 
addpath([pwd, '/SESODS_lib']);    % add SEDS dir to path 

if (strcmp(options.Velocity,'False')==1)
    demos_P=[];
    demos_V=[];
end
% Initialization
[Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V, Data_P_A, Data_V_A,Data_A,demos_P,demos_V,demos,Time]=Initialization(Demos,demos_P,demos_V,options);
% Solver and simulation
[Priors_P, Mu_P, Sigma_P,Priors_V, Mu_V, Sigma_V]=SESODS_Solver(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V, Data_P_A, Data_V_A,Data_A,demos_P,demos_V,Time,options);
clearvars -except Priors_P  Mu_P  Sigma_P Priors_V  Mu_V  Sigma_V