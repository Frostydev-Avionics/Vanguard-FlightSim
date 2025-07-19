function setupEnv()
%% setupEnv - Configures simulation environment vars

simPaths = {

    %% Models
    'Models'
    fullfile('Models', 'Aero')
    fullfile('Models', 'Motor')
    fullfile('Models', 'Controls')
    fullfile('Models', 'Gravity')
    genpath(fullfile('Models', 'RefactoredNav'))
    % genpath(fullfile('Models', 'Nav'))
    
    % Atmospheric Model
    fullfile('Models', 'Atmosphere')

    % IMU Model
    fullfile('Models', 'IMU')

    % Kinematics
    fullfile('Models', 'Kinematics')

    %% Guidance
    fullfile('Guidance')

    %% Controls
    fullfile('Controls')

    %% Utilities
    fullfile('Utils');

    %% Plotting
    fullfile('Plotting');
    
};

simPaths = strjoin(simPaths, ';');
addpath(simPaths);

disp('[Setup] Configured Environment!');

end