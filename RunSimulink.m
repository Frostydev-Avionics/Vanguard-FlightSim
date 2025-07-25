% function SimOut = RunSimulink()
%% WPI High Power Rocket MQP - Flight Simulator
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 12.15.2024

clear variables; close all; clc;

%% Configure constants and model data
const = setupConstants();
% kins = HPMR_MissileKinematics();
% kins = HPMR_ModelRocketKinematics();
kins = HPRC_RocketKinematics();

% Kinematics 
inds = getMissileInds(); % Control State Indices
kfInds = getKfInds();
kfErrInds = getErrorKfInds();
kfConsts = getKfConsts();

% Aerodynamics Model
% AeroModel = initMissileAeroModel();
% AeroModel = initRocketAeroModel();
AeroModel = init_IREC2025_CFDModel();

% IMU Model
ImuModel = getASM330Params();

% Motor Model
MotorModel = initMotorModel();

%% Simulator Config **FOR SIMULINK USE LATER**
% Time Configuration
time.dt = 0.001; % [s] Time Step
time.sensorDt = 0.001;
time.navDt = 0.01;
time.t0 = -10; % [s] Initial Time
% time.tf = 60*3; % [s] Final Time
time.tf = 100;

simCfg.time = time;

%% Launch Site Initialization
% [launchLat, launchLon, launchAlt] = selectLaunchLocation();
launchLat =  42.27405; % [deg] Latitude - Football Field
launchLon = -71.81174; % [deg] Longitude - Football Field
launchAlt = 10; % [m] Altitude MSL - Football Field

% launchLat =  31.942558857776472; % [deg] Latitude - TX
% launchLon = -102.20475753497975; % [deg] Longitude - TX
% launchAlt = 875; % [m] Altitude MSL - TX

launchLLA = [launchLat, launchLon, launchAlt];
% currLLA = launchLLA;

launch_ECEF_m = lla2ecef(launchLLA);

% Attitude Initialization
yaw_0 = deg2rad(0);
roll_0 = deg2rad(0);
pitch_0 = deg2rad(86);
 
eul_0 = [roll_0; pitch_0; yaw_0];

q_0 = hpmr_eul2quat(roll_0, pitch_0, yaw_0);

% Angular Rate Initialization
w_ib_x = 1e-10; % [rad/s]
w_ib_y = 1e-10; % [rad/s]
w_ib_z = 1e-10; % [rad/s]

% Velocity Initialization
R_ET = [
    -sind(launchLat)*cosd(launchLon), -sind(launchLon), -cosd(launchLat)*cosd(launchLon);
    -sind(launchLat)*sind(launchLon),  cosd(launchLon), -cosd(launchLat)*sind(launchLon);
     cosd(launchLat),            0,         -sind(launchLat)
];

R_TB = hpmr_quat2rotm(q_0);
R_EB = R_ET * R_TB;

V_0_B = [1e-5; 1e-5; 1e-5];

V_0_E = R_EB * V_0_B;

% Initial Mass
m_0 = kins.m_0 + MotorModel.emptyWt + MotorModel.propWt;

%% State Initialization
x_0 = [
    q_0;
    launch_ECEF_m';
    V_0_E(1);
    V_0_E(2);
    V_0_E(3);
    w_ib_x;
    w_ib_y;
    w_ib_z;
    m_0;
];

%% Initialize Navigator
kfParams = initNavParams(kins);
assignin('base', 'kfParams', kfParams);

N = length(kfParams.x);
N_err = kfErrInds.maxStateIndex;

elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = [N 1];

elems(2) = Simulink.BusElement;
elems(2).Name = 'x_pred';
elems(2).Dimensions = [N 1];

elems(3) = Simulink.BusElement;
elems(3).Name = 'Q_k';
elems(3).Dimensions = [N_err N_err];

elems(4) = Simulink.BusElement;
elems(4).Name = 'P';
elems(4).Dimensions = [N_err N_err];

elems(5) = Simulink.BusElement;
elems(5).Name = 'P_min';
elems(5).Dimensions = [N_err N_err];

elems(6) = Simulink.BusElement;
elems(6).Name = 'f_pred';
elems(6).Dimensions = [N 1];

kfParamsBus = Simulink.Bus;
kfParamsBus.Elements = elems;

assignin('base', 'kfParamsBus', kfParamsBus);

%% Load Simulink Model
modelName = 'FlightSimulation';
saveRate = 1000; % [Hz]
saveDir = fullfile(pwd, 'SIM_OUT');

% Open Simulation
open_system(modelName);

% Start Timer
tic

SimOut = sim(modelName, 'StopTime', num2str(simCfg.time.tf), 'SaveOutput', 'on');

runTime = toc;

fprintf("Run Time: %.1f sec\n", runTime);

% Assign Parameters
assignin('base', 'const', const); 
assignin('base', 'kins', kins);
assignin('base', 'inds', inds);
assignin('base', 'AeroModel', AeroModel);
assignin('base', 'ImuModel', ImuModel);
assignin('base', 'MotorModel', MotorModel);
assignin('base', 'simCfg', simCfg);
assignin('base', 'launchLLA', launchLLA);
assignin('base', 'x_0', x_0);

% end