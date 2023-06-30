clc
close all
clear

% Choose slash according to your operating system automatically
if ispc
    path = 'functions\';
else
    path = 'functions/';
end

addpath(path);

%% Load parameters
parameters;

%% Initialization
[chute, ground_check, true_centroid_store] = initialization_chutes();
check_correlation(chute)
t = 1;
pause
%% Localization and consensus on the positions
chute = localization_chutes_KF(chute); 

% Distribute the positions
chute = distribute_informations2(chute);

%% Compute the global centroid
chute = wls_centroid(chute);

%% Voronoi
chute = voronoi_chutes(chute);

%% Dynamic
[chute, ground_check, true_centroid_store] = dynamic_chutes_m2(chute, t, ground_check, true_centroid_store, t);

%% Localization and consensus on the positions
chute = localization_chutes_KF(chute);

check_correlation(chute);
