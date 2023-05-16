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

%% Initialization
chute = initialization_chutes();

%% Localization and measurement
chute = localization_chutes(chute);

%% Distribute informations
chute = distribute_informations(chute);

%% Voronoi
chute = voronoi_chutes(chute);

%% Dynamic

%% Plot
plot_chutes(chute);