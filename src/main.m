clc
close all
clear

addpath("functions\")

%% Initialization
chute = initialization_chutes();

%% Distribute informations
chute = distribute_informations(chute);

%% Voronoi
chute = voronoi_chutes(chute);

%% Dynamic

%% Plot
plot_chutes(chute);