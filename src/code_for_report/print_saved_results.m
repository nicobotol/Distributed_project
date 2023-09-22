clc;
close all;
clear;

user_par.n_agents = 9;
user_par.IK = 1;
user_par.mdl = 2;
user_par.x0 = [500 500 1000];
user_par.seed = 6;
variable_param.prob_connection = 1;
variable_param.prob_rel_measurement = 1;
variable_param.prob_GPS = 1;
par.parametric = 1;
par = parameters(variable_param, user_par, par);

post_process_data = load('post_process_data_9_agents.mat');
post_process_data.post_process_data{5}.agents{1}.loc_error_mean{9} = [NaN, NaN, NaN]';
post_process_data.post_process_data{5}.agents{1}.loc_error_mean_after_wls{9} = [NaN, NaN, NaN]';

plot_chutes_trajectory([], [], 1, [], par, post_process_data.post_process_data, 1);