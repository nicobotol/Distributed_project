% Choose slash according to your operating system automatically
if ispc
  path = 'functions\';
else
  path = 'functions/';
end

addpath(path);

set(0,'DefaultFigureWindowStyle','docked');
set(0, 'DefaultAxesFontSize', 14)
set(0, 'DefaultTextFontSize', 14)
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

rng(6);                  % random number generator seed
