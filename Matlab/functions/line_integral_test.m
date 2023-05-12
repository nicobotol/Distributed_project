clc;
close all;
clear all;

%% Test

syms F1(x,y) F2(x,y) r1(t) r2(t)
edges 
r1(t) = cos(t)+2;
r2(t) = sin(t);
F1(x,y) = x;
F2(x,y) = 0;
r = [r1;r2];
F = [F1;F2];
Fr = F(r1(t),r2(t));
dr = diff(r(t),t);
integrand = dot(Fr,dr);
work = int(integrand,t,[0,2*pi])