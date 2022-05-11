clc; close all; clear all;

M = csvread('tarjei3.csv');

q1_calc = M(:,6);
q1_enc = M(:,10);
q2_calc = M(:,8);
q2_enc = M(:,12);

t = M(:,5);

hold on
plot(t,q1_calc)
%plot(t,q1_enc)