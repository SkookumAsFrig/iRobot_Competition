clc
close all
clear all

origin = [0 0];
onepoint = [0 1];
otherpoint = [-1 -1];
matforplot = [onepoint; origin; otherpoint];

angle1 = FindAngLines(origin,onepoint,otherpoint)

plot(matforplot(:,1),matforplot(:,2));
axis equal