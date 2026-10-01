close all; 
clc; clear;

points = readtable('datatrampa.txt');
points2 = readtable('dataset2.txt')
points3 = readtable('dataset3.txt')
points = points{:,:};
points2 = points2{:,:};
points3 = points3{:,:};

plot(points(:,1),points(:,3), 'DisplayName', "Datos1")
hold on
plot(points2(:,1),points2(:,3), 'DisplayName', "Datos2")
plot(points3(:,1),points3(:,3), 'DisplayName', "Datos3")

legend
%plot(points(:,1),points(:,3))
grid on
hold off