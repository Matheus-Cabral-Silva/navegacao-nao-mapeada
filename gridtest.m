clc;
clear all;
image = imread('sala_comum.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
map = binaryOccupancyMap(bwimage,100);
save sala_comum map
show(map)