% Linear expansion with Logarithmic plot 

clear; clc; close all

Sx=[0.6400  0.3000  0.1500];
Sy=[0.3300  0.6000  0.0600];
ADx=[0.6400 0.2100  0.1500];
ADy=[0.3300 0.7100  0.0600];
D65x=[0.3127];
D65y=[0.3290];
Ovec =[1 0];

Origin = [D65x;D65y];

filename = ['reformatted_images\\house-1834826_640'];
formatfile = ['.jpg'];
readfilename = [filename formatfile];
img = imread(readfilename);
img = imresize(img, 0.1);
[hgt, wid, color] = size(img);

imgSize = hgt*wid

sRGB_XYZ_Sum = zeros(imgSize,1);
sRGB_z = zeros(imgSize,1);
sRGB_xy = zeros(imgSize,2);
sRGB_xy_Exp = zeros(imgSIze,2);

for i=1:hgt
    for j=1:wid
        
        buf(1) = img(i,j,1);
        buf(2) = img(i,j,2);
        buf(3) = img(i,j,3);
        
        buf = double(buf);
        
        RExyz =srgb2XYZ(buf);
        
        