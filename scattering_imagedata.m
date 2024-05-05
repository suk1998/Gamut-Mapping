% When an image is imported, this function scatters its xy data on a CIE
% diagram. 

clear all;
close all;
clc;

in=xlsread('Munsell_sRGB_C_D65_photoshop.xlsx',1,'E:H');

D65x=[0.3127] %D65 White Point
D65y=[0.3290]
Ex=[0.5100 0.3100 0.2300]
Ey=[0.3200 0.4800 0.1900]
Tx=[0.570 0.300 0.190]%Toy Gamut
Ty=[0.320 0.530 0.130]
Sx=[0.6400 0.3000 0.1500] %sRGB gamut
Sy=[0.3300 0.6000 0.0600]
% DPx=[0.680 0.265 0.150] %DCI-P3 gamut
% DPy=[0.320 0.690 0.060]
%ADx=[0.6400 0.2100 0.1500] %Adobe RGB gamut
%ADy=[0.3300 0.7100 0.0600]
% Ux=[0.708 0.170 0.131]%Rec 2020 gamut
% Uy=[0.292 0.797 0.046]
% Px=[0.7347 0.1596 0.0366] %ProPhoto
% Py=[0.2653 0.8404 0.0001]


figure(1)
cieplot();

%scatter(D65x,D65y,30,'w','filled'); %D65 White Point

hold on

k=convhull(Ex,Ey);
plot(Ex(k),Ey(k),'y--o','LineWidth',2,'markersize',4);
hold on

k=convhull(Sx,Sy);
plot(Sx(k),Sy(k),'m--o','LineWidth',2,'markersize',4);
hold on

k=convhull(Tx,Ty);
plot(Tx(k),Ty(k),'c--o','LineWidth',2,'markersize',4);
hold on

img = imread('Toy_gamut\\InputImage5.png');
img = imresize(img, 1.0);
[hgt, wid, color]=size(img);

for i=1:hgt
    for j=1:wid
        
        rgbimg(1) = img(i,j,1);
        rgbimg(2) = img(i,j,2);
        rgbimg(3) = img(i,j,3);
        
        rgbimg = double(rgbimg);
        
        RExyz = srgb2xyz(rgbimg);
        
        x = RExyz(1)/(RExyz(1)+RExyz(2)+RExyz(3));
        y = RExyz(2)/(RExyz(1)+RExyz(2)+RExyz(3));
        
        bufxy=[x y];
        dXYZ((i-1)*wid+j) = RExyz(1) +RExyz(2) + RExyz(3);
        
        xy ((i-1)*wid+j,:) = bufxy;
        z((i-1)*wid+j) = RExyz(3);
        
    end
end

scatter(xy(:,1),xy(:,2),30,'*','r');

scatter(D65x,D65y,30,'w','filled'); %D65 White Point

figure(2)
imshow(img);
