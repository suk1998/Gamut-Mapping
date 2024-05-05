% Curve Mode로 Color Extention 된 먼셀 데이터를 다시 읽고 CIE Diagram 에서 뿌린다
% Curve Mode 가 1일때는 Linear Mode, Curve Mode 가 0 일때는 위의 데이터를 뿌린다.

clear all;
close all;
clc;

curve_mode = 0;

if curve_mode ~= 1,
    in=xlsread('Neutral-125-sRGB_alienskinconversion.xlsx',1,'A:D');
else
    in=xlsread('Neutral-125-sRGB_alienskinconversion_curve.xlsx',1,'E:H');
end

D65x=[0.3127]; %D65 White Point
D65y=[0.3290];
Sx=[0.6400 0.3000 0.1500]; %sRGB gamut
Sy=[0.3300 0.6000 0.0600];
% DPx=[0.680 0.265 0.150] %DCI-P3 gamut
% DPy=[0.320 0.690 0.060]
ADx=[0.6400 0.2100 0.1500]; %Adobe RGB gamut
ADy=[0.3300 0.7100 0.0600];
% Ux=[0.708 0.170 0.131]%Rec 2020 gamut
% Uy=[0.292 0.797 0.046]
% Px=[0.7347 0.1596 0.0366] %ProPhoto
% Py=[0.2653 0.8404 0.0001]



source = in(:,[1:2]);
target = in(:,[3:4]);

origin = [D65x D65y];

[m,n]=size(source);
n=1;


%%%%%%%%%%%% newtarget = newsource + distance*direction %%%%%%%%%%%%%%%%%%%
if curve_mode ~= 1,
    for i=1:m,
        
        newsource(i-n+1,:) = source(i,:);
        
        dist(i-n+1) = pdist2(source(i,:),target(i,:),'euclidean');
        
        dir(i-n+1,:) = (source(i,:) - origin)/norm((source(i,:) - origin));
        
        newtarget(i-n+1,:) = source(i,:) +  dist(i-n+1)*dir(i-n+1,:);
        
    end    
    figure(1),
    cieplot();
    plot(newsource(:,1),newsource(:,2),'ko'); %Munsell 5B
    plot(D65x,D65y,'r*','markersize',10); %D65 White Point
    plot(newtarget(:,1),newtarget(:,2),'co');; %5B Linear Expansion
    hold on,
else        
    
    newsource = source;
    newtarget = target;
    
    figure(1),
    cieplot();
    plot(source(:,1),source(:,2),'ko'); %Munsell 5B
    plot(D65x,D65y,'r*','markersize',10); %D65 White Point
    plot(target(:,1),target(:,2),'co');; %5B Linear Expansion
    hold on,
end

k=convhull(Sx,Sy);
hold on,
plot(Sx(k),Sy(k),'b--','LineWidth',2);
axis equal;
hold on,

k=convhull(ADx,ADy);
hold on,
plot(ADx(k),ADy(k),'g--','LineWidth',2);
axis equal;
hold on,


for i=1:m,
    
    Bufx=[newsource(i,1) newtarget(i,1)];
    Bufy=[newsource(i,2) newtarget(i,2)];
    line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,
    
end
fprintf('sRGB and Extended RGB point plot !! \n');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%% loading image and covert XYZ %%%%%%%%%%%%%%%%%%%%%
img = imread('lowresolution\\florence-1936780_640.jpg');
img = imresize(img, 0.5);
[hgt, wid,color]=size(img);

imgSize = hgt*wid;
sRGB_XYZ_Sum = zeros(imgSize,1);
sRGB_z = zeros(imgSize,1);
sRGB_xy = zeros(imgSize,2);


RExyz(:,:,:) = RGBtoXYZ(img(:,:,1),img(:,:,2),img(:,:,3),'D65');
for i=1:hgt,
    for j=1:wid,
        x = RExyz(i,j,1)/(RExyz(i,j,1)+RExyz(i,j,2)+RExyz(i,j,3));
        y = RExyz(i,j,2)/(RExyz(i,j,1)+RExyz(i,j,2)+RExyz(i,j,3));
        
        xy((i-1)*wid+j,:) = [x y];
        z((i-1)*wid+j) = RExyz(i,j,3);
        
        dXYZ((i-1)*wid+j) = RExyz(i,j,1)+RExyz(i,j,2)+RExyz(i,j,3);
    end
end

plot(xy(:,1),xy(:,2),'r*');
hold off;

figure(2),
imshow(img);
imwrite(img,'InImg.tif');
fprintf('loading image and covert XYZ !! \n');
%%%%%%%%%%%%%%%%%%%% Inverse Image %%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=1:hgt,
%     for j=1:wid,          
%         
%        rewardxy = xy((i-1)*wid+j,:)*dXYZ((i-1)*wid+j);
%        rexyz = [rewardxy z((i-1)*wid+j)];
%        
%        rexyz = rexyz./100.0;
%        
%        bufRimg = xyz2srgb(rexyz);
%        Rimg(i,j,1) = bufRimg(1);
%        Rimg(i,j,2) = bufRimg(2);
%        Rimg(i,j,3) = bufRimg(3);
%           
%                
%     end
% end
% Rimg = uint8(Rimg);
% figure(5),
% imshow(Rimg);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% find Point(sRGB) having minimal distance from x,y(image) to srgb %%%%%%
E = newtarget;
X = newsource;
Y = xy;
clear newtarget newsource xy;

figure(3),
cieplot();
k=convhull(Sx,Sy);
hold on,
plot(Sx(k),Sy(k),'b--o','LineWidth',2)
hold on,
k=convhull(ADx,ADy);
hold on,
plot(ADx(k),ADy(k),'g--','LineWidth',2);
hold on,


plot(X(:,1),X(:,2),'ko'); %Munsell 5B
hold on,
plot(Y(:,1),Y(:,2),'ro'); %Munsell 5B
hold on,


for i=1:hgt,
    for j=1:wid,
         tmp = Y((i-1)*wid+j,:);
         D = pdist2(X,tmp,'euclidean'); % euclidean distance
         [val inx]=min(D);
         
         Bufx=[tmp(1) X(inx,1)];
         Bufy=[tmp(2) X(inx,2)];
         line(Bufx,Bufy,'Color','r','LineWidth',1);
         hold on,    
   
         newXY((i-1)*wid+j,1) = E(inx,1);
         newXY((i-1)*wid+j,2) = E(inx,2);
   
         Bufx=[tmp(1) E(inx,1)];
         Bufy=[tmp(2) E(inx,2)];
         line(Bufx,Bufy,'Color','g','LineWidth',1);
    end
end
plot(newXY(:,1),newXY(:,2),'go'); %Munsell 5B
axis equal;
grid on;
hold off;
clearvars  -except newXY dXYZ z hgt wid;
fprintf('Find minimal distance sRGB Point !! \n');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 
%%%%%%%%%%%%% For extended srgb(x %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:hgt,
    for j=1:wid,       
               
       conxy = newXY((i-1)*wid+j,:)*dXYZ((i-1)*wid+j);
       conxyz = [conxy z((i-1)*wid+j)];
       
       conxyz = conxyz./100.0;
       
       RGBimg = xyz2srgb(conxyz);
       fimg(i,j,1) = RGBimg(1);
       fimg(i,j,2) = RGBimg(2);
       fimg(i,j,3) = RGBimg(3);    
               
    end
end
fimg = uint8(fimg);
figure(4),
imshow(fimg);
imwrite(fimg,'OutImg.tif');
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

