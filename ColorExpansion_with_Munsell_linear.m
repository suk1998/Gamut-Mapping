clear all;
close all;
clc;


in=xlsread('Neutral-125-sRGB_alienskinconversion.xlsx',1,'A:D');

D65x=[0.3127] %D65 White Point
D65y=[0.3290]
Sx=[0.6400 0.3000 0.1500] %sRGB gamut
Sy=[0.3300 0.6000 0.0600]
% DPx=[0.680 0.265 0.150] %DCI-P3 gamut
% DPy=[0.320 0.690 0.060]
% ADx=[0.6400 0.2100 0.1500] %Adobe RGB gamut
% ADy=[0.3300 0.7100 0.0600]
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
for i=1:m
    
                
    newsource(i-n+1,:) = source(i,:);
    
    dist(i-n+1) = pdist2(source(i,:),target(i,:),'euclidean');
    
    dir(i-n+1,:) = (source(i,:) - origin)/norm((source(i,:) - origin));
    
    newtarget(i-n+1,:) = source(i,:) +  dist(i-n+1)*dir(i-n+1,:);
    
end


figure(1)
cieplot();
scatter(newsource(:,1),newsource(:,2),50,'k'); %Munsell 5B
scatter(D65x,D65y,30,'*','r'); %D65 White Point
scatter(newtarget(:,1),newtarget(:,2),50,'c','filled'); %5B Linear Expansion
hold on

k=convhull(Sx,Sy);
hold on
plot(Sx(k),Sy(k),'b--o','LineWidth',2,'markersize',4);
axis equal;
hold off


for i=1:m
    
    Bufx=[newsource(i,1) newtarget(i,1)];
    Bufy=[newsource(i,2) newtarget(i,2)];
    line(Bufx,Bufy,'Color','k','LineWidth',1);hold on
    
end
%saveas(gcf,'expansion_chr.tif');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%% loading image and covert XYZ %%%%%%%%%%%%%%%%%%%%%
img = imread('reformatted_images\\13.jpg');
img = imresize(img, 1.0);
[hgt, wid,color]=size(img);

for i=1:hgt
    for j=1:wid
        
        rgbimg(1) = img(i,j,1);
        rgbimg(2) = img(i,j,2);
        rgbimg(3) = img(i,j,3);        
        
        rgbimg = double(rgbimg);
               
        RExyz = RGBtoXYZ(rgbimg(1),rgbimg(2),rgbimg(3),'D65');
        
        x = RExyz(1)/(RExyz(1)+RExyz(2)+RExyz(3));
        y = RExyz(2)/(RExyz(1)+RExyz(2)+RExyz(3));
        
        
        bufxy = [x y];
       
        dXYZ((i-1)*wid+j) = RExyz(1)+RExyz(2)+RExyz(3);
        
        xy((i-1)*wid+j,:) = bufxy;
        z((i-1)*wid+j) = RExyz(3);       
        
    end
end

%scatter(xy(:,1),xy(:,2),30,'*','r'); %Plot x,y(image)
    
figure(2)
imshow(img);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






%% find target points have minimal distance from x,y(image)  to source point %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
E = newtarget;
X = newsource;
Y = xy;
clear newtarget newsource xy;

figure(3)
cieplot();
k=convhull(Sx,Sy);
hold on;
plot(Sx(k),Sy(k),'b--o','LineWidth',2,'markersize',4)
hold on;

scatter(X(:,1),X(:,2),5,'K'); %Munsell 5B
hold on;
scatter(Y(:,1),Y(:,2),5,'r'); %Munsell 5B
hold on;


for i=1:hgt
    for j=1:wid
         tmp = Y((i-1)*wid+j,:);
         D = pdist2(X,tmp,'euclidean'); % euclidean distance
         [val inx]=min(D);
         
         Bufx=[tmp(1) X(inx,1)];
         Bufy=[tmp(2) X(inx,2)];
         line(Bufx,Bufy,'Color','r','LineWidth',1);
         hold on;    
   
         newXY((i-1)*wid+j,1) = E(inx,1);
         newXY((i-1)*wid+j,2) = E(inx,2);
   
         Bufx=[tmp(1) E(inx,1)];
         Bufy=[tmp(2) E(inx,2)];
         line(Bufx,Bufy,'Color','g','LineWidth',1);
    end
end
scatter(newXY(:,1),newXY(:,2),5,'g'); %Munsell 5B
axis equal;
grid on;
clearvars  -except newXY dXYZ z hgt wid;

%D = pdist2(X,Y,'euclidean'); % euclidean distance
%[val inx]=min(D);



% for i=1: max(size(inx))
%     
%    Bufx=[Y(i,1) X(inx(i),1)];
%    Bufy=[Y(i,2) X(inx(i),2)];
%    line(Bufx,Bufy,'Color','r','LineWidth',1);
%    hold on,    
%    
%    newXY(i,1) = E(inx(i),1);
%    newXY(i,2) = E(inx(i),2);
%    
%    Bufx=[Y(i,1) E(inx(i),1)];
%    Bufy=[Y(i,2) E(inx(i),2)];
%    line(Bufx,Bufy,'Color','g','LineWidth',1);
%    
% end
% scatter(newXY(:,1),newXY(:,2),5,'g'); %Munsell 5B
% axis equal;
% grid on;

%clearvars  -except newXY dXYZ z hgt wid;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure(4),
for i=1:hgt
    for j=1:wid       
               
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
figure(5)
imshow(fimg);
clear all;
% 
% 
% 
% 
% 
% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% for i=1:hgt
%     for j=1:wid
%           
%         
%        rewardxy = xy((i-1)*wid+j,:)*dXYZ((i-1)*wid+j);
%        rexyz = [rewardxy z((i-1)*wid+j)];
%        
%        rexyz = rexyz./100.0;
%        
%        bufRimg = xyz2srgb(rexyz);
%        cimg(i,j,1) = bufRimg(1);
%        cimg(i,j,2) = bufRimg(2);
%        cimg(i,j,3) = bufRimg(3);
%           
%                
%     end
% end
% cimg = uint8(cimg);
% figure(3)
% imshow(cimg);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
