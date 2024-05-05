% Following Munsell Linear Distance-When an image is inputted, each color point of
% the image tries to find the nearst munsell point and follows it. 2016.12
% This is a revised version to import and process all the files

clear all;
close all;
clc;

for indxx=1 : 60
    in=xlsread('Alienskin.xlsx',1,'A:D');

    D65x=[0.3127]; %D65 White Point
    D65y=[0.3290];
    Sx=[0.6400 0.3000 0.1500]; %sRGB gamut
    Sy=[0.3300 0.6000 0.0600];
    % DPx=[0.680 0.265 0.150] %DCI-P3 gamut
    % DPy=[0.320 0.690 0.060]
    ADx=[0.6400 0.2100 0.1500] %Adobe RGB gamut
    ADy=[0.3300 0.7100 0.0600]
    % Ux=[0.708 0.170 0.131]%Rec 2020 gamut
    % Uy=[0.292 0.797 0.046]
    % Px=[0.7347 0.1596 0.0366] %ProPhoto
    % Py=[0.2653 0.8404 0.0001]
    % 
    % 
    % figure(1),
    % cieplot();
    % 
    % x=in(:,1);
    % y=in(:,2);
    % Ax=in(:,3);
    % Ay=in(:,4);
    % 
    % scatter(x,y,50,'k'); %Munsell 5B
    % scatter(D65x,D65y,30,'*','r'); %D65 White Point
    % scatter(Ax,Ay,50,'c','filled'); %5B Linear Expansion



    source = in(:,[1:2]);
    target = in(:,[3:4]);

    origin = [D65x D65y];

    [m,n]=size(source);
    n=1;


    % n=1253,m=1281;


    %%%%%%%%%%%% newtarget = newsource + distance*direction %%%%%%%%%%%%%%%%%


    for i=n:m
        newsource(i-n+1,:) = source(i,:);

        dist(i-n+1) = pdist2(source(i,:),target(i,:),'euclidean');

        dir(i-n+1,:) = (source(i,:) - origin)/norm((source(i,:) - origin));

        newtarget(i-n+1,:) = source(i,:) +  dist(i-n+1)*dir(i-n+1,:);

    end


    h = figure(1);
    cieplot();
    scatter(newsource(:,1),newsource(:,2),50,'k'); %neutral
    scatter(D65x,D65y,30,'*','r'); %D65 White Point
    scatter(newtarget(:,1),newtarget(:,2),50,'c','filled'); % Linear Expansion
    hold on


    k=convhull(Sx,Sy);
    hold on
    plot(Sx(k),Sy(k),'b--o','LineWidth',2,'markersize',4)
    hold on
    
    k=convhull(ADx,ADy);
    hold on
    plot(ADx(k),ADy(k),'y--o','LineWidth',2,'markersize',4)
    hold on

    m = m-n+1;

    for i=1:m
     Bufx=[newsource(i,1) newtarget(i,1)];
     Bufy=[newsource(i,2) newtarget(i,2)];
     line(Bufx,Bufy,'Color','k','LineWidth',1); hold on

    end


    %FileName = './reformatted_images/1.fig'
    %savefig(h,FileName);
    


    filename = sprintf('./reformatted_images/%d.jpg',indxx)
    img = imread(filename);
    
    img = imresize(img, 1.0);

    [hgt, wid,color]=size(img);

    for i=1:hgt
        for j=1:wid

            rgbimg(1) = img(i,j,1);
            rgbimg(2) = img(i,j,2);
            rgbimg(3) = img(i,j,3);        

            rgbimg = double(rgbimg);

            RExyz = srgb2xyz(rgbimg);

            x = RExyz(1)/(RExyz(1)+RExyz(2)+RExyz(3));
            y = RExyz(2)/(RExyz(1)+RExyz(2)+RExyz(3));


            bufxy = [x y];

            dXYZ((i-1)*wid+j) = RExyz(1)+RExyz(2)+RExyz(3);

            xy((i-1)*wid+j,:) = bufxy;
            z((i-1)*wid+j) = RExyz(3);

            %rexyz = [xy((i-1)*wid+j,:) z((i-1)*wid+j)];

        end
    end
    scatter(xy(:,1),xy(:,2),30,'*','r'); %D65 White Point



  %  h =figure(2);
  %  imshow(img);
    
  
    FileName = sprintf('./reformatted_images/%d_2.png',indxx);
    imwrite(img,FileName)
    
  %  savefig(h,FileName);
  %FileName2 = sprintf('./reformatted_images/%d_2_fig.bmp',indxx);
  %saveas(h,FileName2);

%     for i=1:hgt
%         for j=1:wid
% 
%            rewardxy = xy((i-1)*wid+j,:)*dXYZ((i-1)*wid+j);
%            rexyz = [rewardxy z((i-1)*wid+j)];
% 
% 
%            bufRimg = xyz2srgb(rexyz);
%            cimg(i,j,1) = bufRimg(1);
%            cimg(i,j,2) = bufRimg(2);
%            cimg(i,j,3) = bufRimg(3);
% 
%            %[bufR,bufG,bufB]=xyz2rgb(rexyz(1),rexyz(2),rexyz(3));
%            %cimg(i,j,1) = bufR;
%            %cimg(i,j,2) = bufG;
%            %cimg(i,j,3) = bufB;
% 
%         end
%     end
%     cimg = uint8(cimg);
%    % h =figure(3);
%     imshow(cimg);
%     
%    % FileName = sprintf('./lowresolution/%d_3.fig',indxx);
%     %savefig(h,FileName);
%     FileName3 = sprintf('./lowresolution/%d_3.png',indxx);
%     imwrite(cimg,FileName3)
%     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



    E = newtarget;
    X = newsource;
    Y = xy;
    D = pdist2(X,Y,'euclidean'); % euclidean distance



    [val inx]=min(D);

    %h4 = figure(4);
    figure(4);
    cieplot();
    k=convhull(Sx,Sy);
    hold on;
   plot(Sx(k),Sy(k),'b--o','LineWidth',2,'markersize',4);
   hold on;

     scatter(X(:,1),X(:,2),5,'K'); %Munsell 
    hold on;
     scatter(Y(:,1),Y(:,2),5,'r'); %Munsell 
    hold on;


    for i=1: max(size(inx))

       Bufx=[Y(i,1) X(inx(i),1)];
       Bufy=[Y(i,2) X(inx(i),2)];
       line(Bufx,Bufy,'Color','r','LineWidth',1);
       hold on;    

       newXY(i,1) = E(inx(i),1);
       newXY(i,2) = E(inx(i),2);

       Bufx=[Y(i,1) E(inx(i),1)];
       Bufy=[Y(i,2) E(inx(i),2)];
       line(Bufx,Bufy,'Color','g','LineWidth',1);

    end
    scatter(newXY(:,1),newXY(:,2),5,'g'); %converted data

    axis equal;
    grid on;
    
    clear img cimg;

    %FileName = sprintf('./reformatted_images/%d_4.fig',indxx);
    %savefig(h,FileName);
    %  FileName = sprintf('./reformatted_images/%d_2.fig',indxx);
   % FileName4 = sprintf('./reformatted_images/%d_4_fig.bmp',indxx);
   % saveas(h,FileName4);

   % figure(5);
    for i=1:hgt
        for j=1:wid


           conxy = newXY((i-1)*wid+j,:)*dXYZ((i-1)*wid+j);
           conxyz = [conxy z((i-1)*wid+j)];


           RGBimg = xyz2srgb(conxyz);
           fimg(i,j,1) = RGBimg(1);
           fimg(i,j,2) = RGBimg(2);
           fimg(i,j,3) = RGBimg(3);

           %[bufR,bufG,bufB]=xyz2rgb(rexyz(1),rexyz(2),rexyz(3));
           %cimg(i,j,1) = bufR;
           %cimg(i,j,2) = bufG;
           %cimg(i,j,3) = bufB;

        end
    end
    fimg = uint8(fimg);
  %  h=figure(5);
  %  imshow(fimg);
  %  FileName = sprintf('./reformatted_images/%d_5.fig',indxx);
  %  savefig(h,FileName);
    FileName5 = sprintf('./reformatted_images/%d_5.png',indxx);
    imwrite(fimg,FileName5);

    %clear fimg;
    %clear dXYZ, xy, z;
    %clear E, X, Y, D;
    %clear X, Y,  newXY;
    
    clearvars -except indxx;
    
end






