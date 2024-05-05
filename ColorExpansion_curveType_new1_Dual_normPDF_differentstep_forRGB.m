% Curve with hue Variation with two different Sigma values for distances.
% Each RGB has different distence values.
clear all;
close all;
clc;

Sx=[0.6400 0.3000 0.1500]; %sRGB gamut
Sy=[0.3300 0.6000 0.0600];
ADx=[0.6400 0.2100 0.1500]; %Adobe RGB gamut
ADy=[0.3300 0.7100 0.0600];
D65x=[0.3127];%D65 White Point
D65y=[0.3290];
Ovec = [1 0];

Origin = [D65x;D65y];

%%%%%%%%%%%%%%%%%%%%%%%%%% Setting mid Points%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:max(size(Sx)),
    if i==3,
        midP(i,1) = (Sx(i)+Sx(1))/2.0;
        midP(i,2) = (Sy(i)+Sy(1))/2.0;
    else    
        midP(i,1) = (Sx(i)+Sx(i+1))/2.0;
        midP(i,2) = (Sy(i)+Sy(i+1))/2.0;
    end
end

midPnt_RG = midP(1,:);
%midPnt_GB = midP(2,:);
midPnt_GB = [0.23 0.348];
midPnt_BR = midP(3,:);
clear midP;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%% loading Input Image %%%%%%%%%%%%%%%%%%%%%%%%%%%%

charmode = 0;

if charmode ==1,
    filename = ['Munsell_sRGB_C_D65_Alienskin' ];
    formatfile = ['.xlsx'];
    readfilename = [filename formatfile];
    in=xlsread(readfilename,1,'E:H');
    %in=xlsread('Alienskin.xlsx',1,'A:D');
    sRGB_xy = in(:,[1:2]);
    target = in(:,[3:4]);
    imgSize = max(size(sRGB_xy));
else
    filename = ['reformatted_images\\engagement-1718244_640'];
    formatfile = ['.jpg'];
    readfilename = [filename formatfile];
    img = imread(readfilename);
    img = imresize(img, 1.0);
    [hgt, wid,color]=size(img);
    
    imgSize = hgt*wid;
    
    sRGB_XYZ_Sum = zeros(imgSize,1);
    sRGB_z = zeros(imgSize,1);
    sRGB_xy = zeros(imgSize,2);
    sRGB_xy_Exp = zeros(imgSize,2);   
    
    %RExyz(:,:,:) = RGBtoXYZ(img(:,:,1),img(:,:,2),img(:,:,3),'D65');
        
    for i=1:hgt,
        for j=1:wid,
            
            %x = RExyz(i,j,1)/(RExyz(i,j,1)+RExyz(i,j,2)+RExyz(i,j,3));
            %y = RExyz(i,j,2)/(RExyz(i,j,1)+RExyz(i,j,2)+RExyz(i,j,3));
            %z = RExyz(i,j,3);
            %sRGB_xy((i-1)*wid+j,:) = [x y];
            %sRGB_z((i-1)*wid+j) = z;
            %sRGB_XYZ_Sum((i-1)*wid+j) = RExyz(i,j,1)+RExyz(i,j,2)+RExyz(i,j,3);           
            
            
            buf(1) = img(i,j,1);
            buf(2) = img(i,j,2);
            buf(3) = img(i,j,3);
            
            buf = double(buf);
            
            RExyz = srgb2xyz(buf);
           
            
            x = RExyz(1)/(RExyz(1)+RExyz(2)+RExyz(3));
            y = RExyz(2)/(RExyz(1)+RExyz(2)+RExyz(3));
            z = RExyz(3);
            
            
            sRGB_xy((i-1)*wid+j,:) = [x y];
            sRGB_z((i-1)*wid+j) = z;
            
            sRGB_XYZ_Sum((i-1)*wid+j) = RExyz(1)+RExyz(2)+RExyz(3);
        end
    end
    
    figure(1),
    imshow(img);
    fprintf('Input Images !!!\n');
    clear RExyz img;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%% left distance distribution %%%%%%%%%%%%%%%%%%%%%%%
% Sx=[0.6400 0.3000 0.1500]; %sRGB gamut
% Sy=[0.3300 0.6000 0.0600];
% ADx=[0.6400 0.2100 0.1500]; %Adobe RGB gamut
% ADy=[0.3300 0.7100 0.0600];
% D65x=[0.3127];%D65 White Point
% D65y=[0.3290];

MaxStep = 0.06;
sig1 = 0.06;
sig2 = 0.07;
sig3 = 0.09;
sig4 = 0.1;

RMeanPercent = 0.6;
GMeanPercent = 0.7;
BMeanPercent = 0.5;

R_Maxdis = pdist2([D65x D65y],[ADx(1) ADy(1)],'euclidean');
G_Maxdis = pdist2([D65x D65y],[ADx(2) ADy(2)],'euclidean');
B_Maxdis = pdist2([D65x D65y],[ADx(3) ADy(3)],'euclidean');


R_disMean = max([R_Maxdis G_Maxdis B_Maxdis])*RMeanPercent;
G_disMean = max([R_Maxdis G_Maxdis B_Maxdis])*GMeanPercent;
B_disMean = max([R_Maxdis G_Maxdis B_Maxdis])*BMeanPercent;


%%%%%%%%%%% Red distance distribution %%%%%%%%%%
xn = [0:0.001:R_Maxdis];

yn = normpdf(xn,R_disMean,sig2);
Rdegree_ratio1 = MaxStep/max(yn);

xn_1 = [0:0.001:R_disMean];
yn_1 = normpdf(xn_1,R_disMean,sig2)*Rdegree_ratio1;

yn = normpdf(xn,R_disMean,sig4);
Rdegree_ratio2 = MaxStep/max(yn);

xn_2 = [R_disMean+0.001:0.001:R_Maxdis];
yn_2 = normpdf(xn_2,R_disMean,sig4)*Rdegree_ratio2;
    
xn = [xn_1 xn_2];
yn = [yn_1 yn_2];

figure(2), plot(xn,yn,'r-'),grid on,hold on,
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% Green distance distribution %%%%%%%%%%
xn = [0:0.001:G_Maxdis];

yn = normpdf(xn,G_disMean,sig3);
Gdegree_ratio1 = MaxStep/max(yn);

xn_1 = [0:0.001:G_disMean];
yn_1 = normpdf(xn_1,G_disMean,sig3)*Gdegree_ratio1;

yn = normpdf(xn,G_disMean,sig4);
Gdegree_ratio2 = MaxStep/max(yn);

xn_2 = [G_disMean+0.001:0.001:G_Maxdis];
yn_2 = normpdf(xn_2,G_disMean,sig4)*Gdegree_ratio2;
    
xn = [xn_1 xn_2];
yn = [yn_1 yn_2];
plot(xn,yn,'g-'),grid on,hold on,
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% Blue distance distribution %%%%%%%%%%
xn = [0:0.001:B_Maxdis];

yn = normpdf(xn,B_disMean,sig1);
Bdegree_ratio1 = MaxStep/max(yn);

xn_1 = [0:0.001:B_disMean];
yn_1 = normpdf(xn_1,B_disMean,sig1)*Bdegree_ratio1;

yn = normpdf(xn,B_disMean,sig4);
Bdegree_ratio2 = MaxStep/max(yn);

xn_2 = [B_disMean+0.001:0.001:B_Maxdis];
yn_2 = normpdf(xn_2,B_disMean,sig4)*Bdegree_ratio2;
    
xn = [xn_1 xn_2];
yn = [yn_1 yn_2];
plot(xn,yn,'b-'),grid on,hold off,
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%% CIE Domain %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3),
cieplot();
plot(sRGB_xy(:,1),sRGB_xy(:,2),'k+','markersize',4); %sRGB
plot(Origin(1),Origin(2),'r*','markersize',4); %D65 White Point
plot(midPnt_RG(1),midPnt_RG(2),'r+','markersize',4); % mid Point
plot(midPnt_GB(1),midPnt_GB(2),'g+','markersize',4); % mid Point
plot(midPnt_BR(1),midPnt_BR(2),'b+','markersize',4); % mid Point
axis equal;
hold on,

k=convhull(Sx,Sy);
plot(Sx(k),Sy(k),'r--','LineWidth',2,'markersize',4);
hold on,

k=convhull(ADx,ADy);
plot(ADx(k),ADy(k),'g--','LineWidth',2,'markersize',4);
hold on;
fprintf('Plot xy !!!\n');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%% sRGB Point clustering (R,G,B region) %%%%%%%%%%%%%%%%%%%%%%

R_polygonX = [Sx(1) midPnt_RG(1) Origin(1) midPnt_BR(1) Sx(1)];
R_polygonY = [Sy(1) midPnt_RG(2) Origin(2) midPnt_BR(2) Sy(1)];

[inR] = inpolygon(sRGB_xy(:,1),sRGB_xy(:,2),R_polygonX,R_polygonY);

sRGB_R = sRGB_xy(inR,:);
if charmode ~=1,
    sRGB_R_z = sRGB_z(inR);
    sRGB_R_XYZ = sRGB_XYZ_Sum(inR);
end

plot(R_polygonX,R_polygonY); % polygon
hold on,
plot(sRGB_R(:,1),sRGB_R(:,2),'r+') % points inside
hold on,


G_polygonX = [Sx(2) midPnt_GB(1) Origin(1) midPnt_RG(1) Sx(2)];
G_polygonY = [Sy(2) midPnt_GB(2) Origin(2) midPnt_RG(2) Sy(2)];

[inG] = inpolygon(sRGB_xy(:,1),sRGB_xy(:,2),G_polygonX,G_polygonY);

sRGB_G = sRGB_xy(inG,:);
if charmode ~=1,
    sRGB_G_z = sRGB_z(inG);
    sRGB_G_XYZ = sRGB_XYZ_Sum(inG);
end
plot(G_polygonX,G_polygonY); % polygon
hold on,
plot(sRGB_G(:,1),sRGB_G(:,2),'g+') % points inside
hold on,


B_polygonX = [Sx(3) midPnt_BR(1) Origin(1) midPnt_GB(1) Sx(3)];
B_polygonY = [Sy(3) midPnt_BR(2) Origin(2) midPnt_GB(2) Sy(3)];

[inB] = inpolygon(sRGB_xy(:,1),sRGB_xy(:,2),B_polygonX,B_polygonY);

sRGB_B = sRGB_xy(inB,:);
if charmode ~=1,
    sRGB_B_z = sRGB_z(inB);
    sRGB_B_XYZ = sRGB_XYZ_Sum(inB);
end
plot(B_polygonX,B_polygonY); % polygon
hold on,
plot(sRGB_B(:,1),sRGB_B(:,2),'b+') % points inside
hold on,
fprintf('xy(RGB) Clustering !!!\n');
clear sRGB_R sRGB_G sRGB_B;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%File recording %%%%%%%%%%%%%%%%%%%
if charmode ==1,
    reCnt = 1;
    recording{reCnt,1} = '';
    recording{reCnt,2} = 'h';
    recording{reCnt,3} = 'V';
    recording{reCnt,4} = 'C';
    
    recording{reCnt,5} = 'x';
    recording{reCnt,6} = 'y';
    
    recording{reCnt,7} = 'Ax';
    recording{reCnt,8} = 'Ay';
    reCnt=reCnt+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Red %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('sRGB(xy) Expansion !!!\n');
xnum = 30;
min_dis = 0.05;
max_dis = 0.5;
tic
for i=1:imgSize,
    
        if inR(i)==1,      
            
            Rvec = [(ADx(1)-D65x) (ADy(1)-D65y)];
            Rvec = Rvec/norm(Rvec);
            theta= acos(Ovec*Rvec');
            RotMat = [cos(theta) -sin(theta);sin(theta) cos(theta)];
            
            D65xy = RotMat*[D65x;D65y];
            OrignX = D65xy(1);
            OrignY = D65xy(2);
            
            ADxy = RotMat*[ADx;ADy];
            Ax = ADxy(1,:);
            Ay = ADxy(2,:);
            
            
            sRGB_R1 = RotMat*sRGB_xy(i,:)';
            sRGB_R1=sRGB_R1';
            
                
            Pnt = [sRGB_R1(1) sRGB_R1(2)];
            x = [Ax(1) OrignX Pnt(1)];
            y = [Ay(1) OrignY Pnt(2)];
            
            tmpx = [Ax(3) OrignX Pnt(1)];
            tmpy = [Ay(3) OrignY Pnt(2)];
            
            p = polyfit(x,y,2);
            
            x1 = linspace(min(x),max(x),xnum);
            y1 =  p(1)*x1.^2+p(2)*x1+p(3);
            
            circle = @(x, y, r) rectangle('Position', [x-r, y-r, 2*r, 2*r], ...
                'Curvature', [1 1]);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if charmode ==1, % curve expansion 데이터로 보고 싶을 때는 0 아니면 먼셀데이터로 보고 싶으면 1
                A = [sRGB_xy(i,1) sRGB_xy(i,2)];
                B = [target(i,1) target(i,2)];
                r_sq = pdist2(A,B,'euclidean');
                if r_sq == 0.0,
                    r_sq = 0.0004;
                else
                    r_sq = r_sq^2;
                end
            else
                % r_sq = 0.0050;
                %%%% revision 2017.02.02 : 원점 근처의 sRGB는 고정.
                A = [sRGB_xy(i,1) sRGB_xy(i,2)];
                B = [D65x D65y];
                r_sq = pdist2(A,B,'euclidean');
     
                if r_sq <= R_disMean,
                    r_sq = normpdf(r_sq,R_disMean,sig1)*Rdegree_ratio1;
                else
                    r_sq = normpdf(r_sq,R_disMean,sig2)*Rdegree_ratio2;
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            syms u v;
            f1 = (u-x(3))^2+(v-y(3))^2-r_sq;
            f2 = p(1)*u^2+p(2)*u+p(3)-v;
            [u,v]=solve(f1,f2);
            
            solx = find(imag(u)==0);
            soly = find(imag(v)==0);
            
            ADxy=[x(1) y(1)];
            Solxy=[double(u(solx)) double(v(soly))];
            
            dist = pdist2(ADxy,Solxy,'euclidean');
            
            [val index]=min(dist);
            
            tarPnt = [double(u(index));double(v(index))];
            
            x2 = linspace(min(x(3),tarPnt(1)),max(x(3),tarPnt(1)),xnum);
            y2 =  p(1)*x2.^2+p(2)*x2+p(3);
            
            %%%%%%%%%%%%%% inverse rotation %%%%%%%%%%%%
            tarPnt = RotMat'*tarPnt;
            xy = RotMat'*[x;y];
            x = xy(1,:);
            y = xy(2,:);
            
            xy1 = RotMat'*[x1;y1];
            x1 = xy1(1,:);
            y1 = xy1(2,:);           
            
            xy2 = RotMat'*[x2;y2];
            x2 = xy2(1,:);
            y2 = xy2(2,:);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%% Recording %%%%%%%%%%%%%%%%%
            if charmode ==1,
                recording{reCnt,5} = x(3);
                recording{reCnt,6} = y(3);
                
                recording{reCnt,7} = tarPnt(1);
                recording{reCnt,8} = tarPnt(2);
                reCnt=reCnt+1;
            else
                sRGB_xy_Exp(i,:) = [tarPnt(1) tarPnt(2)];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %circle(x(3),y(3), sqrt(r_sq)), hold on,
            %plot(x,y,'ro','markersize',7), hold on,
            %plot(x1,y1,'k--');
            %hold on;
            
            plot(x(3),y(3),'go','markersize',4,'LineWidth',2), hold on,
            plot(tarPnt(1),tarPnt(2),'bo','markersize',4,'LineWidth',2), hold on,
            
            plot(x2,y2,'b--','LineWidth',2);
            hold on;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        end
       
        
        
        if inG(i)==1,   
            
            Gvec = [(ADx(2)-D65x(1)) (ADy(2)-D65y(1))];
            Gvec = Gvec/norm(Gvec);
            theta= acos(Ovec*Gvec');
            RotMat = [cos(theta) sin(theta);-sin(theta) cos(theta)];
           
            
            D65xy = RotMat*[D65x;D65y];
            OrignX = D65xy(1);
            OrignY = D65xy(2);
            
            ADxy = RotMat*[ADx;ADy];
            Ax = ADxy(1,:);
            Ay = ADxy(2,:);
            
            sRGB_G1 = RotMat*sRGB_xy(i,:)';
            sRGB_G1=sRGB_G1';
            
           
                
            Pnt = [sRGB_G1(1) sRGB_G1(2)];
            x = [Ax(2) OrignX Pnt(1)];
            y = [Ay(2) OrignY Pnt(2)];
            
            tmpx = [Ax(3) OrignX Pnt(1)];
            tmpy = [Ay(3) OrignY Pnt(2)];
            
            p = polyfit(x,y,2);
            
            x1 = linspace(min(x),max(x),xnum);
            y1 =  p(1)*x1.^2+p(2)*x1+p(3);
            
            circle = @(x, y, r) rectangle('Position', [x-r, y-r, 2*r, 2*r], ...
                'Curvature', [1 1]);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if charmode ==1, % curve expansion 데이터로 보고 싶을 때는 0 아니면 먼셀데이터로 보고 싶으면 1
                A = [sRGB_xy(i,1) sRGB_xy(i,2)];
                B = [target(i,1) target(i,2)];
                r_sq = pdist2(A,B,'euclidean');
                if r_sq == 0.0,
                    r_sq = 0.0004;
                else
                    r_sq = r_sq^2;
                end
            else
                % r_sq = 0.0050;
                %%%% revision 2017.02.02 : 원점 근처의 sRGB는 고정.
                A = [sRGB_xy(i,1) sRGB_xy(i,2)];
                B = [D65x D65y];
                r_sq = pdist2(A,B,'euclidean');
                
                if r_sq <= G_disMean,
                    r_sq = normpdf(r_sq,G_disMean,sig1)*Gdegree_ratio1;
                else
                    r_sq = normpdf(r_sq,G_disMean,sig2)*Gdegree_ratio2;
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            syms u v;
            f1 = (u-x(3))^2+(v-y(3))^2-r_sq;
            f2 = p(1)*u^2+p(2)*u+p(3)-v;
            [u,v]=solve(f1,f2);
            
            solx = find(imag(u)==0);
            soly = find(imag(v)==0);
            
            ADxy=[x(1) y(1)];
            Solxy=[double(u(solx)) double(v(soly))];
            
            dist = pdist2(ADxy,Solxy,'euclidean');
            
            [val index]=min(dist);
            
            tarPnt = [double(u(index));double(v(index))];
            
            x2 = linspace(min(x(3),tarPnt(1)),max(x(3),tarPnt(1)),xnum);
            y2 =  p(1)*x2.^2+p(2)*x2+p(3);
            
            %%%%%%%%%%%%%% inverse rotation %%%%%%%%%%%%
            tarPnt = RotMat'*tarPnt;
            xy = RotMat'*[x;y];
            x = xy(1,:);
            y = xy(2,:);
            
            xy1 = RotMat'*[x1;y1];
            x1 = xy1(1,:);
            y1 = xy1(2,:);           
            
            xy2 = RotMat'*[x2;y2];
            x2 = xy2(1,:);
            y2 = xy2(2,:);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%% Recording %%%%%%%%%%%%%%%%%
            if charmode ==1,
                recording{reCnt,5} = x(3);
                recording{reCnt,6} = y(3);
                
                recording{reCnt,7} = tarPnt(1);
                recording{reCnt,8} = tarPnt(2);
                reCnt=reCnt+1;
            else
                sRGB_xy_Exp(i,:) = [tarPnt(1) tarPnt(2)];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %circle(x(3),y(3), sqrt(r_sq)), hold on,
            %plot(x,y,'ro','markersize',7), hold on,
            %plot(x1,y1,'k--');
            %hold on;
            
            plot(x(3),y(3),'bo','markersize',4,'LineWidth',2), hold on,
            plot(tarPnt(1),tarPnt(2),'ro','markersize',4,'LineWidth',2), hold on,
            
            plot(x2,y2,'r--','LineWidth',2);
            hold on;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
        end
        
        if inB(i)==1,
            Bvec = [(ADx(3)-D65x) (ADy(3)-D65y)];
            Bvec = Bvec/norm(Bvec);
            theta= acos(Ovec*Bvec');
            RotMat = [cos(theta) -sin(theta);sin(theta) cos(theta)];
            
            D65xy = RotMat*[D65x;D65y];
            OrignX = D65xy(1);
            OrignY = D65xy(2);
            
            ADxy = RotMat*[ADx;ADy];
            Ax = ADxy(1,:);
            Ay = ADxy(2,:);
            
            sRGB_B1 = RotMat*sRGB_xy(i,:)';
            sRGB_B1=sRGB_B1';
            
            
            
            Pnt = [sRGB_B1(1) sRGB_B1(2)];
            x = [Ax(3) OrignX Pnt(1)];
            y = [Ay(3) OrignY Pnt(2)];
            
            tmpx = [Ax(3) OrignX Pnt(1)];
            tmpy = [Ay(3) OrignY Pnt(2)];
            
            p = polyfit(x,y,2);
            
            x1 = linspace(min(x),max(x),xnum);
            y1 =  p(1)*x1.^2+p(2)*x1+p(3);
            
            circle = @(x, y, r) rectangle('Position', [x-r, y-r, 2*r, 2*r], ...
                'Curvature', [1 1]);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if charmode ==1, % curve expansion 데이터로 보고 싶을 때는 0 아니면 먼셀데이터로 보고 싶으면 1
                A = [sRGB_xy(i,1) sRGB_xy(i,2)];
                B = [target(i,1) target(i,2)];
                r_sq = pdist2(A,B,'euclidean');
                if r_sq == 0.0,
                    r_sq = 0.0004;
                else
                    r_sq = r_sq^2;
                end
            else
                % r_sq = 0.0050;
                %%%% revision 2017.02.02 : 원점 근처의 sRGB는 고정.
                A = [sRGB_xy(i,1) sRGB_xy(i,2)];
                B = [D65x D65y];
                r_sq = pdist2(A,B,'euclidean');
                
                if r_sq <= B_disMean,
                    r_sq = normpdf(r_sq,B_disMean,sig1)*Bdegree_ratio1;
                else
                    r_sq = normpdf(r_sq,B_disMean,sig2)*Bdegree_ratio2;
                end           
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            syms u v;
            f1 = (u-x(3))^2+(v-y(3))^2-r_sq;
            f2 = p(1)*u^2+p(2)*u+p(3)-v;
            [u,v]=solve(f1,f2);
            
            solx = find(imag(u)==0);
            soly = find(imag(v)==0);
            
            ADxy=[x(1) y(1)];
            Solxy=[double(u(solx)) double(v(soly))];
            
            dist = pdist2(ADxy,Solxy,'euclidean');
            
            [val index]=min(dist);
            
            tarPnt = [double(u(index));double(v(index))];
            
            x2 = linspace(min(x(3),tarPnt(1)),max(x(3),tarPnt(1)),xnum);
            y2 =  p(1)*x2.^2+p(2)*x2+p(3);
            
            %%%%%%%%%%%%%% inverse rotation %%%%%%%%%%%%
            tarPnt = RotMat'*tarPnt;
            xy = RotMat'*[x;y];
            x = xy(1,:);
            y = xy(2,:);
            
            xy1 = RotMat'*[x1;y1];
            x1 = xy1(1,:);
            y1 = xy1(2,:);          
            
            xy2 = RotMat'*[x2;y2];
            x2 = xy2(1,:);
            y2 = xy2(2,:);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%% Recording %%%%%%%%%%%%%%%%%
            if charmode ==1,
                recording{reCnt,5} = x(3);
                recording{reCnt,6} = y(3);
                
                recording{reCnt,7} = tarPnt(1);
                recording{reCnt,8} = tarPnt(2);
                reCnt=reCnt+1;
            else
                sRGB_xy_Exp(i,:) = [tarPnt(1) tarPnt(2)];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %circle(x(3),y(3), sqrt(r_sq)), hold on,
            %plot(x,y,'ro','markersize',7), hold on,
            %plot(x1,y1,'k--');
            %hold on;
            
            plot(x(3),y(3),'ro','markersize',4,'LineWidth',2), hold on,
            plot(tarPnt(1),tarPnt(2),'go','markersize',4,'LineWidth',2), hold on,
            
            plot(x2,y2,'g--','LineWidth',2);
            hold on;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold off;


fprintf('Convert xy to AdobeRGB !!!\n');
if charmode ==1,
    
    sheet = 1;
    xlRange = 'A1';
    
    filename = [filename '_curve' ];
    writefilename = [filename formatfile];
    
    xlswrite(writefilename,recording,sheet,xlRange);
    clearvars  -except filename formatfile;
    clc;  
    
    %%%%%%%%%%%%%%%%%%%%%%%%%Expasion RGB Debugging %%%%%%%%%%%%%%%%%%%%%%
    readfilename = [filename formatfile];
    
    in=xlsread(readfilename,1,'E:H');
    
    D65x=[0.3127]; %D65 White Point
    D65y=[0.3290];
    Sx=[0.6400 0.3000 0.1500]; %sRGB gamut
    Sy=[0.3300 0.6000 0.0600];
    DPx=[0.680 0.265 0.150] %DCI-P3 gamut
    DPy=[0.320 0.690 0.060]
    ADx=[0.6400 0.2100 0.1500]; %Adobe RGB gamut
    ADy=[0.3300 0.7100 0.0600];
    Ux=[0.708 0.170 0.131]%Rec 2020 gamut
    Uy=[0.292 0.797 0.046]
    Px=[0.7347 0.1596 0.0366] %ProPhoto
    Py=[0.2653 0.8404 0.0001]
    
    
    
    sRGB = in(:,[1:2]);
    Ex_RGB = in(:,[3:4]);
    
    origin = [D65x D65y];
    
    [m,n]=size(sRGB);
    
    figure(3),
    cieplot();
    plot(sRGB(:,1),sRGB(:,2),'ko'); %Munsell 5B
    plot(D65x,D65y,'r*','markersize',10); %D65 White Point
    plot(Ex_RGB(:,1),Ex_RGB(:,2),'co');; %5B Linear Expansion
    hold on,
    
    k=convhull(Sx,Sy);
    hold on,
    plot(Sx(k),Sy(k),'r--','LineWidth',2);
    axis equal;
    hold on,
    
    k=convhull(ADx,ADy);
    hold on,
    plot(ADx(k),ADy(k),'g--','LineWidth',2);
    axis equal;
    hold on,
        
    for i=1:m,
        
        Bufx=[sRGB(i,1) Ex_RGB(i,1)];
        Bufy=[sRGB(i,2) Ex_RGB(i,2)];
        line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,
        
    end
    fprintf('sRGB and Extended RGB point plot !! \n');
    hold off;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else

    clearvars  -except inR inG inB imgSize wid sRGB_xy_Exp sRGB_z sRGB_XYZ_Sum target;
    %%%%%%%%%%%%% For extended srgb(x %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('Convert xy into images !!\n');
    for i=1:imgSize,        
        if (inR(i)==1)||(inG(i)==1)||(inB(i)==1),
            
            conxy = sRGB_xy_Exp(i,:)*sRGB_XYZ_Sum(i);
            conxyz = [conxy sRGB_z(i)];
            %conxyz = conxyz./100.0;
            
            
            RGBimg = xyz2Adobergb(conxyz);
            
            r = floor(i/wid)+1;            
            if mod(i,wid) == 0,
                c = wid;
            else
                c = mod(i,wid);
            end
            
            fimg(r,c,1) = RGBimg(1);
            fimg(r,c,2) = RGBimg(2);
            fimg(r,c,3) = RGBimg(3);
        end
            
    end
    fimg = uint8(fimg);
    figure(4),
    imshow(fimg);
    imwrite(fimg,'OutImg.tif');   
end
clear all;
clc;
toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


