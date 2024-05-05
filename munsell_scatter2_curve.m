ORx=[0.3127	0.329926316	0.347152632	0.364378947	0.381605263	0.398831579	0.416057895	0.433284211	0.450510526	0.467736842	0.484963158	0.502189474	0.519415789	0.536642105	0.553868421	0.571094737	0.588321053	0.605547368	0.622773684	0.64]
ORy=[0.328976358	0.341226582	0.352121448	0.361660957	0.369845108	0.376673902	0.382147338	0.386265417	0.389028138	0.390435502	0.390487509	0.389184158	0.386525449	0.382511383	0.37714196	0.370417179	0.36233704	0.352901545	0.342110691	0.32996448]
OGx=[0.3127	0.312031579	0.311363158	0.310694737	0.310026316	0.309357895	0.308689474	0.308021053	0.307352632	0.306684211	0.306015789	0.305347368	0.304678947	0.304010526	0.303342105	0.302673684	0.302005263	0.301336842	0.300668421	0.3]
OGy=[0.3227	0.3175	0.3145	0.3136	0.3149	0.3183	0.3239	0.3317	0.3417	0.3538	0.368	0.3845	0.4031	0.4239	0.4468	0.4719	0.4992	0.5286	0.5602	0.594]
OBx=[0.3127	0.304136842	0.295573684	0.287010526	0.278447368	0.269884211	0.261321053	0.252757895	0.244194737	0.235631579	0.227068421	0.218505263	0.209942105	0.201378947	0.192815789	0.184252632	0.175689474	0.167126316	0.158563158	0.15]
OBy=[0.329004729	0.298989194	0.270735722	0.244244314	0.219514971	0.196547694	0.175342478	0.155899326	0.138218239	0.122299215	0.108142256	0.09574736	0.085114528	0.076243761	0.069135057	0.063788418	0.060203843	0.058381331	0.058320884	0.0600225]


D65x=[0.3127] %D65 White Point
D65y=[0.3290]
Sx=[0.6400 0.3000 0.1500] %sRGB gamut
Sy=[0.3300 0.6000 0.0600]
DPx=[0.680 0.265 0.150] %DCI-P3 gamut
DPy=[0.320 0.690 0.060]
ADx=[0.6400 0.2100 0.1500] %Adobe RGB gamut
ADy=[0.3300 0.7100 0.0600]
%Ux=[0.708 0.170 0.131]%Rec 2020 gamut
%Uy=[0.292 0.797 0.046]
%Px=[0.7347 0.1596 0.0366] %ProPhoto
%Py=[0.2653 0.8404 0.0001]

cieplot();

scatter(ORx,ORy,50,'k'); %R original data
scatter(D65x,D65y,30,'*','r'); %D65 White Point
scatter(OGx,OGy,50,'k'); % G original data
scatter(OBx,OBy,50,'k'); % B original data
%scatter(PRx,PRy,50,'r'); % R photoshop data
%scatter(PGx,PGy,50,'g'); % G photoshop data
%scatter(PBx,PBy,50,'b'); % B photoshop data


 hold on,
 k=convhull(Sx,Sy);
 hold on,
 plot(Sx(k),Sy(k),'c--o','LineWidth',2,'markersize',4)

 hold on,
 k=convhull(DPx,DPy);
 hold on,
 plot(DPx(k),DPy(k),'m--o','LineWidth',2,'markersize',4)
% 
% hold on,
% k=convhull(ADx,ADy);
% hold on 
% plot(ADx(k), ADy(k),'g--o','LineWidth',2,'markersize',4)
% 
% hold on,
% k=convhull(Ux,Uy);
% hold on,
% plot(Ux(k),Uy(k),'y--o','LineWidth',2,'markersize',4)
% 
% hold on,
% k=convhull(Px,Py);
% hold on 
% plot(Px(k), Py(k),'c--o','LineWidth',2,'markersize',4)
% 
% legend('xy diagram', 'CIE PLOT', 'Original Munsell', 'D65 WhitePoint','Munsell Linear Expansion', 'sRGB-Rec 709', 'DCI-P3','AdobeRGB','Rec-2020', 'ProPhoto')
% 
 %hold on,

%[hgt wid]=size(ORx);


%for i=1:wid,
% Bufx=[ORx(i) PRx(i)];
% Bufy=[ORy(i) PRy(i)];
 %line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,

%end

 %hold on,

%[hgt wid]=size(OGx);


%for i=1:wid,
% Bufx=[OGx(i) PGx(i)];
% Bufy=[OGy(i) PGy(i)];
% line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,

%end

%hold on,

%[hgt wid]=size(OBx);


%for i=1:wid,
% Bufx=[OBx(i) PBx(i)];
% Bufy=[OBy(i) PBy(i)];
% line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,

%end





