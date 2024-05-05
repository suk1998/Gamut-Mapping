% basic illustration of Gamut for respresentation

D65x=[0.3127] %D65 White Point
D65y=[0.3290]
%Tx=[0.570 0.3 0.19]
%Ty=[0.320 0.530 0.130]
Sx=[0.6400 0.3000 0.1500] %sRGB gamut
Sy=[0.3300 0.6000 0.0600]
%x=[0.4316 0.4197 0.2760 0.3703 0.2999 0.2848 0.5295 0.2305 0.5012 0.3319 0.3984 0.4957 0.2018 0.3253 0.5686 0.4697 0.4159 0.2131]
%y=[0.3777 0.3744 0.3016 0.4499 0.2856 0.3911 0.4055 0.2106 0.3273 0.2482 0.5008 0.4427 0.1692 0.5032 0.3303 0.4734 0.2688 0.3023]

%DPx=[0.680 0.265 0.150] %DCI-P3 gamut
%DPy=[0.320 0.690 0.060]
ADx=[0.6400 0.2100 0.1500] %Adobe RGB gamut
ADy=[0.3300 0.7100 0.0600]
%Ux=[0.708 0.170 0.131]%Rec 2020 gamut
%Uy=[0.292 0.797 0.046]

cieplot(); % 없애고 쓸것

 scatter(D65x,D65y,50,'*','r'); %D65 White Point
 %scatter(Sx,Sy,20,'k','filled'); 
 %scatter(DPx,DPy,20,'k','filled'); 
 %scatter(ADx,ADy,20,'k','filled');
 %scatter(Ux,Uy,20,'k','filled'); 
 %scatter(x,y,50,'k','filled'); 
 
 %hold on,
 %k=convhull(Tx,Ty);
 %hold on,
 %plot(Tx(k),Ty(k),'g-o','LineWidth',2,'markersize',4)

 hold on,
 k=convhull(Sx,Sy);
 hold on,
 plot(Sx(k),Sy(k),'r-o','LineWidth',2,'markersize',4)

 %hold on,
 %k=convhull(DPx,DPy);
 %hold on,
 %plot(DPx(k),DPy(k),'m-o','LineWidth',2,'markersize',4)
 
 hold on,
 k=convhull(ADx,ADy);
 hold on 
 plot(ADx(k), ADy(k),'b-o','LineWidth',2,'markersize',4)
 
 %hold on,
 %k=convhull(Ux,Uy);
 %hold on,
 %plot(Ux(k),Uy(k),'b-o','LineWidth',2,'markersize',4)
 

 