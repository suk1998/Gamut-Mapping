ORx=[0.312715907	0.319869064	0.326266058	0.334816923	0.344250495	0.35366445	0.365022136	0.377509256	0.391199894	0.406150977	0.423930629	0.443219112	0.462134904	0.485771816	0.510353345	0.535201953	0.563032477	0.589975132	0.615644862	0.640074499]
ORy=[0.329001481	0.329022633	0.329041516	0.329066812	0.329094796	0.329122801	0.329156104	0.329193001	0.329233653	0.32927834	0.329330551	0.329387863	0.329443591	0.329513981	0.329586368	0.329660372	0.329742085	0.329822373	0.329898022	0.329970511]
OGx=[0.312715907	0.311971616	0.311370162	0.310565712	0.309764131	0.30910224	0.308223394	0.307378509	0.306648032	0.305844993	0.305045828	0.304199084	0.303572584	0.302861703	0.302158062	0.301674697	0.301131688	0.300637748	0.300310506	0.3]
OGy=[0.329001481	0.344864127	0.357683695	0.374823479	0.391906425	0.406015397	0.424743246	0.442756167	0.458316307	0.475432362	0.492466016	0.510508271	0.523863719	0.539012855	0.554007233	0.564317892	0.575887214	0.586407457	0.593381935	0.6]
OBx=[0.312715907	0.303108261	0.294012595	0.287121615	0.278430118	0.269302122	0.261263557	0.252413267	0.244050069	0.235553932	0.226994054	0.218450773	0.209782094	0.201313262	0.192329436	0.183989043	0.174872845	0.166867418	0.158464455	0.150016622]
OBy=[0.329001481	0.313116815	0.298078604	0.286685883	0.272316217	0.257224276	0.242969782	0.229301538	0.215474607	0.201427706	0.187275303	0.173150193	0.158063528	0.143415209	0.129963019	0.115604642	0.101102489	0.087866086	0.073973264	0.060006649]
PRx=[0.312715907	0.319189335	0.32552986	0.33400429	0.344250495	0.356654589	0.370553279	0.387342962	0.404739045	0.422390497	0.44489225	0.467478812	0.485771816	0.514178409	0.537100419	0.561234473	0.583612845	0.604508485	0.623811427	0.640074499]
PRy=[0.329001481	0.329020674	0.329039331	0.329064646	0.329094796	0.329131572	0.329172649	0.329222432	0.329274012	0.329326024	0.329392501	0.329459495	0.329513981	0.329597856	0.32966516	0.329736853	0.329803056	0.329865305	0.329922213	0.329970511]
PGx=[0.312715907	0.311873911	0.311213818	0.310228278	0.309040612	0.30803079	0.306848282	0.305778219	0.304848165	0.3039452	0.303151803	0.302365967	0.301909548	0.301411188	0.300950636	0.300695234	0.300481847	0.300240079	0.300129644	0.3]
PGy=[0.329001481	0.346945402	0.361013088	0.382019552	0.40732607	0.428853428	0.454052663	0.476858647	0.496680107	0.515917419	0.532835169	0.549571918	0.559306683	0.56992804	0.579740587	0.585186453	0.58973262	0.594877494	0.597236199	0.6]
PBx=[0.312715907	0.303733362	0.294683493	0.287825349	0.276938125	0.264586358	0.254887563	0.244050069	0.234699779	0.225281289	0.215904126	0.206691883	0.198336983	0.19034581	0.182112139	0.174872845	0.16739013	0.161640461	0.155825336	0.150016622]
PBy=[0.329001481	0.314150047	0.299188153	0.287849292	0.269849451	0.249427911	0.233392183	0.215474607	0.200015498	0.18444349	0.16894006	0.153708956	0.139215007	0.125441576	0.113071093	0.101102489	0.088731316	0.079224574	0.069609985	0.060006649]


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
scatter(PRx,PRy,50,'r'); % R photoshop data
scatter(PGx,PGy,50,'g'); % G photoshop data
scatter(PBx,PBy,50,'b'); % B photoshop data


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
 hold on,

[hgt wid]=size(ORx);


for i=1:wid,
 Bufx=[ORx(i) PRx(i)];
 Bufy=[ORy(i) PRy(i)];
 line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,

end

 hold on,

[hgt wid]=size(OGx);


for i=1:wid,
 Bufx=[OGx(i) PGx(i)];
 Bufy=[OGy(i) PGy(i)];
 line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,

end

hold on,

[hgt wid]=size(OBx);


for i=1:wid,
 Bufx=[OBx(i) PBx(i)];
 Bufy=[OBy(i) PBy(i)];
 line(Bufx,Bufy,'Color','k','LineWidth',1),hold on,

end




