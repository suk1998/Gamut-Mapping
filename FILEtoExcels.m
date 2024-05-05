clear all;
close all;
clc;



name={'25B','25BG','25G','25GY','25P','25PB','25R','25RP','25Y','25YR';...
       '5B','5BG','5G','5GY','5P','5PB','5R','5RP','5Y','5YR';...
       '75B','75BG','75G','75GY','75P','75PB','75R','75RP','75Y','75YR';...
       '10B','10BG','10G','10GY','10P','10PB','10R','10RP','10Y','10YR'};


for s=1:10,
im=imread(['Alienskin\Munsell_Chart_Sample_sRGB_' name{2,s} '.tif']);

[hgt wid ch]=size(im);

Reds = im(:,:,1);
Greens = im(:,:,2);
Blues = im(:,:,3);

colorCnts = zeros(256,256,256);

for i=1:hgt,
    for j=1:wid,
        if((Reds(i,j)~=255)||(Greens(i,j)~=255)||(Greens(i,j)~=255)),
            colorCnts(Reds(i,j),Greens(i,j),Blues(i,j))=+1;
        end
    end
end

clear im Reds Greens Blues;


im=imread(['Alienskin\Munsell_Chart_Sample_sRGB_' name{2,s} '_alienskin' '.tif']);

[hgt wid ch]=size(im);

Reds = im(:,:,1);
Greens = im(:,:,2);
Blues = im(:,:,3);

colorCntA = zeros(256,256,256);

for i=1:hgt,
    for j=1:wid,
        if((Reds(i,j)~=255)||(Greens(i,j)~=255)||(Greens(i,j)~=255)),
            colorCntA(Reds(i,j),Greens(i,j),Blues(i,j))=+1;
        end
    end
end






filename = ['Alienskin\Munsell_Chart_sRGB_Alienskin_' name{2,s} '.xlsx' ]

recording{1,1} = 'x';
recording{1,2} = 'y';
recording{1,3} = 'Ax';
recording{1,4} = 'Ay';

cnts=2;
cntA=2;
for r=1:255,
    for g=1:255,
        for b=1:255,
            if colorCnts(r,g,b) ~= 0,
                rgb(1) =r;
                rgb(2) =g;
                rgb(3) =b;
                XYZ=srgb2xyz(rgb);
                
                sx = XYZ(1)/(XYZ(1)+XYZ(2)+XYZ(3));
                sy = XYZ(2)/(XYZ(1)+XYZ(2)+XYZ(3));
                
                recording{cnts,1} = sx;
                recording{cnts,2} = sy;
                cnts = cnts + 1;
            end
            
            if colorCntA(r,g,b) ~= 0,
                rgb(1) =r;
                rgb(2) =g;
                rgb(3) =b;
                XYZ=srgb2xyz(rgb);
                
                Ax = XYZ(1)/(XYZ(1)+XYZ(2)+XYZ(3));
                Ay = XYZ(2)/(XYZ(1)+XYZ(2)+XYZ(3));
                
                recording{cntA,3} = Ax;
                recording{cntA,4} = Ay;
                cntA = cntA + 1;
            end
        end
    end
end



sheet = 1;
xlRange = 'E1';
xlswrite(filename,recording,sheet,xlRange);

keep(name);
clear all;
end
