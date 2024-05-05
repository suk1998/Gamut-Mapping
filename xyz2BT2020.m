% ===================================================
% *** FUNCTION xyz2BT2020
% ***
% *** function [RGB] = xyz2BT2020(XYZ)
% *** computes 8-bit BT2020 from XYZ 
% *** XYZ is n by 3 and in the range 0-1
% *** see also srgb2xyz
function [RGB] = xyz2BT2020(XYZ)
if (size(XYZ,2)~=3)
   disp('XYZ must be n by 3'); return;   
end

M = [1.716651 -0.355671  -0.253366; -0.666684 1.616481 0.015769; 0.017640 -0.042771 0.942103];

RGB = (M*XYZ')';


RGB(RGB<0) = 0;
RGB(RGB>1) = 1;

DACS = zeros(size(XYZ));
index = (RGB<=0.0031308);
DACS = DACS + (index).*(12.92*RGB);
DACS = DACS + (1-index).*(1.055*RGB.^(1/2.4)-0.055);

RGB=ceil(DACS*255);
RGB(RGB<0) = 0;
RGB(RGB>255) = 255;
end