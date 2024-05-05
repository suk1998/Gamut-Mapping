% ===================================================
% *** FUNCTION xyz2srgb Adobe RGB
% ***
% *** function [RGB] = xyz2srgb(XYZ)
% *** computes 8-bit sRGB from XYZ 
% *** XYZ is n by 3 and in the range 0-1
% *** see also srgb2xyz
function [RGB] = xyz2Adobergb(XYZ)
if (size(XYZ,2)~=3)
   disp('XYZ must be n by 3'); return;   
end

M = [2.0414 -0.5649 -0.3447; -0.9693 1.8760 0.0415; 0.0134 -0.1184 1.0154];

RGB = (M*XYZ')';


RGB(RGB<0) = 0;
RGB(RGB>1) = 1;

DACS = zeros(size(XYZ));
index = (RGB<=0.0031308);
DACS = DACS + (index).*(12.92*RGB);
DACS = DACS + (1-index).*(1.055*RGB.^(1/2.2)-0.055);

RGB=ceil(DACS*255);
RGB(RGB<0) = 0;
RGB(RGB>255) = 255;
end