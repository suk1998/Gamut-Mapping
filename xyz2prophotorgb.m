% ===================================================
% *** FUNCTION xyz2srgb Prophoto RGB
% ***
% *** function [RGB] = xyz2srgb(XYZ)
% *** computes 8-bit sRGB from XYZ 
% *** XYZ is n by 3 and in the range 0-1
% *** see also srgb2xyz
function [RGB] = xyz2prophotorgb(XYZ)
if (size(XYZ,2)~=3)
   disp('XYZ must be n by 3'); return;   
end

M = [1.3459 -0.2556 -0.0511; -0.5446 1.5082 0.0205; 0.0000 0.0000 1.0693];

RGB = (M*XYZ')';


RGB(RGB<0) = 0;
RGB(RGB>1) = 1;

DACS = zeros(size(XYZ));
index = (RGB<=0.0031308);
DACS = DACS + (index).*(12.92*RGB);
DACS = DACS + (1-index).*(1.055*RGB.^(1/1.8)-0.055);

RGB=ceil(DACS*255);
RGB(RGB<0) = 0;
RGB(RGB>255) = 255;
end