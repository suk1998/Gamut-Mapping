% ===================================================
% *** FUNCTION srgb2xyz PtophotoRGB
% ***
% *** function [XYZ] = srgb2xyz(RGB)
% *** computes XYZ from 8-bit RGB 
% *** RGB is n by 3 and in the range 0-255
% *** XYZ is returned in the range 0-1
% *** see also xyz2srgb
function [XYZ] = srgb2xyz(RGB)
if (size(RGB,2)~=3)
   disp('RGB must be n by 3'); return;   
end

XYZ = zeros(size(RGB));

M = [0.7977 0.1352 0.0313; 0.2880 0.7119 0.0001; 0.0000 0.0000 0.8252];

DACS=RGB/255;
RGB = zeros(size(RGB));

index = (DACS<=0.04045);
RGB = RGB + (index).*(DACS/12.92);
RGB = RGB + (1-index).*((DACS+0.055)/1.055).^1.8;

XYZ = (M*RGB')';

end