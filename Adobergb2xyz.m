% ===================================================
% *** FUNCTION srgb2xyz Adobe RGB
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

M = [0.5767 0.1855 0.1882; 0.2974 0.6273 0.0753; 0.0270 0.0707 0.9911];

DACS=RGB/255;
RGB = zeros(size(RGB));

index = (DACS<=0.04045);
RGB = RGB + (index).*(DACS/12.92);
RGB = RGB + (1-index).*((DACS+0.055)/1.055).^2.2;

XYZ = (M*RGB')';

end