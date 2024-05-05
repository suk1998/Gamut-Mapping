% X = 18.2181;
% Y = 19.77;
% Z = 27.2164;
% var_X = X / 100;        %Where X = 0 ? 95.047
% var_Y = Y / 100;        %Where Y = 0 ?100.000
% var_Z = Z / 100;        %Where Z = 0 ?108.883
% 
% var_R = var_X *  3.2406 + var_Y * -1.5372 + var_Z * -0.4986;
% var_G = var_X * -0.9689 + var_Y *  1.8758 + var_Z *  0.0415;
% var_B = var_X *  0.0557 + var_Y * -0.2040 + var_Z *  1.0570;
% 
% if ( var_R > 0.0031308 ) var_R = 1.055 * ( var_R ^ ( 1 / 2.4 ) ) - 0.055;
% else                     var_R = 12.92 * var_R;
% end
% if ( var_G > 0.0031308 ) var_G = 1.055 * ( var_G ^ ( 1 / 2.4 ) ) - 0.055;
% else                     var_G = 12.92 * var_G;
% end
% if ( var_B > 0.0031308 ) var_B = 1.055 * ( var_B ^ ( 1 / 2.4 ) ) - 0.055;
% else                     var_B = 12.92 * var_B;
% end
% 
% R = var_R * 255
% G = var_G * 255
% B = var_B * 255


function [R,G,B] = xyz2rgb(X,Y,Z)

var_X = X / 100;        %Where X = 0 ? 95.047
var_Y = Y / 100;        %Where Y = 0 ?100.000
var_Z = Z / 100;        %Where Z = 0 ?108.883

var_R = var_X *  3.2406 + var_Y * -1.5372 + var_Z * -0.4986;
var_G = var_X * -0.9689 + var_Y *  1.8758 + var_Z *  0.0415;
var_B = var_X *  0.0557 + var_Y * -0.2040 + var_Z *  1.0570;

if ( var_R > 0.0031308 ) var_R = 1.055 * ( var_R ^ ( 1 / 2.4 ) ) - 0.055;
else                     var_R = 12.92 * var_R;
end
if ( var_G > 0.0031308 ) var_G = 1.055 * ( var_G ^ ( 1 / 2.4 ) ) - 0.055;
else                     var_G = 12.92 * var_G;
end
if ( var_B > 0.0031308 ) var_B = 1.055 * ( var_B ^ ( 1 / 2.4 ) ) - 0.055;
else                     var_B = 12.92 * var_B;
end

R = var_R * 255
G = var_G * 255
B = var_B * 255

end