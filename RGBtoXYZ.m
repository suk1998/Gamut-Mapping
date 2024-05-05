function XYZ = RGBtoXYZ(R,G,B, Dxx)

% XYZ = rgb2xyz(RGB, 'Dxx')
% Convert an image from RGB color space to XYZ color space
%
% Input image can be either as 3 channels, i.e. a 3D array or individual
% channels, i.e. THREE 1D arrays.
% RGB channels can have common range [0 255] or standard range [0 1].
%
% Dxx = Illuminant spectral
%        - 'D65' 
%        - 'D50'
% Becaus sRGB’s white is defined with illuminant D65, the default is 'D65'.
%
% References:
% http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
%
% Last modified: June 2014
% Copyright Hamid Mahmoudabadi


Dxx = 'D65';
if nargin == 2
  Dxx = G;
  B = double(R(:,:,3));
  G = double(R(:,:,2));
  R = double(R(:,:,1));
else if nargin == 1
  B = double(R(:,:,3));
  G = double(R(:,:,2));
  R = double(R(:,:,1));
end
end

if max(max(R)) > 1.0 || max(max(G)) > 1.0 || max(max(B)) > 1.0
var_R = (double(R) / 255 );        %R from 0 to 255
var_G = (double(G) / 255 );        %G from 0 to 255
var_B = (double(B) / 255 );        %B from 0 to 255
else
var_R = double(R);
var_G = double(G);
var_B = double(B);
end


id_R_more=find(var_R>0.04045);
id_R_less=setdiff(1:numel(R),id_R_more);


id_G_more=find(var_G>0.04045);
id_G_less=setdiff(1:numel(G),id_G_more);

id_B_more=find(var_B>0.04045);
id_B_less=setdiff(1:numel(B),id_B_more);


    var_R(id_R_more) = ( ( var_R(id_R_more) + 0.055 ) ./ 1.055 ) .^ 2.4;
    var_R(id_R_less) = var_R(id_R_less) ./ 12.92;

    var_G(id_G_more) = ( ( var_G(id_G_more) + 0.055 ) ./ 1.055 ) .^ 2.4;
    var_G(id_G_less) = var_G(id_G_less) ./ 12.92;

    var_B(id_B_more) = ( ( var_B(id_B_more) + 0.055 ) ./ 1.055 ) .^ 2.4;
    var_B(id_B_less) = var_B(id_B_less) ./ 12.92;


var_R = var_R .* 100;
var_G = var_G .* 100;
var_B = var_B .* 100;

 [M, N] = size(R);
 s = M * N;
 RGB_var = [reshape(var_R,1,s); reshape(var_G,1,s); reshape(var_B,1,s)];

     switch Dxx
         case 'D65'
        % Standard Observer. = 2? Illuminant spectral = D65
        Mat = [0.4124 0.3576 0.1805;
            0.2126 0.7152 0.0722;
            0.0193 0.1192 0.9505];
        
         case 'D50'
         % Standard Observer. = 2? Illuminant spectral = D50
     Mat = [0.4360747  0.3850649  0.1430804;
            0.2225045  0.7168786  0.0606169;
            0.0139322  0.0971045  0.7141733];
     end
 
xyz=Mat*RGB_var;
 
XYZ(1:M,1:N,1)=reshape(xyz(1,:),[M N]);
XYZ(1:M,1:N,2)=reshape(xyz(2,:),[M N]);
XYZ(1:M,1:N,3)=reshape(xyz(3,:),[M N]);
 