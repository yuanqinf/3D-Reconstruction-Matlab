
function [cam,x,X] = calibrate(imfile,cam);

%
% calibrate only the extrinsic parameters from a single checkerboard pattern
% this code assumes that the focal length of cam is already set.
%

%
% first load in the calibration images
%
I = im2double(rgb2gray(imread(imfile)));

% get the grid corner points
x = mapgrid(I,10,8);

% true 3D cooridnates  (for a planar pattern)
[yy,xx] = meshgrid(linspace(0,19.55,8),linspace(0,25.2,10));
zz = zeros(size(xx(:)));
X = [xx(:) yy(:) zz(:)]';  

% now calibrate the camera
thx = pi;
thy = 0;
thz = 0;
tx = -30;
ty = -30;
tz = 30;

pinit = [thx,thy,thz,tx,ty,tz];

%we can specify lower and upper bounds, this can be
% useful in getting the optimization to converge to
% a reasonable solution
ub = [inf inf inf    inf inf inf];
lb = [-inf -inf -inf -inf -inf 0];
opts = optimset('maxfunevals',100000,'maxiter',5000);
pfinal = lsqnonlin( @(params) project_error_ext(params,X,x,cam),pinit,lb,ub,opts);

thx = pfinal(1);
thy = pfinal(2);
thz = pfinal(3);
tx = pfinal(4);
ty = pfinal(5);
tz = pfinal(6);

fprintf('final rot = %2.2f %2.2f %2.2f\n',thx,thy,thz);

cam.t= [tx;ty;tz];
cam.R = buildrotation(thx,thy,thz);
xest = project(X,cam);

figure(1); clf;
imagesc(I);
hold on;
plot(x(1,:),x(2,:),'b.')
plot(xest(1,:),xest(2,:),'r.')
axis image;

