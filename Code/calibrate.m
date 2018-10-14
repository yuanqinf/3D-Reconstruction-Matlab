function [cam,Pcam,Pworld] = calibrate(imfile);

% function [cam,Pcam,Pworld] = calibrate(imfile);
%
%  This function takes an image file name, loads in the image
%  and uses it for camera calibration.  The user clicks
%  on corner points of a grid (assumed to be a particular calibration
%  object).  These  points along with their true coordinates are used to 
%  optimize the camera parameters with respect to the reprojection
%  error.
%
%  Output: 
%     cam : a data structure describing the recovered camera
%     Pcam : the 2D coordinates of points in the image
%     Pworld : the 3D "ground truth" coordinates of the points in the image
%
I = im2double(rgb2gray(imread(imfile)));

% get the grid corner points
fprintf('click on the corners of each of the three faces\n');
fprintf('always start at the origin point and go around in a circle\n');
fprintf('XY plane  (9x8 squares) -> go along the 9 edge first  (CCW)\n');
XY = mapgrid(I,9,8);  
fprintf('XZ plane  (10x8 squares) -> go along the 10 edge first (CW)\n');
XZ = mapgrid(I,10,8);
fprintf('YZ plane  (8x10 squares) -> go along the 8 edge first (CCW)\n');
YZ = mapgrid(I,8,10);

% true 3D cooridnates (in cm) of the grid corners. this is only correct 
% assuming points were clicked in the order specified above
[yy,xx] = meshgrid( linspace(0,19.55,8), linspace(0,22.05,9));
zz = zeros(size(yy(:)));
XYworld = [xx(:) yy(:) zz(:)]';

[zz,xx] = meshgrid(linspace(0,19.55,8),linspace(0,25.2,10));
yy = zeros(size(xx(:)));
XZworld = [xx(:) yy(:) zz(:)]';

[zz,yy] = meshgrid(linspace(0,25.2,10),linspace(0,19.55,8));
xx = zeros(size(yy(:)));
YZworld = [xx(:) yy(:) zz(:)]';

figure(2); clf;
subplot(1,2,1); imagesc(I); axis image; colormap gray;
subplot(1,2,2);
plot3(XYworld(1,:),XYworld(2,:),XYworld(3,:),'r.'); hold on;
for i = 1:size(XYworld,2)
  h = text(XYworld(1,i),XYworld(2,i),XYworld(3,i),num2str(i));
  set(h,'FontSize',8,'Color','r');
end
plot3(XZworld(1,:),XZworld(2,:),XZworld(3,:),'g.');
for i = 1:size(XZworld,2)
  h = text(XZworld(1,i),XZworld(2,i),XZworld(3,i),num2str(i));
  set(h,'FontSize',8,'Color','g');
end
plot3(YZworld(1,:),YZworld(2,:),YZworld(3,:),'b.');
for i = 1:size(YZworld,2)
  h = text(YZworld(1,i),YZworld(2,i),YZworld(3,i),num2str(i));
  set(h,'FontSize',8,'Color','b');
end
axis vis3d; grid on; set(gca,'Projection','perspective');
camorbit(180,0); %rotate the figure so it looks a bit like the calibration image
xlabel('x'); ylabel('y'); zlabel('z')

% now calibrate the camera
Pcam = [XY XZ YZ];
Pworld = [XYworld XZworld YZworld];

% initial guesses of parameters
cy = size(I,1) / 2;
cx = size(I,2) / 2;
f = 1000;
thx = 3*pi/2; thy = 3*pi/4; thz = 0; %no rotation
tx = 100; ty = 100; tz = 100; %make sure camera is away from origin

% build parameter vector
paramsinit = [f,thx,thy,thz,tx,ty,tz]

% upper and lower bounds on what we think the values should be
ub = [3000 inf inf inf 1000 1000 1000];
lb = [0    -inf -inf -inf   -1000 -1000 -1000];

% setup the optimization routine 
opts = optimset('maxfunevals',100000,'maxiter',10000);

% use an anonymous function to capture fixedparams,X and x.
params_opt = lsqnonlin( @(params) project_error(params,Pworld,Pcam,cx,cy),paramsinit,[],[],opts);

% now unpack params_opt vector back into a cam struct.
cam.f = params_opt(1);
cam.c = [cx;cy];
cam.R = buildrotation(params_opt(2),params_opt(3),params_opt(4));
cam.t = params_opt(5:7)'

% lastly, plot the estimated projected locations of the 3D
%  points on top of the 2D image so we can visualize the 
% reprojection error
Pest = project(Pworld,cam);

figure(3); clf;
imagesc(I); axis image; colormap gray
hold on;
plot(Pcam(1,:),Pcam(2,:),'b.')
plot(Pest(1,:),Pest(2,:),'r.')
hold off;
title('reprojections after optimization');

