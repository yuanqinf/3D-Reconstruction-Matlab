%
% load in calibration data
%

load camParams.mat 

thresh = 0.02;
scandir = 'teapot_u/grab_0_u/';

[L_h,L_h_good] = decode([scandir 'frame_C1_'],0,19,thresh);
[L_v,L_v_good] = decode([scandir 'frame_C1_'],20,39,thresh);
[R_h,R_h_good] = decode([scandir 'frame_C0_'],0,19,thresh);
[R_v,R_v_good] = decode([scandir 'frame_C0_'],20,39,thresh);

%
% visualize the masked out horizontal and vertical
% codes for left and right camera
%
figure(1); clf;
subplot(2,2,1); imagesc(R_h.*R_h_good); axis image; axis off;title('right camera, h coord');
subplot(2,2,2); imagesc(R_v.*R_v_good); axis image; axis off;title('right camera,v coord');
subplot(2,2,3); imagesc(L_h.*L_h_good); axis image; axis off;title('left camera,h coord');  
subplot(2,2,4); imagesc(L_v.*L_v_good); axis image; axis off;title('left camera,v coord');  
colormap jet

%
% combine horizontal and vertical codes
% into a single code and a single mask.
%
Rmask = R_h_good & R_v_good;
R_code = R_h + 1024*R_v;
Lmask = L_h_good & L_v_good;
L_code = L_h + 1024*L_v;

%
% now find those pixels which had matching codes
% and were visible in both the left and right images
%
% only consider good pixels
Rsub = find(Rmask(:));
Lsub = find(Lmask(:));

% find matching pixels 
[matched,iR,iL] = intersect(R_code(Rsub),L_code(Lsub));
indR = Rsub(iR);
indL = Lsub(iL);

% indR,indL now contain the indices of the pixels whose 
% code value matched

% pull out the pixel coordinates of the matched pixels
[h,w] = size(Rmask);
[xx,yy] = meshgrid(1:w,1:h);
xL = []; xR = [];
xR(1,:) = xx(indR);
xR(2,:) = yy(indR);
xL(1,:) = xx(indL);
xL(2,:) = yy(indL);

%
% now triangulate the matching pixels using the calibrated cameras
%
X = triangulate(xL,xR,camL,camR);

% plot 2D overhead view
clf; plot(X(1,:),X(3,:),'.');
axis image; axis vis3d; grid on;
hold on;
plot(camL.t(1),camL.t(3),'ro')
plot(camR.t(1),camR.t(3),'ro')
xlabel('X-axis');
ylabel('Z-axis');


% plot 3D view
clf; plot3(X(1,:),X(2,:),X(3,:),'.');
axis image; axis vis3d; grid on;
hold on;
plot3(camL.t(1),camL.t(2),camL.t(3),'ro')
plot3(camR.t(1),camR.t(2),camR.t(3),'ro')
set(gca,'projection','perspective')
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');


%
% save the results of all our hard work
%
save('matlab_calib.mat','X','xL','xR','camL','camR','Lmask','Rmask');


