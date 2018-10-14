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






%
% load results of reconstruction
%
load matlab_calib

xmin = 0;
xmax = 250;
ymin = -50;
ymax = 200;
zmin = -275;
zmax = -175;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% cleaning step 1: remove points outside known bounding box
%
disp('clearning step 1');

% Find bad point(out of Bounding Box) for each dimension
b1 = find(X(1,:)<xmin); b2 = find(X(1,:)>xmax);
bad_points = union(b1,b2);

b1 = find(X(2,:)<ymin); b2 = find(X(2,:)>ymax);
bp_temp = union(b1,b2);
bad_points = union(bad_points,bp_temp);

b1 = find(X(3,:)<zmin); b2 = find(X(3,:)>zmax);
bp_temp = union(b1, b2);
bad_points = union(bad_points,bp_temp);

%
% drop bad points from both 2D and 3D list
%
disp('drop bad points from both 2D and 3D list');
X(:,bad_points) =[];
xL(:,bad_points) =[];
xR(:,bad_points) = [];

% re-run delaunay triangulation
tri = delaunay(xL(1,:),xL(2,:));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% cleaning step 2: remove triangles which have long edges
%

disp('clearning step 2');
TRITHRESH = 10;   %10mm

removed_points =[];

tris = size(tri,1);
err = zeros(tris,1);
for i = 1:tris
  d1 = sum((X(:,tri(i,1)) - X(:,tri(i,2))).^2);
  d2 = sum((X(:,tri(i,1)) - X(:,tri(i,3))).^2);
  d3 = sum((X(:,tri(i,2)) - X(:,tri(i,3))).^2);
  err(i) = max([d1 d2 d3]).^0.5;
  if d1>TRITHRESH || d2>TRITHRESH||d3>TRITHRESH  %check if the distance exceed the threshold
    removed_points = union(removed_points, tri(i,:));
  end
end

subt = find(err<10);

% remove bad triangles
tri = tri(subt,:);
disp('remove bad triangles');

%
% remove unreferenced points which don't appear in any triangle
%

disp('remove unreferenced points');

removed_points_exclude = ismember(removed_points, tri);
removed_points_exclude = ~removed_points_exclude;
removed_points = removed_points(removed_points_exclude);
removed_points = sort(removed_points);
for i = 1:size(tri,1)
    for z = 1:3
        p11 = tri(i,z);
        count =0;
        for j = 1:size(removed_points,1)
            if p11>removed_points(j)
                count = count + 1;
            else
                break;
            end
        end
        tri(i,z) = p11-count;
    end
end

% delete unreferenced points
X(:,removed_points) = [];
xL(:,removed_points) = [];
xR(:,removed_points) = [];

% fix norm and mesh smoothing
temp = tri;
tri(:,2) = temp(:,3);
tri(:,3) = temp(:,2);

X = nbr_smooth(tri,X,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% display results
%
figure(1); clf;
xColor = 0.7*ones(size(X,2),3);
%set(gcf,'renderer','opengl')
%h = trimesh(tri,X(1,:),X(2,:),X(3,:),'edgecolor','k','facecolor','w');
h = trimesh(tri,X(1,:),X(2,:),X(3,:),'facevertexcdata',xColor,'edgecolor','interp','facecolor','interp');
axis image; axis vis3d;
set(gca,'projection','perspective')
set(gca,'CameraPosition',10*camL.t);
set(gca,'CameraUpVector',[0 -1 0]);
set(gca,'CameraViewAngle',8);
lighting phong;
shading interp;
camlight headlight;  


figure(1); clf;
h = trisurf(tri,X(1,:),X(2,:),X(3,:));
set(h,'edgecolor','none')
set(gca,'projection','perspective')
axis image; axis vis3d;
camorbit(45,0);
camorbit(0,-120);
camroll(-8);

lighting flat;
shading interp;
material dull;
camlight headlight;i()

mesh_2_ply(X,xColor',tri,'/Users/Aaron_van/Desktop/project_code/new_test_grab_0_u.ply');
