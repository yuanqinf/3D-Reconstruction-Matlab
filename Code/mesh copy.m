%
% load results of reconstruction
%
load matlab_calib


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% cleaning step 1: remove points outside known bounding box
%

disp('clearning step 1');

% Find bad point(out of Bounding Box) for each dimension
min_1 = -230; max_1 =  370;
b1 = find(X(1,:)<min_1); b2 = find(X(1,:)>max_1);
bad_points = (b1 & b2);

min_2 = -300; max_2 = 300;
b1 = find(X(2,:)<min_2); b2 = find(X(2,:)>max_2);
bp_temp = (b1 & b2);
bad_points = union(bad_points,bp_temp);

min_3 = -250; max_3 = 150;
b1 = find(X(3,:)<min_3); b2 = find(X(3,:)>max_3);
bp_temp = union(b1, b2);
bad_points = union(bad_points,temp);

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% display results
%
figure(1); clf;
h = trisurf(tri,X(1,:),X(2,:),X(3,:));
set(h,'edgecolor','none')
set(gca,'projection','perspective')
axis image; axis vis3d;

% rotate the view around so we see from
% the front  (can also do this with the mouse in the gui)
camorbit(45,0);
camorbit(0,-120);
camroll(-8);




%
% MATLAB has other interesting options to control the
% rendering of the mesh... see e.g.
%
%  lighting flat;
%  shading interp;
%  material shiny;
%  camlight headlight;
%
%
