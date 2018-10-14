
function err = project_error(params,X,xL,cx,cy)

%
% wrap our project function for the purposes of optimization
%  params contains the parameters of the camera we want to 
%  estimate.  X,cx,cy are given.
%

%location of camera center  %not optimized
cam.f = params(1);
cam.c = [cx;cy];
cam.R = buildrotation(params(2),params(3),params(4));
cam.t = params(5:7)';


x = project(X,cam);
err = x-xL;


figure(4);  clf;
plot(x(1,:),x(2,:),'b.'); hold on;
plot(xL(1,:),xL(2,:),'r.');
axis image;
drawnow;

