function quaterniondemo( time, sap, DCMin, interval )
% function quaterniondemo( time, sap, DCMin, interval )
% quaternion demo, compare numerical and analytical rotational motion
% solutions
% Inputs:
%  time(nt)     time series array (s)
%  sap(3)       [spin rate (Hz), precession cone half angle (rad),
%               precession rate (Hz)]
%  DCMin(3,3) [OPTIONAL] initial Direction Cosine Matrix; default = eye(3)
%  interval   [OPTIONAL] plot update pause interval (s); default = 0.1
% Outputs:
%  2 figures ploting axis orientations for the time series from matlab's
%  ode45 (Ordinary Differential Equation) numerical solution using the
%  quaternion class and PropagateEulerEq method, and the analytical
%  solution for axisymmetric objects computed in the spinprecess function.
% Author:
%  Mark Tincknell, 4 January 2011

if (nargin < 3) || isempty(DCMin)
    DCMin   = eye(3);
end
if (nargin < 4) || isempty(interval)
    interval    = 0.1;
end
ws      = 2 * pi * sap(1);
theta   = sap(2);
wp      = 2 * pi * sap(3);
costh   = cos( theta );
if (abs( costh ) < eps) || (ws == 0)
    w0  = [ 0; -wp; 0 ];
    I   = [ 1; 1; 1 ];
else
    mu  = 1 + ws /( wp * costh );
    w0  = [ ws /( 1 - 1/mu ); -ws * tan( theta )/( mu - 1 ); 0 ];
    I   = [ 1; mu; mu ];
end
q0      = quaternion.angleaxis( theta, [0;0;1] );
[q1 w1] = PropagateEulerEq( q0, w0, I, time );
q1      = quaternion.rotationmatrix( DCMin ) .* q1;
PlotRotation( q1, interval );
title( 'quaternion' );

DCM     = spinprecess( time, sap, DCMin );
plotRotation( DCM, interval );
title( 'spinprecess' );

end % quaterniondemo

function plotRotation( DCM, interval )
% function plotRotation( DCM, interval )
% Plot rotation axes for a series of Direction Cosine Matrices, adapted
% from the quaternion PlotRotation method
if ~exist( 'interval', 'var' )
    interval    = 0.1;
end
nel = size( DCM, 3 );
or  = zeros(1,3);
ax  = eye(3);
alx = zeros( nel, 3, 3 );
figure;
for iel = 1 : nel
%     plot3( [ or; ax(:,1).' ], [ or ; ax(:,2).' ], [ or; ax(:,3).' ], ':' );
    plot3( [ or; ax(1,:) ], [ or ; ax(2,:) ], [ or; ax(3,:) ], ':' );
    hold on
    set( gca, 'Xlim', [-1 1], 'Ylim', [-1 1], 'Zlim', [-1 1] );
    xlabel( 'x' );
    ylabel( 'y' );
    zlabel( 'z' );
    grid on
    nax = DCM(:,:,iel);
    alx(iel,:,:)    = nax; 
%     plot3( [ or; nax(:,1).' ], [ or ; nax(:,2).' ], [ or; nax(:,3).' ], '-', 'LineWidth', 2 );
    plot3( [ or; nax(1,:) ], [ or ; nax(2,:) ], [ or; nax(3,:) ], '-', 'LineWidth', 2 );
%     plot3( alx(1:iel,:,1), alx(1:iel,:,2), alx(1:iel,:,3), '*' );
    plot3( squeeze(alx(1:iel,1,:)), squeeze(alx(1:iel,2,:)), squeeze(alx(1:iel,3,:)), '*' );
    if interval
        pause( interval );
    end
    hold off
end % for
end % plotRotation

function [DCMout, Omega] = spinprecess( time, sap, DCMin )
% function [DCMout, Omega] = spinprecess( time, sap, DCMin )
% Direction Cosine (Rotation) Matrices for a spinning, precessing
% axisymmetric object.  "x" ([1;0;0]) is the principal body axis.
% Inputs:
%  time(nt)     times (s)
%  sap(3)       [spin rate (Hz), prec. cone angle (rad), prec. rate (Hz)]
%  DCMin(3,3) [OPTIONAL] initial Direction Cosine Matrix; default = eye(3)
% Outputs:
%  DCMout(3,3,nt)   Direction Cosine Matrices
%  Omega(3,3,nt)    Rotational velocity matrices
% Application:
%  x_rot = DCMout * x (where x is a 3 element column position vector)
%  v_rot = Omega * x + v (where v is the translational velocity vector)
% Calls:
%  rodrigues(axis,theta)    Rotation by theta about axis
% Author:
%  Mark Tincknell, MIT LL, 23 Nov. 2004
%                  revised 14 Dec. 2009

if (nargin < 3) || isempty(DCMin)
    DCMin   = eye(3);
end
nt      = length( time );
Rang    = rodrigues( [0;0;1], sap(2) ); % precession cone angle
omegas  = 2 * pi * sap(1);              % spin
omegap  = 2 * pi * sap(3);              % precession
for it = nt : -1 : 1
    [Rs, dRs]   = rodrigues( [1;0;0], omegas * time(it) );
    [Rp, dRp]   = rodrigues( [1;0;0], omegap * time(it) );
    DCMout(:,:,it)  = DCMin * Rp * Rang * Rs;
    Omega(:,:,it)   = DCMin *( omegap * dRp * Rang * Rs + ...
                               omegas * Rp * Rang * dRs );
end
end % spinprecess

function [R, dR] = rodrigues( u, theta )
% function [R, dR] = rodrigues( u, theta )
% Computes rotation matrix (Direction Cosine Matrix) from Rodrigues'
% rotation formula. See http://mathworld.wolfram.com/RodriguesRotationFormula.html
% Inputs:
%  u(3)     fixed axis of rotation
%  theta    rotation angle in radians
% Outputs:
%  R(3,3)   rotation matrix
%  dR(3,3)  differential of R
% Application:
%  x_rot = R * x (where x is a 3 element column vector)
%  v_rot = omega * dR * x (where omega is d(theta)/dt)
% Authors:
%  Douglas Lanman, Mark Tincknell, MIT LL, 8 August 2003

% Normalize u (fixed-axis of rotation)
u   = u/norm(u);

% Skew-symmetric matrix W
W   = [ 0    -u(3)  u(2);
        u(3)  0    -u(1);
       -u(2)  u(1)  0];
W2  = W^2;

% Trig functions
sth = sin( theta );
cth = cos( theta );

% Rotation matrix R, differential rotation matrix dR
R   = eye(3) + sth * W + (1 - cth) * W2;
dR  = cth * W + sth * W2;

end % rodrigues