function quaterniondemo2
% function quaterniondemo2
% quaternion demo 2, Reentry Vehicle tip off on separation and spin-up
mass    = 500;  % kg
radius  = 0.5;  % m
len     = 3;    % m
Iaxial  = mass *  0.3  * radius^2;  % kg * m^2, solid cone axial moment
Ilat    = mass *( 0.15 * radius^2 + 0.0375 * len^2 );   % lateral moment
I       = [ Iaxial; Ilat; Ilat ];   % principal moments of inertia
w0      = [ 0; 0.1; 0 ];    % rad/s, tip off angular velocity about y axis
q0      = quaternion( 1 );  % initial orientation (x axis is RV symmetry axis)
t       = 0 : 0.05 : 5;     % s, sample times

[q1, w1, t1] = PropagateEulerEq( q0, w0, I, t, @torque );

figure;
subplot( 3, 1, 1 );
plot( t1, w1(1,:), 'b' );
title( 'Body x angular velocity' );
subplot( 3, 1, 2 );
plot( t1, w1(2,:), 'g' );
title( 'Body y angular velocity' );
subplot( 3, 1, 3 );
plot( t1, w1(3,:), 'r' );
title( 'Body z angular velocity' );

PlotRotation( q1, 0.25 );   % 0.25 s between figure updates
end

function tau = torque( t, y )
% function tau = torque( t, y )
% spin-up motors supply constant torque about x axis from 1 to 3 seconds
    if (1 <= t) && (t <= 3)
        tau = [ 235; 0; 0 ];    % kg * m^2 / s^2 about x axis
    else
        tau = [ 0; 0; 0 ];
    end
end