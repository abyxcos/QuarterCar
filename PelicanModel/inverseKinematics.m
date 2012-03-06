function qd = inverseKinematics( taskSpacePos )

%Robot parameters
global m1 m2 l1 l2 lc1 lc2 I1 I2 g;

xd = taskSpacePos( 1 );
yd = taskSpacePos( 2 );

qd2 = acos( ( xd^2 + yd^2 - l1^2 - l2^2 ) / ( 2 * l1 * l2 ) );
qd1 = atan2( xd, -1 * yd ) - atan2( l2 * sin( qd2 ), ...
        l1 + l2 * cos( qd2 ) );

qd = [ qd1; qd2 ];

end

