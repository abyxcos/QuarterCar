function taskSpace = forwardKinematics( q )

%Robot parameters
global m1 m2 l1 l2 lc1 lc2 I1 I2 g;

q1 = q( 1 );
q2 = q( 2 );

x = l1 * sin( q1 ) + l2 * sin( q1 + q2 );
y = -1 * l1 * cos( q1 ) - l2 * cos( q1 + q2 );

taskSpace = [ x; y ];

end

