function tau = controller( u )

%Robot parameters
global m1 m2 l1 l2 lc1 lc2 I1 I2 g;

kp1 = 100.;
kv1 = kp1/8.;
kp2 = 20.;
kv2 = kp2/8.;

currentPosition = u( 1:2 );
currentVelocity = u( 3:4 );
qd = u( 5:6 );
posError = qd - currentPosition;
mass = [ m1; m2 ];
gravity = [ g; g ];
centers = [ lc1; lc2 ];

temp( 1 ) = kp1 * posError( 1 ) - kv1 * currentVelocity( 1 ) + ...
        mass( 1 ) * gravity( 1 ) * centers( 1 ) * sin( qd( 1 ) );
temp( 2 ) = kp2 * posError( 2 ) - kv2 * currentVelocity( 2 ) + ...
        mass( 2 ) * gravity( 2 ) * centers( 2 ) * sin( qd( 2 ) );
temp( 3:4 ) = posError;
tau = temp;

end

