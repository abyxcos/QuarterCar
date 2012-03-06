function qddot = dynamics( u ) %Simulink requires a single input
%function qddot = robot( q, qdot, tau )

%Robot parameters
global m1 m2 l1 l2 lc1 lc2 I1 I2 g;

%State variables
q1 = u( 1 );
q2 = u( 2 );
q1dot = u( 3 );
q2dot = u( 4 );
tau = u( 5:6 );
q = [ q1; q2 ];
qdot = [ q1dot; q2dot ];
damping = 0.5;

%Inertia matrix
M = [ m1 * lc1^2 + m2*( l1^2 + lc2^2 + 2 * l1 * lc2 * cos( q2 ) ) ...
    + I1 + I2, ...
    m2 * ( lc2^2 + l1 * lc2 * cos( q2 ) ) + I2; ...
    m2 * ( lc2^2 + l1 * lc2 * cos( q2 ) ) + I2, ...
    m2 * lc2^2 + I2 ];

%Coriolis and centripetal terms
C = [ -1 * m2 * l1 * lc2 * sin( q2 ) * q2dot, ...
    ( -1 * m2 * l1 * lc2 * sin( q2 ) ) * ( q1dot + q2dot ); ...
    m2 * l1 * lc2 * sin( q2 ) * q2dot, 0 ];

%Gravity compensation terms
G = [ ( m1 * lc1 + m2 * l1 ) * g * sin( q1 ) + ...
    m2 * lc2 * g * sin( q1 + q2 ); ...
    m2 * lc2 * g * sin( q1 + q2 ) ];

%qddot = inv(M) * ( tau -C*qdot - G );
qddot = M \ ( tau -C*qdot -damping*qdot - G ); %MATLAB suggested change
