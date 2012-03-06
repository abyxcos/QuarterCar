function stickPlot( q )
%stickPlot function with help from Thomas Carlone tomcarlone@wpi.edu

%Robot parameters
global m1 m2 l1 l2 lc1 lc2 I1 I2 g;

x(1,1) = 0;
y(1,1) = 0;

%Link 1
y(2,1) = -l1 * cos(q(1)); 
x(2,1) = l1 * sin(q(1)); 

%Link 2
temp = forwardKinematics( q );
x( 3, 1 ) = temp( 1 );
y( 3, 1 ) = temp( 2 );


plot( [ 0; 0; 0 ], [ 0; -0.6; 0 ], '-b', 'LineWidth', 6 )
hold on
plot(x,y,'-ro','LineWidth',4,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'MarkerSize',10)
hold off
grid on
axis( [ -.60 .60 -.60 .60 ] )
xlabel( 'x' )
ylabel( 'y' )
title( 'Pelican Arm' )

end

