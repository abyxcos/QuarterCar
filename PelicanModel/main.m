clear; clear classes; clc; close all;

%Robot parameters
global m1 m2 l1 l2 lc1 lc2 I1 I2 g;

g   = -9.8;
l1  = 0.26;
l2  = 0.26;
lc1 = 0.0983;
lc2 = 0.0229;
m1  = 6.5225;
m2  = 2.0458;
I1  = 0.1213;
I2  = 0.0116;

open_system( 'pelican_sim.mdl' );

q0 = [ 0; 0; 0; 0 ];

prompt = { 'Enter desired end-effector x value:', ...
          'Enter desired end-effector y value:' };
name = 'End-effector desired position';
numlines = 1;
defaultanswer = { '0.25', '0.25' };
options.Resize = 'on';
options.WindowStyle = 'normal';
answer = inputdlg( prompt, name, numlines, defaultanswer, options );
xyd = str2double( answer );

sim( 'pelican_sim.mdl' );

for i = 1:size( q, 1 )
    stickPlot( q( i,: ) );
    pause( 0.1 );
end