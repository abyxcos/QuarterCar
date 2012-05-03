%Simulation - Navfun_Fixedpos
%By Michael Audi, Aaron Fineman
%For RBE/ECE 595
%-------------------------------------------------------------------------%
%Description
%-------------------------------------------------------------------------%
%Leader-Follower navigation simulation with n number of robots
%Simulation based on the concept of the potential field thereom
%Robots keep a fixed distance X from thier respective leader
%The robots will follow a path chosen by the user input on a reference
%plane pre-define in the function

%-------------------------------------------------------------------------%
%Function - navfun_fixedpos(NumRobots)
%-------------------------------------------------------------------------%
%Inputs - integer ranging from 2-n of the number of robots to simulate
%       - a boolean if the simulation should output serial commands
%Outputs - array of the positions followed by the leader robot 
%Description - main function call for running the simulation

function [traj] = navfun_fixedpos(NumRobots, serial)
close all
%-------------------------------------------------------------------------%
%Declarations
%-------------------------------------------------------------------------%
%Robot Navigation Control Gain
K = 300000;
%Gradient Initilization
normGradf = 1000;
%Integration Time Step
dt = 0.01; 
%Destination Tolerance
eps = .00005;
%Workspace Width (meters)
WW = 30;
%Workspace Length (meters)
WL = 70;
%Distance Offset From Leader (centi-meteters)
Offset = 25;

if(serial~=0)
    % Close any leftover serial connections
    try
        out1 = instrfind;
        fclose(out1);
    catch error
    end

    % Create a serial connection with the robot,
    % log in, and start the translation program
    s = serial('/dev/tty.KHIII13914-BluetoothSer');
    fopen(s);
    fprintf(s,'root');
    fprintf(s,'');
    fprintf(s,'./khepera3_test');

    % Create a serial connection with the robot,
    % log in, and start the translation program
    s2 = serial('/dev/tty.KHIII13914-BluetoothSer');
    fopen(s2);
    fprintf(s2,'root');
    fprintf(s2,'');
    fprintf(s2,'./khepera3_test');
end

%-------------------------------------------------------------------------%
%Workspace Layout
%-------------------------------------------------------------------------%
%Hallway
rectangle('Position',[0,0,WW,WL]); 
hold on

%Obsticles 
%Number of obstacles = number of rows in obst
%Each row denotes [cx, cy, r] for each 
obst = [ 8  30  4 ];

%Draw Workspace
theta = linspace(0,2*pi,100);
for i = 1:size(obst,1)
    ox = obst(i,3)*cos(theta)+obst(i,1);
    oy = obst(i,3)*sin(theta)+obst(i,2);
    plot(ox, oy, 'k', 'LineWidth', 2);
end
axis([-1 71 -1 71])
set(gca,'Visible','off')

%-------------------------------------------------------------------------%
%PLOT to PNG conversion.
%-------------------------------------------------------------------------%
print('-dpng','-r100','Workspace')
%Saves the files as a 100dpi PNG
close all

OccupancyInv = imread('Workspace.png');
OccupancyInv = OccupancyInv(:,:,1);
OccupancyInv = double(OccupancyInv)/255;
%The image is represented in "grayscale"
  
figure(2)
image(255*OccupancyInv); colormap gray
Occupancy = 1-OccupancyInv;
 
%-------------------------------------------------------------------------%
%Gaussian Filter
%-------------------------------------------------------------------------%
%A gaussian filter is applied to the image blurring the edges. 
%This filter is then converged with the original matrix 
F = fspecial('gaussian', 70, 10);
BrightnessHeight = 20;

%Brightness is a weight that "heightens" the obstacles in three-space
GaussBlurGrad = imfilter(Occupancy,F,'replicate')*BrightnessHeight;
imshow(GaussBlurGrad); colormap jet

%image displayed
hold on
 
%-------------------------------------------------------------------------%
%Get User Input
%-------------------------------------------------------------------------%

%start = ginput(1)
start = [200 450]
%plot start position
plot(start(1), start(2), 'go','markersize',10,'linewidth',2);

% Obtain a goal location
goal = ginput(1)
 
%plot goal position
plot(goal(1), goal(2), 'ro','markersize',10,'linewidth',2);
pause(.2)
hold on

%-------------------------------------------------------------------------%
%Create Gradient Matrix
%-------------------------------------------------------------------------%
% A matrix of zeros to contain the gradient or paraboloid down to the goal.
[n,m] = size(GaussBlurGrad);
GoalGrad = zeros(n,m);

for i=1:n
    for j=1:m
        current = [j,i];
        % GoalGrad is filled in each location with the distance^2 between
        % said location and the goal location.
        GoalGrad(i,j)=(norm(current-goal))^2;
    end
end

% maximum = max(max(GoalGrad)')
% Matrix divided by its max to bring it down to values between 0 and 1
GoalGrad = GoalGrad./(max(max(GoalGrad)'));
imshow(GoalGrad); colormap jet

% Obstacle repulsive function added to goal attractive function
FullGrad = GoalGrad+GaussBlurGrad;
imshow(FullGrad); colormap jet
hold on

% Gradient of function matrix found at each point in matrix. X values
% stored in one matrix while Y values stored in another.
[GradX,GradY] = gradient(FullGrad);

%-------------------------------------------------------------------------%
%Initialize Robot Positions
%-------------------------------------------------------------------------%
%Leader Robot Navigation Structure
robot = struct('x', start(1), 'y', start(2), 'theta', 0);

%Initial X & Y start positions
robot.x = start(1);
robot.y = start(2);

%Create an array of cells for each robots trajectory
RobotNum = cell(1,NumRobots);
%Create an array of cells for each robots distance
RobotDist = cell(1,NumRobots);
%Create an array of cells for each robots local position
loc = cell(1,NumRobots);
%Initialize the trajectory matrix
traj = [robot.x,robot.y,robot.theta];
%Initialize the distance offset
dist = [0,0];

%Initialize cell values
for i=1:NumRobots
    RobotNum{i} = traj;
    RobotDist{i} = dist;
    loc{i} = 1;
end

%Loop Indicies 
%its is the master position index
its = 1;
%n is an index for the following robots
n = 2;

while (normGradf > eps)&&(its<1000)
    pause(.001)

    %gradf is the positive gradient vector (slope) at the robots current
    %position in the matrix
    gradf = [GradX(round(robot.y),round(robot.x));
             GradY(round(robot.y),round(robot.x))];
   
    %function u is a vector that the robot will follow. -down.
    u = -K*gradf;
    robot = moveRobot(robot, u, dt);
    
    % determine the norm of the gradient of the navigation function
    normGradf = norm(gradf);
    
    %add the new position to the trajectroy matrix
    traj = [traj;robot.x,robot.y,robot.theta];
    
    %update the leader trajectory
    RobotNum{1} = traj;
        
        %Loop for all following robots
        for n=2:NumRobots
            buff1 = RobotNum{n};
            buff2 = RobotNum{1};
            
            %Calculate distance offset for respective leader
            RobotDist{n} = calcdist(buff1(its,:),buff2(its,:));
            
            if RobotDist{n} > Offset*(n-1)
                buff = RobotNum{1};
                %buff2 = RobotNum{n};
                %Concantanate the new location to trajectory
                RobotNum{n} = cat(1,RobotNum{n},buff(loc{n},:));
                if its ~= 1
                    %[rnum,wr,wl] = calcmove(n,buff2(its-1,:),buff2(its,:))
                end
                loc{n} = loc{n}+1;
            else
                buff = RobotNum{n};
                %Robot doesnt change position
                RobotNum{n} = cat(1,RobotNum{n},buff(its,:));
            end         
        end
            %Plot the updated positions on the graph
            for i=2:NumRobots
                buff = RobotNum{i};
                plot(buff(its,1), buff(its,2), 'wo', 'MarkerFaceColor', 'b', 'MarkerSize', 4)
            end 
            
        if its ~= 1
            buff = RobotNum{2};
            [rnum,wr,wl] = calcmove(1,traj(its-1,:),traj(its,:));
            [rnum2,wr2,wl2] = calcmove(1,buff(its-1,:),buff(its,:));
            r = wr*218.72;
            l = wl*218.72;
            r2 = wr2*218.72;
            l2 = wl2*218.72;
            
            if(serial~=0)
                output_string2 = sprintf('setmotspeed %.0f %.0f', l2, r2)
                output_string = sprintf('setmotspeed %.0f %.0f', l, r)
                fprintf(s,output_string);
                fprintf(s2,output_string2);
                %pause(.00001);
            end
            
        end
            
        its = its+1;
end

%Leader sucessfully made it to the goal
if normGradf < eps
    display('GOAL!')
    %Plot final clean trajectory & goal star
    plot(traj(:,1),traj(:,2),'w-','linewidth',2)
    plot(goal(1),goal(2),'cp','markersize',20,'linewidth',2)
else
    %Leader could not make it to the goal    
    display('A path could not be found.')
end

if(serial~=0)
    % Write 0 to the serial and create a clean state
    output_string = sprintf('setmotspeed 0 0')
    fprintf(s,output_string);
    output_string = sprintf('exit')
    fprintf(s,output_string);
    output_string = sprintf('exit')
    fprintf(s,output_string);
    fclose(s)
    delete(s)
    clear s
    
    % Write 0 to the serial and create a clean state
    output_string2 = sprintf('setmotspeed 0 0')
    fprintf(s2,output_string2);
    output_string2 = sprintf('exit')
    fprintf(s2,output_string2);
    output_string2 = sprintf('exit')
    fprintf(s2,output_string2);
    fclose(s2)
    delete(s2)
    clear s2
end

end


%-------------------------------------------------------------------------%
%Function - moveRobot(Robot, v, dt)
%-------------------------------------------------------------------------%
%Inputs - Robot position structure (robot.x, robot.y & robot.theta)
%Outputs - Updated robot position structure
%Description - move robot from position rob.x, rob.y and rob.theta in the 
%direction given the vector v with speed given my ||v| for dt seconds.

function robot = moveRobot(robot, v, dt)
%clear path on plot
plot(robot.x, robot.y, 'wo', 'MarkerFaceColor', 'w', 'MarkerSize', 4)

%calculate new position delta
dq = v*dt;

%update robot position
robot.x = robot.x + dq(1);
robot.y = robot.y + dq(2);
robot.theta = mod(atan2(v(2), v(1)), 2*pi);

%plot new robot position
plot(robot.x, robot.y, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 4)
end

%-------------------------------------------------------------------------%
%Function - calcdist(pos1, pos2)
%-------------------------------------------------------------------------%
%Inputs - pos1 & pos2 are 1x2 arrays of the form [x y]
%Outputs - integer dist output of the distance from pos1 to pos 2
%Description - calculates the scalar distance from pos1 to pos2

function dist = calcdist(pos1, pos2) 
x1 = pos1(1);
y1 = pos1(2);
x2 = pos2(1);
y2 = pos2(2);

dist = sqrt(((x2-x1)^2)+((y2-y1)^2));
end

%-------------------------------------------------------------------------%
%Function - calcmove(pos1, pos2) %where pos1 & pos2 are 1x2 arrays of the form [x y]
%-------------------------------------------------------------------------%

function [Rnum WR WL] = calcmove(num, pos1, pos2)
dt = .014;
r = .0021;
d = .08841;

theta = pos1(3);
xdot = pos2(1) - pos1(1);
ydot = pos2(2) - pos1(2);
thetadot = pos2(3)-pos1(3);

v = xdot/(cos(theta));

w = (thetadot)./dt;
WR = (v+d*w)/r;
WL = (v/r)-((d*w)/(2*r));
Rnum = num;
end
