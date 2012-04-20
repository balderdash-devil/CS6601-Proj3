function [Force] = simulator(K, muS , muK)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%define the controller for the arm F =k(deltax)

xprev = 0;%inital estimate of the previous location
loc = 0; %loc is for estimating the current position of the robot.
pos =3; %position where the object is located
mass =2;
g = 9.8;
contact =0; %intially no contact
motion =0;%initioally no motion

%debugging infor and plots
Force = 0:0.1:pos+2;
index =1;

static = muS*mass*g;
kinetic = muK*mass*g;

for xnext = 0.1:0.1:pos+2 %increment deltax through the simulator
deltax = xnext-xprev;
F = K * deltax;

% See if the robot is in contact with the object
%Assume that the object os located at x = 3
loc = xnext;
if(loc >= pos && F< static)
    contact = 1;
end

% If the force on the robot exced the static friction force
if(contact == 1) % the robot is in contact
    F = K * deltax; % the Force on the pressure sensor increases
    if (F >= static)
        motion =1; % object is moving
    else motion =0;% object is not moving
    end
end


%If the object is stationary and the robot is in contact with it
if(motion ==0 && contact ==1)
    %keep the current position of the robot as same
    F = K * deltax;
    Force(index) = F;
end


%If the object is moving the the robot is in contact with it
if(motion ==1 && contact ==1)
    %the force is a difference in static and kinetic friction
    F = static - kinetic + F * deltax ;
    xprev = xnext; %update the next position sinnce it is moving
    Force(index) = F;
end

%If not in contact with object
if(contact ==0)
    F =0; % Force is zero if there is no contact
    xprev = xnext; %update the next position
    Force(index) = F;
end
index = index+1;

end

end

