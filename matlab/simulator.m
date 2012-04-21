function [Fprofile] = simulator(K, f, mass, muS , muK)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%define the controller for the arm F =k(deltax)

%% Robot Parameters
deltax = 0.1; % distance moved by the robot in one sampling frequency
% spring contant we aleardy know from the input, K
% Also we know sampling frequency, f;

%% Object Parameters
pos =0.5; %position where the object is located
% mass of the object we know from the input
contact =0; %intially no contact
motion =0;%initioally no motion
vo =0; %object at rest initially
x = 0.5; % displacement for object initially
%% Global & Environmental Parameters
loc = 0; %loc is for estimating the current position of the robot.
%xprev = 0;%inital estimate of the previous location
g = 9.8;

%% Debigging & Plot Parameters
%Force = 0:0.1:pos+2;
index =0;

%% Maximum Static and Kinetic Friction Force on the onject
static = muS*mass*g;
kinetic = muK*mass*g;

%% Let us assume that for every time step, (1/f), we will assume that the robot has moved a distance of deltax

vbot = deltax/(1/f);
T = (pos+3)/vbot;
%%

for t = 0:(1/f): T %for every sampling element 
% First Update the woudbe position if the hand was moving without any
% object
loc = loc+deltax; % this is the only parameter that is truly independent of the position of the object
objloc =0; % for now , believe it.
% Start Backwards!!
%% If the object is moving and there is no contact 
if (contact ==0 && motion ==1)
    % It means that there is no pressure on the sensor
    Pressure = 0;
    %F = K * (loc -actual);
    index = index+1;
    Fprofile(index,1) = awgn(Pressure,20);
    
    % Update the next position of the robot arm
    %actual = actual+deltax;
    % Update the status for the next sampling frequency
    a = (kinetic)/mass;
    x = x + vo*(1/f) - 0.5*a*(1/f)*(1/f);
    v = vo*(1/f) - a*(0.5);
    
    objloc = objloc+x;
    
    if (actual+deltax > objloc)
        contact =1;
        actual = objloc;
    else
        contact =0;
        actual = actual+deltax;
    end
    
    if(v <=0) 
        motion =0;
    else
        motion =1;
    end
    
    continue;
end
%% If there is contact and it is moving
if (contact ==1 && motion ==1)

    F = K*(loc - actual);
    a = (F- kinetic)/mass; 
    Pressure = F - mass*a;
    index = index+1;
    Fprofile(index,1) = awgn(Pressure,20);
    
    %a  = Pressure/mass;
    v = vo+a*(1/f);
    x = x + vo*(1/f)+0.5*a*(1/f)*(1/f);
    %actual = actual+deltax;
    objloc = objloc+x;
    
    if(actual+deltax > objloc)
        contact =1;
        actual = objloc;
        vo = vo+a*(1/f);
    else
        contact =0;
        actual = actual+deltax;
        vo = vo+a*(1/f);
    end
    
    if(v <= 0)
        motion =0;
    end
    
    continue;
end
%% If the robot is in contact with the object and it is not moving, what will pressure be?
if(contact == 1 && motion == 0) % the robot is in contact
    Pressure = K * (loc - actual); % the Force on the pressure sensor increases
    F = K * (loc - actual); %Force on the object also increases
    if (F >= static)
        motion =1; %object is moving
        a = (F -kinetic)/mass;
        vo = a*(1/f);
        x = x+ 0.5*a*(1/f)*(1/f);
        objloc = objloc+x;
        if(objloc > actual+deltax)
            contact =0;
            actual = actual+deltax;
        else
            contact =1;
            actual = objloc;
        end
    else motion =0;%object is not moving
    end
    index = index+1;
    Fprofile(index,1) = awgn(Pressure,20);
    continue;
end
%% If object is not moving and there is no contact
if( contact == 0 && motion == 0)
    F = K * deltax;
    actual = loc;
    Pressure = 0;
    index = index+1;
    Fprofile(index,1) = awgn(Pressure,20);
    
    if(actual >= pos)
        contact = 1;
        motion =0;
    end    
    continue;
end


end

%% Plot the Result Graph
 plot(Fprofile(1:index,1),'DisplayName','force(1:51,1)','YDataSource','force(1:51,1)');figure(gcf)
end

