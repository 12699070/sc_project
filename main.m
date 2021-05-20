close all
clear all
clc

rosshutdown;
rosinit

[velPub, velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
laserSub = rossubscriber('/scan');
imgSub = rossubscriber("/camera/color/image_raw/compressed");


%% Camera parameters*
camWidth = 640;
camHeight = 480;

%% Gain
gainLinearLow = 0.05;
gainLinearHigh = 0.2;
gainAngularLow = 0.05;
gainAngularHigh = 0.3;

%% Thresholds
transitionDist = 0.7;
stopDist = 0.3;
coarseCenterWidth = 190;
fineCenterWidth = 75;

%% Fixed
left = -1;
right = 1;
forward = 1;
coarseApproachDone = false;
publishRate = 10;




%% Begin runtime
while(1)
    %% Read tag
    imgRecv = receive(imgSub);
    I = readImage(imgRecv);
    [id,loc,detectedFamily] = readAprilTag(I,'tag36h11');
    
    %% Mark corners
    for idx = 1:length(id)
        % Insert markers to indicate the locations
        markerRadius = 8;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        I = insertShape(I,"FilledCircle",markerPosition,"Color","red","Opacity",1);
    end
    
    %% Read laser scan
    scanData = receive(laserSub);
    frontDistance = scanData.Ranges(1);
    if frontDistance < stopDist
        I = insertText(I,[230 240],"TOO CLOSE",'FontSize',30,'TextColor','red');
    end
    
    %% Coarse approach
    if (coarseApproachDone == false)
        try
            %   x
            % START------------transition-------------GOAL
            %
            desiredCenterWidth = coarseCenterWidth;
            [centerLine_left, centerLine_right] = ThresholdGenerator(camWidth,camHeight,desiredCenterWidth);
            I = insertShape(I,"Line",centerLine_left,"LineWidth",2,"color","yellow");
            I = insertShape(I,"Line",centerLine_right,"LineWidth",2,"color","yellow");
            
            if frontDistance > transitionDist
                if loc(1,1) < centerLine_left(1,1)
                    linearSpeed = forward*gainLinearHigh;
                    angularSpeed = right*gainAngularHigh; % steer right
                    
                elseif loc(2,1) > centerLine_right(1,1)
                    linearSpeed = forward*gainLinearHigh;
                    angularSpeed = left*gainAngularHigh; % steer left
                    
                else
                    linearSpeed = forward*gainLinearHigh;
                    angularSpeed = 0;
                end
                
                velMsg.Linear.X = linearSpeed;
                velMsg.Angular.Z = angularSpeed;
                velPub.send(velMsg);
                rate = rateControl(publishRate);
                
                %                       x
                % START------------transition-------------GOAL
                %
            elseif frontDistance < transitionDist && frontDistance > stopDist
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = 0;
                velPub.send(velMsg);
                rate = rateControl(publishRate);
                %TODO: debounce
                coarseApproachDone = true; % trigger finer approach
                
            elseif frontDistance < stopDist
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = 0;
                velPub.send(velMsg);
                rate = rateControl(publishRate);
            end
            
        catch ME
            velMsg.Linear.X = 0;
            velMsg.Linear.Y = 0;
            velMsg.Angular.Z = 0;
            velPub.send(velMsg);
            rate = rateControl(publishRate);
        end
    end
    
    
    
    
    
    %% Fine approach
    if coarseApproachDone == true
        try
            %% Generate thresholds
            if frontDistance > transitionDist
                coarseApproachDone = false;
            elseif frontDistance <= transitionDist && frontDistance >= 0.45
                desiredCenterWidth = fineCenterWidth;
            elseif frontDistance < 0.45 && frontDistance > stopDist
                desiredCenterWidth = fineCenterWidth+50; % slightly increase the threshold
            end
            disp(frontDistance)
            [centerLine_left, centerLine_right] = ThresholdGenerator(camWidth,camHeight,desiredCenterWidth);
            I = insertShape(I,"Line",centerLine_left,"LineWidth",2,"color","green");
            I = insertShape(I,"Line",centerLine_right,"LineWidth",2,"color","green");
            
            
            %% Apply gain
            topLeft_x = loc(1,1); %sim(2,1), real(1,1)*
            topRight_x = loc(2,1); %sim (1,1), real(2,1)*
            
            % both inside green
            if topLeft_x >= centerLine_left(1,1) && topLeft_x <= centerLine_right(1,1) &&...
                    topRight_x >= centerLine_left(1,1) && topRight_x <= centerLine_right(1,1)
                linearSpeed = forward*gainLinearLow;
                angularSpeed = 0;
                
                % both outside left
            elseif topLeft_x < centerLine_left(1,1) && topRight_x <= centerLine_left(1,1)
                linearSpeed = 0;
                angularSpeed = right*gainAngularHigh;
                
                % both outside right
            elseif topLeft_x >= centerLine_right(1,1) && topRight_x > centerLine_left(1,1)
                linearSpeed = 0;
                angularSpeed = left*gainAngularHigh;
                
                % one outside left
            elseif topLeft_x < centerLine_left(1,1) && topRight_x > centerLine_left(1,1)
                linearSpeed = forward*gainLinearLow;
                angularSpeed = right*gainAngularLow;
                
                % one outside right
            elseif topLeft_x < centerLine_right(1,1) && topRight_x > centerLine_right(1,1)
                linearSpeed = forward*gainLinearLow;
                angularSpeed = left*gainAngularLow;
            end
            
            velMsg.Linear.X = linearSpeed;
            velMsg.Angular.Z = angularSpeed;
            velPub.send(velMsg);
            rate = rateControl(publishRate);
            
        catch ME
            velMsg.Linear.X = 0;
            velMsg.Linear.Y = 0;
            velMsg.Angular.Z = 0;
            velPub.send(velMsg);
            rate = rateControl(publishRate);
        end
    end
    imshow(I);
end