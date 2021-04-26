cam = webcam(1); %%Comment this out and subscribe to your RGB camera instead


while 1
    Scene = snapshot(cam); 
    % preview(cam);
    [X_Location, Target_Average_X, Target_Average_Y] = Track_Func(Scene);
    Target_Average_X = round(Target_Average_X,1);
    Target_Average_Y = round(Target_Average_Y,1);
    if X_Location ~= 1000
        X = sprintf('%0.2f , %0.2f ',Target_Average_X,Target_Average_Y);
         disp(X)
    end
   
end


function [X_Location, Target_Average_X, Target_Average_Y] = Track_Func(Scene)

    %Returns X Distance from Centre, X & Y Coords (Top Left is 0,0)

    %Read Target
    Target = imread('MarkerData_1999.png'); %Load Reference Image of Target (Needs to be Approx. the same resolution as Camera)
    Target_Image = rgb2gray(Target);

    %Read Scene
    Scene_Image = rgb2gray(Scene);

    %Detect Features
    Target_Points = detectSURFFeatures(Target_Image);
    Scene_Points = detectSURFFeatures(Scene_Image);

    %Extract Points of Interest
    [Target_Features, Target_Points] = extractFeatures(Target_Image, Target_Points);
    [Scene_Features, Scene_Points] = extractFeatures(Scene_Image, Scene_Points);

    %Match Features
    Matched_Pairs = matchFeatures(Target_Features, Scene_Features);

    %Display Matched Features
    Matched_Target_Points = Target_Points(Matched_Pairs(:, 1), :);
    Matched_Scene_Points = Scene_Points(Matched_Pairs(:, 2), :);

    if Matched_Scene_Points.Count > 20          %Only Run if Enough Points Match
        
        %Locate Target in Scene
        [tform, ~, ~, status] = estimateGeometricTransform(Matched_Target_Points, Matched_Scene_Points, 'affine');
        
        %Get Bounding Polygon
        boxPolygon = [1, 1;...                           % top-left
            size(Target_Image, 2), 1;...                 % top-right
            size(Target_Image, 2), size(Target_Image, 1);... % bottom-right
            1, size(Target_Image, 1);...                 % bottom-left
            1, 1];                  % top-left again to close the polygon
        
        %Find Location in Scene
        newBoxPolygon = transformPointsForward(tform, boxPolygon);
        
        Top_Left_X = newBoxPolygon(1, 1);       %Get Point X Coord
        Top_Right_X = newBoxPolygon(2, 1);      %Get Point X Coord
        Bottom_Right_X = newBoxPolygon(3, 1);   %Get Point X Coord
        Bottom_Left_X = newBoxPolygon(4, 1);    %Get Point X Coord
        
        Top_Left_Y = newBoxPolygon(1, 2);       %Get Point Y Coord
        Top_Right_Y = newBoxPolygon(2, 2);      %Get Point Y Coord
        Bottom_Right_Y = newBoxPolygon(3, 2);   %Get Point Y Coord
        Bottom_Left_Y = newBoxPolygon(4, 2);    %Get Point Y Coord
        
        Top_Average_X = ((Top_Right_X - Top_Left_X) / 2) + Top_Left_X;              %Get Top Average X Coord
        Bottom_Average_X = ((Bottom_Right_X - Bottom_Left_X) / 2) + Bottom_Left_X;  %Get Bottom Average X Coord
        
        Right_Average_Y = ((Bottom_Right_Y - Top_Right_Y) / 2) + Top_Right_Y;       %Get Right Average Y Coord
        Left_Average_Y = ((Bottom_Left_Y - Top_Left_Y) / 2) + Top_Left_Y;           %Get Left Average Y Coord
        
        Target_Average_X = abs((Top_Average_X + Bottom_Average_X) / 2);             %Get Target Average X Coord
        Target_Average_Y = abs((Right_Average_Y + Left_Average_Y) / 2);             %Get Target Average Y Coord
        
        Scene_Centre_X = size(Scene, 2) / 2;    %Define Centre of Frame
        
        X_Location = Target_Average_X - Scene_Centre_X;     %Pixel Distance from Centre of Frame Along X Axis
        
        %Show Target in Scene
        imshow(Scene);
        line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'y');
        title('Target Found');
        drawnow;
    
    else
        %Show Scene when no Target Found
        X_Location = 1000;          %Set Return Value to Specified 'Target Not Found' Value
        Target_Average_X = 1000;    %Set Return Value to Specified 'Target Not Found' Value
        Target_Average_Y = 1000;    %Set Return Value to Specified 'Target Not Found' Value
        imshow(Scene);
        title('Target Not Found');
        drawnow;
    end
end
