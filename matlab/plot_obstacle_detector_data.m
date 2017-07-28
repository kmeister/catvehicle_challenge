
% Author: Kurt Meister
% 
% Copyright (c) 2017, Kurt Meister 
% All rights reserved. 
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are met: 
% 
%  * Redistributions of source code must retain the above copyright notice, 
%    this list of conditions and the following disclaimer. 
%  * Redistributions in binary form must reproduce the above copyright 
%    notice, this list of conditions and the following disclaimer in the 
%    documentation and/or other materials provided with the distribution. 
% 
% THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY 
% EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
% DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY 
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
% OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
% DAMAGE. 

function plot_obstacle_detector_data( bagfile_path, time_offset)
% plots ufiltered, filtered, transformed point cloud data and detections
% polygons at a point in time "time_offset" seconds from the bagfile
% StartTime
%   
bag = rosbag(bagfile_path);
time_interval =  [bag.StartTime+time_offset bag.StartTime+time_offset+1];
input = select(bag, 'Time', time_interval , 'Topic', '/catvehicle/lidar_points');
output = select(bag, 'Time', time_interval , 'Topic', '/meister/transformed_lidar_points');
filtered = select(bag, 'Time', time_interval , 'Topic', '/meister/filtered_lidar_points');
detections = select(bag, 'Time', time_interval , 'Topic', '/detections');


input_msgs = readMessages(input,1);
filtered_msgs = readMessages(filtered,1);
output_msgs = readMessages(output,1);
detections_msgs = readMessages(detections);

i = 1; %the following used to run in a loop in an attempt to animate the plots, but was too slow
subplot(2,1,1);
hold on
for j = 1:size(detections_msgs,1)
   if stamp2double(detections_msgs{j}.Header.Stamp) == stamp2double(output_msgs{i}.Header.Stamp)
        x = [detections_msgs{j}.Polygon.Points.X];
        x = [x x(1)];
        y = [detections_msgs{j}.Polygon.Points.Y];
        y = [y y(1)];
        plot(x, y, 'LineWidth',3, 'Color', 'yellow');
   end

end

scatter([output_msgs{1}.Points.X],[output_msgs{1}.Points.Y], '.r');


title(sprintf('Comparison of filtered and transformed lidar datapoints to detections (Time = %f)', stamp2double(output_msgs{1}.Header.Stamp)));
legend('/meister/transformed\_lidar\_points', '/detections');
hold off
subplot(2,1,2);
scatter([input_msgs{i}.Points.X],[input_msgs{i}.Points.Y], 'ob');
hold on
scatter([filtered_msgs{i}.Points.X],[filtered_msgs{i}.Points.Y], 'xr');
title(sprintf('Comparison of raw lidar points to filtered lidar points (Time = %f)', stamp2double(output_msgs{1}.Header.Stamp)));
legend('/catvehicle/lidar\_points', '/meister/filtered\_lidar\_points');
hold off

end

