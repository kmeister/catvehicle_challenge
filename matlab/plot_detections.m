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
function plot_detections( bagfile, time )
%plots /detection polygons from a bagfile with a specific timestamp
bag = rosbag(bagfile);
selection = select(bag, 'Time', [bag.StartTime bag.EndTime] , 'Topic', '/detections');
msgs = readMessages(selection, [1:selection.NumMessages]);

for j = 1:size(msgs)
    t = stamp2double(msgs{j}.Header.Stamp);
   if stamp2double(msgs{j}.Header.Stamp) == time
        x = [msgs{j}.Polygon.Points.X];
        x = [x x(1)];
        y = [msgs{j}.Polygon.Points.Y];
        y = [y y(1)];
        plot(x, y, 'LineWidth',3, 'Color', 'yellow');
   end

end

end

