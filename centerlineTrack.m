function waypoint_matrix = centerlineTrack(name)
% Frome centerLine2Mat.m from Shahar Gitlab

    data = readmatrix(name, 'NumHeaderLines', 1);
    data(end+1,:) = data(1,:);
  
    % Extract columns based on your header
    x_m = data(:, 1);
    y_m = data(:, 2);
    waypoint_matrix = [x_m'; y_m'];
end