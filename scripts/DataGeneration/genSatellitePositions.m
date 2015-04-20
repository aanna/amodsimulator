function [ sat_pos ] = genSatellitePositions( center, start_pos, num_sat )
%GENSATELLITELOCATIONS Generates satellite locations around a center
%   returns an array of with the satellite positions
    sat_pos = zeros(num_sat,2);
    angle = 2*pi/num_sat; 
    
    for i=1:num_sat
        rot_angle = (i-1)*angle;
        rot_mat = [cos(rot_angle) -sin(rot_angle);
            sin(rot_angle) cos(rot_angle)];
        sat_pos(i,:) = center(:) + rot_mat*start_pos(:);
    end

end

