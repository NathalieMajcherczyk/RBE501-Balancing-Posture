function [ transform ] = dh2mat( theta, d, a, alpha )
%dh2mat Produces homogeneous transformation matrix
%   Takes in DH parameters in order to produce a 4x4 SE(3) transformation
%   matrix.

    transform = [cos(theta) -1*sin(theta)*cos(alpha) sin(theta)*sin(alpha)    a*cos(theta);
                 sin(theta) cos(theta)*cos(alpha)    -1*cos(theta)*sin(alpha) a*sin(theta);
                 0          sin(alpha)               cos(alpha)               d;
                 0          0                        0                        1];
end

