function [matrix] = rotationMatrixFromAxisAndAngle(axis, angle)
%ROTATIONMATRIXFROMAXISANDANGLE returns the matrix which performs a rotation
%around a generic axis passing through the origin of a desired angle.
%
%   [matrix] = rotationMatrixFromAxisAndAngle(axis, angle)
%
%
%   input:
%          axis     =  a vector in 3D space indicating the direction of the  
%                      axis of rotation
%          angle    =  the angle of the rotation
%
%   output:
%          matrix   =  3x3 matrix performing a rotation around axis 'axis' of 
%                      an angle 'angle' 


    unitAxis = axis./norm(axis);
    x = unitAxis(1);
    y = unitAxis(2);
    z = unitAxis(3);
    
    cosAngle = cos(angle);
    sinAngle = sin(angle);
    
    matrix = [cosAngle+x*x*(1-cosAngle)    x*y*(1-cosAngle)-z*sinAngle   x*z*(1-cosAngle)+y*sinAngle;
              y*x*(1-cosAngle)+z*sinAngle  cosAngle+y*y*(1-cosAngle)     y*z*(1-cosAngle)-x*sinAngle;
              z*x*(1-cosAngle)-y*sinAngle  z*y*(1-cosAngle)+x*sinAngle   cosAngle+z*z*(1-cosAngle)];
            
end
