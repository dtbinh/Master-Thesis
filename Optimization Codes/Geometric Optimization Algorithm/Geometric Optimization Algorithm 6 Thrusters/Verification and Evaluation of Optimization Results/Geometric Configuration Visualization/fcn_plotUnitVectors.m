function [] = fcn_plotUnitVectors(origin, csX, csY, csZ, vectorScale,colorSel)
%Author: Jim West
%Date: 7/11/2010
%Use: Easiest use is fcn_plotUnitVectors(trMatrix), however the vectorScale is not dynamic
%so it will need to be changed in the code if it doesn't suit your data.
%Description: This function plots unitvectors X Y and Z from an origin.
%colorSel can be a string used to format the vector as well.  For example:
%if you have '''k'', ''LineWidth'', 5' as the input, it will set the color to black
% and the thickness of the line to 5

if nargin == 1
    if length(origin) == 4;
        display('transformation matrix recognized');
        vectorScale = 100;
        center = origin(2:4,1);
        csX = origin(2:4,2);
        csY = origin(2:4,3);
        csZ = origin(2:4,4);
        quiver3(center(1), center(2), center(3), csX(1), csX(2), csX(3), vectorScale, 'r', 'LineWidth',8); hold on
        quiver3(center(1), center(2), center(3), csY(1), csY(2), csY(3), vectorScale, 'g', 'LineWidth',8); hold on
        quiver3(center(1), center(2), center(3), csZ(1), csZ(2), csZ(3), vectorScale, 'b', 'LineWidth',8); hold on
        return
    else
        error('If you only input one variable it needs to be a transformation matrix');
    end
end


if nargin < 6
    colorSel = 'b';
elseif nargin< 5
    colorSel = 'b';
    vectorScale = 50;
elseif nargin < 4
    error('Your input arguments are not correct.  Need an origin, X, Y, and Z');
end
hold on
if nargin == 5 %If the color is not selected set each axis to a different color
    quiver3(origin(1), origin(2), origin(3), csX(1), csX(2), csX(3), vectorScale, 'r'); hold on
    quiver3(origin(1), origin(2), origin(3), csY(1), csY(2), csY(3), vectorScale, 'g'); hold on
    quiver3(origin(1), origin(2), origin(3), csZ(1), csZ(2), csZ(3), vectorScale, 'b'); hold on
else
    if length(colorSel) < 4
        quiver3(origin(1), origin(2), origin(3), csX(1), csX(2), csX(3), vectorScale, colorSel); hold on
        quiver3(origin(1), origin(2), origin(3), csY(1), csY(2), csY(3), vectorScale, colorSel); hold on
        quiver3(origin(1), origin(2), origin(3), csZ(1), csZ(2), csZ(3), vectorScale, colorSel); hold on
    else
        q1 = ['quiver3(origin(1), origin(2), origin(3), csX(1), csX(2), csX(3), vectorScale, ', colorSel, ');']; eval(q1); hold on
        q2 = ['quiver3(origin(1), origin(2), origin(3), csY(1), csY(2), csY(3), vectorScale, ', colorSel, ');']; eval(q2); hold on
        q3 = ['quiver3(origin(1), origin(2), origin(3), csZ(1), csZ(2), csZ(3), vectorScale, ', colorSel, ');']; eval(q3); hold on
    end
end

