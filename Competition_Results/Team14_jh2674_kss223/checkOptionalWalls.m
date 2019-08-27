function [realWall] = checkOptionalWalls(bumpData)
% FUNCTION REALWALL = CHECKOPTIONALWALLS(BUMP)
%
%   INPUT:
%           bumpData   1 x 7 bump sensor data [toc, BumpRight, BumpLeft,
%                      DropRight, DropLeft, DropCaster, BumpFront]]
%   OUTPUT:
%           realWall   check the wall existance 
%                      0 = fake wall; 1 = true wall
%   
%   Final Competition
%   Jie Huang, Kowin Shi

BumpRight = bumpData(2);
BumpLeft = bumpData(3);
BumpFront = bumpData(7);

bumped= BumpRight || BumpLeft || BumpFront;   

if bumped == 0
    realWall = 0;
else
    realWall = 1;
    disp(bumped)
    disp("BUMP!!!")
end

end