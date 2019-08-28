# iRobot_Competition
Autonomous Mobile Robots Final Competition with iRobot Create
Replay of Final Competition Results With Localization Results:

![Alt text](Competition_Results/Analysis/testAnimated_fixed.gif?raw=true "Title")

The robot is placed in one of k possible initial positions, with arbitrary orientation. The robot is tasked with:
- Localizing itself
- Determining which of the optional walls are actually in the environment and produce an actual map
- Navigating to as many of the given waypoints as possible

No overhead localization is provided during competition, so the robot must navigate the environment "blind". 

The team's score for the competition is calculated as follows:
- 10 points for each correct waypoint visited
- 20 points for each correct ECwaypoint visited
- -5 points every time the robot indicates incorrectly that it is at a waypoint
- 10 points for each optional wall that is correctly determined (is in the workspace or not)
- -10 points for each optional wall that is incorrectly determined (no points are deducted for optional
walls that have not been determined)
- 10*(time limit (minutes) - actual time(minutes)) if all waypoints and ECwaypoints are visited
before time runs out
- up to 20 points for creative and innovative solution

Our team scored 80 points (90 if you count the first beacon, we did not beep for it because we thought it was obvious)

Algorithm Design:

![Alt text](PDFtoJPG.me-1.jpg?raw=true "Title")
