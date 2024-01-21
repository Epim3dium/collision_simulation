## Basic collision simulation
 Some basic collision detection only for circles and convex polygons (for polygons using SAT) and resolution that includes:
 * calculating  restitution reaction
 * calculating friction forces
 * rotational reaction  
### Collision detection
Collision detection can be devided into two steps:
* Broad-phase - detecting simple Axis aligned bounding boxes to see all possible collisions
* Narrow-phase - based on results from broad-phase checks if polygons are actually overlapping (using separated axis theorem)
#### TODO:
* quad-tree instead of line-swipe
* diffrenet materials for different parts of polygons
* data-oriented design tesets
#### Controls:
* LClick anti-clockwise to create polygon
* Enter to connect dots and create polygon
* R to reset points
* LClick+hold to move around polygons
