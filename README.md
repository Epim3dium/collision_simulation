## Basic collision simulation
Some basic collision detection only for circles and convex polygons and resolution that includes:
 * calculating  restitution reaction
 * calculating friction forces
 * rotational reaction  
Aka the basics of rigidbody collision simulation
### Stress test:
![demo](https://github.com/Epim3dium/collision_simulation/blob/e703438d135e941541d9ebad12b01cad296916f6/assets/flowing.gif)
## Used techniques:
 * narrow/broadphase division
 * SAT to detect polygon overlap
 * point to line distance to calculate circle-polygon collision
 * explicit euler integration
 * collision resolution using restition and friction
 * dormant objects (non moving objects) are skipped for better performance
### Editor usage:
![editor](https://github.com/Epim3dium/collision_simulation/blob/c7dfd0d13d5c251e74b7fa4fdb4511b8d80e7e11/assets/EditorExample.gif)

note that yellow color means, that the object is sleeping and won't be considered during collision detection

note: rendering is done using SFML, GUI is done using imgui
