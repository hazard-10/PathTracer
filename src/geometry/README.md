### Mesh Editor

   ![Catmull-Clark Subdivision, cow.js3d](/PathTrcer/renders/editDemo.gif)
Catmull-Clark Subdivision, demo with cow.js3d file

This code base contains my implementation of Scotty3D mesh editor. All editing capabilities are written in C++ using custom Half-edge based model structures supplied by the base code.

#### Feature list
##### Global operations
* Catmull-Clark Subdivision
* Triangulation

##### Half-edge local operations
* Edge flip
* Edge split
* Edge collapse
* Inset vertex on edge
