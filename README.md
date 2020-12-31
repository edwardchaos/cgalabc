# cgalabc
Started as a learning exercise about computational geometry.
Computational Geometry Algorithms Library ABC(preschool baby stuff) 

Transitioning from 2d to 3d and became a graphics engine
Computer Graphics Algorithms Library ABC

For visualization of algorithms, olc Pixel game engine is used.
There are some dependencies for that:
* glew: Modern OpenGL interface.
  * sudo apt install libglew-dev
* libpng: Portable Network Graphics library to load .png-files.
  * sudo apt install libpng-dev
* X11: Basic graphics interface for Linux to create a windows.

Current random 2D stuff (not adding more here, 2d is not as cool as 3d):
- Basic point/line predicates (point on line, distance to line, etc.)
- Polygons
- Convex hull
- Convex polygon collision
- Point in polygon

Shadowcasting using some of the stuff above:

![](super_secrets/shadowcast.gif)


Current 3D stuff:
- Camera control
- Perspective transform
- Near clipping
- Screen clipping
- Scan conversion with texture map
- z buffer

![](super_secrets/teapot_textured_zbuffered.gif)

