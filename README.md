# GeoXL35
> Archive project (2012-2016). Many projects inside.

This repo contains several [research prjects](https://xkunwu.github.io/research/) that I wrote or involved during my PhD study at [MPI](https://www.mpi-inf.mpg.de/departments/computer-graphics/):

-   Approximate 3D Partial Symmetry Detection Using Co-occurrence Analysis, in 3DV 2015.  
-   3D Model Retargeting Using Offset Statistics, in 3DV 2014.  
-   Real-Time Symmetry-Preserving Deformation, in PG 2014.  
-   Symmetry-Aware Template Deformation and Fitting, in CGF 2014.  

And many useful tools for:
-   Symmetry group structure abstrction and (partial) symmetry detection.  
-   Symmetric blue noise (Poisson-disk) sampling.  
-   Symmetry constrained subspace deformation method, soft/hard constraint numerical optimization framework, null-space solver, and Laplacian regularization.  
-   Symmetry based point cloud completion.  
-   [LibGizmo](https://github.com/CedricGuillemet/LibGizmo) based interactive GUI.  
-   [Eigen](http://eigen.tuxfamily.org) or [CLAPACK](https://www.netlib.org/clapack/) or [cuBLAS](https://developer.nvidia.com/cublas) based numerical optimization solvers for real-time interactive editing.  
-   Robust "damping" point-to-line/point-to-plane Iterative Closest Line/Point (ICL/ICP) for matching line/point feature clusters.  
-   2D/3D abstract lattice structure as the basis for finite element method, grid structure matching, ordered point set.  
-   Half-edge mesh structure.  
-   Use cases of:
    -   [Fast Earth Mover’s Distances (EMD)](http://www.cs.huji.ac.il/~werman/Papers/ICCV2009.pdf),
    -   [GCoptimization Library for Graph-cut](http://www.csd.uwo.ca/~olga/OldCode.html),
    -   [Numerical Recipes](http://numerical.recipes/),
    -   [OpenGM for discrete factor graph models](http://hciweb2.iwr.uni-heidelberg.de/opengm/),
    -   [Lp Centroidal Voronoi Tesselation (CVT)](http://alice.loria.fr/index.php/publications.html?redirect=0&Paper=LPCVT@2010&Author=levy),
    -   [Super4PCS](http://geometry.cs.ucl.ac.uk/projects/2014/super4PCS/).  

## Prerequisites
To compile this software, download [GeoXL35](http://www.staff.uni-mainz.de/wandm/software.html#geoxl35) first, then put the source code into the same directory structure.
For example, 'projectsXWu' (along with other projects) should be in:
```
GeoXL35/source/modules/projects/
```
### Build
Note: this was last tested in 2016 using Windows - it's an archive project now.

CMake should help you through all the configuration/compilation job.
