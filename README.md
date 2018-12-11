# zum GeoXL35
> Archive project (2012-2016). Many projects inside.

This repo contains several [research prjects](https://xkunwu.github.io/research/) that I wrote or involved during my PhD study at [MPI](https://www.mpi-inf.mpg.de/departments/computer-graphics/).
Here is a list of peer-reviewed publications that I authored:

-   Approximate 3D Partial Symmetry Detection Using Co-occurrence Analysis, in 3DV 2015.  
-   3D Model Retargeting Using Offset Statistics, in 3DV 2014.  
-   Real-Time Symmetry-Preserving Deformation, in PG 2014.  
-   Symmetry-Aware Template Deformation and Fitting, in CGF 2014.  

This repo also contains many useful tools for:
-   Symmetry group structure abstrction and (partial) symmetry detection.  
-   Symmetric blue noise (Poisson-disk) sampling.  
-   Soft and hard constraint numerical optimization framework:
    -   Symmetry constrained subspace deformation method,
    -   Iterative/GD/CG/Newton solver,
    -   Null-space solver,
    -   Laplacian regularization.  
-   Symmetry based point cloud completion.  
-   [LibGizmo](https://github.com/CedricGuillemet/LibGizmo) based interactive GUI.  
-   [Eigen](http://eigen.tuxfamily.org) or [CLAPACK](https://www.netlib.org/clapack/) or [cuBLAS](https://developer.nvidia.com/cublas) based numerical optimization solvers for real-time interactive editing.  
-   Robust "damping" point-to-line/point-to-plane Iterative Closest Line/Point (ICL/ICP) algorithm for matching line/point feature clusters.  
-   2D/3D abstract lattice structure as the basis for finite element method, grid structure matching, ordered point set.  
-   Half-edge mesh structure.  
-   Discrete differential geometry: Frenet frame, local second-order algebraic surface fitting, principal curvatures (plus mean/Gaussian curvature), Laplacian Eigenvectors, Histogram of Oriented Gradients, etc.
-   Use cases of:
    -   [ANN: Approximate Nearest Neighbor Searching](https://www.cs.umd.edu/~mount/ANN/)
    -   [Fast Earth Mover’s Distances (EMD)](http://www.cs.huji.ac.il/~werman/Papers/ICCV2009.pdf),
    -   [GCoptimization Library for Graph-cut](http://www.csd.uwo.ca/~olga/OldCode.html),
    -   [Numerical Recipes](http://numerical.recipes/),
    -   [OpenGM for discrete factor graph models](http://hciweb2.iwr.uni-heidelberg.de/opengm/),
    -   [Lp Centroidal Voronoi Tesselation (CVT)](http://alice.loria.fr/index.php/publications.html?redirect=0&Paper=LPCVT@2010&Author=levy),
    -   [Super4PCS](http://geometry.cs.ucl.ac.uk/projects/2014/super4PCS/),  
    -   [Symmetry factorization](https://dl.acm.org/citation.cfm?id=2601220).  
-   And probably a few other things that I forgot :expressionless:.

## Prerequisites
Note: this was last tested in 2016 in Windows - it's an archive project now.

To compile this software, download [GeoXL35](http://www.staff.uni-mainz.de/wandm/software.html#geoxl35) first, then put the source code into the same directory structure as this repo.
For example, 'projectsXWu' (along with other projects) should be in:
```
GeoXL35/source/modules/projects/
```
### Build
CMake should help you through all the rest configuration/compilation job.
