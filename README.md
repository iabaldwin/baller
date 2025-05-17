```
    ___       ___       ___       ___       ___       ___   
   /\  \     /\  \     /\__\     /\__\     /\  \     /\  \  
  /::\  \   /::\  \   /:/  /    /:/  /    /::\  \   /::\  \ 
 /::\:\__\ /::\:\__\ /:/__/    /:/__/    /::\:\__\ /::\:\__\
 \:\::/  / \/\::/  / \:\  \    \:\  \    \:\:\/  / \;:::/  /
  \::/  /    /:/  /   \:\__\    \:\__\    \:\/  /   |:\/__/ 
   \/__/     \/__/     \/__/     \/__/     \/__/     \|__|  
```

# Baller

Baller is a small interactive bundle adjustment playground.  It can
generate synthetic scenes, perturb the data and run a Ceres based
optimisation while visualising every iteration.  Two programs are
provided:

* `simple_scene` &ndash; generates a BAL format problem containing
  cameras, 3D points and their observations.
* `render` &ndash; loads a problem file, adds noise and solves it while
  displaying the ground truth (green) and the current estimate (red),
  the Jacobian matrix and the projected observations.

The solver supports three modes:

* **slam** &ndash; estimate cameras and points together.
* **localization** &ndash; estimate cameras while keeping points fixed.
* **mapping** &ndash; estimate points while keeping cameras fixed.

## Requirements

* [RayLib](https://github.com/raysan5/raylib.git) (811e241f)
* [Ceres](https://github.com/ceres-solver/ceres-solver.git) (766f2ca)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (v.3.0)
* [Sophus](http://eigen.tuxfamily.org/index.php?title=Main_Page) (a0fe89a)

Generate a simple bundle-adjustment problem:

```
$ cd build
$ ./simple_scene --output simple.txt
$ ./render --input simple.txt --mode slam
```

Which should give:

![SFM](images/sfm.gif)

Other modes are:

```
$ ./render --input simple.txt --mode localization
```

![SFM](images/localization.gif)

or:

```
$ ./render --input simple.txt --mode mapping
```

![SFM](images/mapping.gif)

In all cases, the Jacobians are rendered on the left-hand side of the image, and
the reprojected points (per-camera) in the lower-right. Green is the
ground-truth, red is estimated.
