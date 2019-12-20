# RansacLib

This library provides a template-based, header-only implementation of RANSAC and some of its variants. It is designed to be easily integrated into projects by keeping dependencies small while making it easy to combine it with (minimal) solvers.

Currently, the following RANSAC-variants are implemented
* LO-MSAC as described in *Lebeda, Matas, Chum, Fixing the Locally Optimized RANSAC, BMVC 2012*: RANSAC with local optimization (LO) and a truncated quadratic scoring function (as used by MSAC, described in *Torr, Zisserman,  Robust computation and parametrization of multiple view relations, ICCV 1998*).
* MSAC with a non-linear refinement of each so-far best minimal model. To use MSAC instead of LO-MSAC, set `num_lo_steps_` in `LORansacOptions` to `0`.
* HybridRANSAC as described in *Camposeco, Cohen, Pollefeys, Sattler, Hybrid Camera Pose Estimation, CVPR 2018*: A RANSAC variant that can handle two types of input data (e.g., 2D-3D and 2D-2D matches) and that uses multiple solvers. The implementation uses local optimization and the MSAC cost function.


## Installation
RansacLib is header-only library, so no compilation is required. If you want to use the library in your project, simply include the directory into which you installed the library (such that `RansacLib/ransac.h` can be found from the path).

Besides the actual library, we also provide examples that illustrate how to use RansacLib. These examples can be found in the `examples` subdirectory. They require compilation, which can easily be done via CMake:
```
mkdir build
cd build/
cmake ../
make
```
We currently provide two examples:
* `line_estimation` shows how to implement a solver for 2D line fitting and integrate it into RansacLib.
* `camera_pose_estimation` shows how to implement a solver for absolute pose estimation of calibrated cameras and how to integrate it into RansacLib.

Other examples provides in `examples/` are used for internal testing and can be safely ignored.

There are currently three dependencies:
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [OpenGV](https://github.com/laurentkneip/opengv)
* [Ceres Solver](http://ceres-solver.org/)

**Important**: Eigen requires alignment of [certain types](https://eigen.tuxfamily.org/dox-devel/group__TopicFixedSizeVectorizable.html) when vectorization is used. OpenGV uses vectorization to accelerate some computations. As such, it is critical to compile the examples that depend on OpenGV with exactly the same optimization flags as OpenGV. This should be ensured at the moment for Unix-based systems (Linux, Mac OS X), but we did not test it under Windows. If you are experiencing problems at run-time or during compilation, please compare `examples/CMakeLists.txt` with `opengv/CMakeLists.txt` and if nessecary add additional flags to `examples/CMakeLists.txt`.

## Using RansacLib
RansacLib uses templates to enable easy integration of novel solvers into RANSAC. More precisely, three classes need to be defined: `class Model`, `class ModelVector`, `class Solver`. These classes are explained in more detail in the following:

### Model Class
A class that describeds the model computed by your minimal solver. In the case of a solver for calibrated absolute pose estimation, this could for example be an `Eigen::Matrix<double, 3, 4>` describing the rotation and translation that transforms from the global to the local camera coordinate system. In the case of 2D line fitting, this could be an `Eigen::Vector3d` describing the three parameters of an implicit line representation. The class must have a default constructor and assignments via the operator `=` must be possible.

### ModelVector Class
A class that contains multiple `Model` instances. In the case of 2D line fitting, this could be a `std::vector<Eigen::Vector3d>` storing line representations. In the case of absolute pose estimation, this could be a `std::vector<Eigen::Matrix<double, 3, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>`. Note that in the later case, Eigen requires the use of its `aligned_allocator` to ensure alignment for vectorization (see [here](https://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html) for details). This requirment is what prevents us from directly using `std::vector<Model>` in RansacLib.

The `ModelVector` class needs to implement the operator `[]` such that a call `[i]` returns the i-th model stored in a class instance. In addition, the class must have a default constructor and assignments via the operator `=` must be possible.

### Solver Class
The `Solver` class implements all functionality to estimate and evaluate minimal models. In addition, a `Solver` instance can provide functionality for the creation of model hypotheses from non-minimal samples and (non-linear) refinement of a model hypotheses. The following shows the how to implement a solver (see also the examples provided with RansacLib):
```
class MySolver {
 public:
  // Returns the number of data points required by a minimal solver.
  int min_sample_size() const;
  
  // Returns the number of data points required by a non-minimal solver for
  // the problem. This number should be larger than 0, even if your class 
  // does not implement a non-minimal solver to avoid trying to generate
  // samples of size 0 inside RANSAC.
  int non_minimal_sample_size() const;
  
  // Returns the number of data points stored in the solver.
  int num_data() const;
  
  // A call to the minimal solver implemented inside the solver. The input
  // is a set of indices between 0 and num_data() - 1 that form the random
  // minimal sample drawn by RANSAC. sample.size() is guaranteed to be
  // min_sample_size(). The function is responsible for running the minimal
  // solver on these selected data points. The estimated models are returned
  // in models and the function returns the estimated number of models.
  int MinimalSolver(const std::vector<int>& sample,
                    ModelVector* models) const;

  // A call to a non-minimal solver implemented in the class. This function
  // is called during local optimization to generate a non-minimal sample from
  // the inliers of the best model found so far. The input contains a list of
  // indices of the data points that should be used to generate the non-minimal
  // sample. It is assumes that the non-minimal solver provides at most one 
  // model, which is returned in model. The function returns 1 if a model could
  // be estimated and 0 otherwise.
  // This function has to be implemented, but could simply always return 0.
  // In this case, local optimization becomes ineffective as no new models are
  // created and refined. However, the best model estimated from a minimal sample
  // is still refined.
  // sample.size() is guaranteed to be non_minimal_sample_size() or larger.
  int NonMinimalSolver(const std::vector<int>& sample,
                       Model* model) const;

  // Evaluates a given model on the i-th data point and returns the squared 
  // error of that correspondence wrt. the model.
  double EvaluateModelOnPoint(const Model& model, int i) const;

  // Performs least squares refinement of a given input Model model. On return,
  // model contains the refined model. sample contains the indices of the data
  // points that should be used to refine the model.
  // This function has to be implemented, but can simply be empty. In this case
  // the input model is left unaltered and no refinement occurs. Not implementing
  // this function will decrease the effectiveness of local optimization and RANSAC.
  void LeastSquares(const std::vector<int>& sample, Model* model) const;
};
```

**Important**: RansacLib does not use a class that encapsulates the input data, e.g., 2D points for 2D line fitting or 2D-3D matches for absolute pose estimation. This is a deliberate design choice: some solvers might require additional data (for example, a solver that computes poses for multi-camera rigs might need a mapping from 2D-3D matches to cameras in the rig) in some very specific form. We thus decided to make the `Solver` class directly responsible for handling all data, including the input correspondences, rather than trying to constantly expand a data class to fit additional needs.

**Important**: Note that all mandatory functions defined above are `const` and do not alter the state of the solver. This is a deliberate design choice: the `Solver` class also encapulates the input data, e.g., 2D-3D matches for absolute pose estimation. This data should not be altered by the solver. We thus pass the solver into RANSAC as `const Solver& solver`. We acknowledge that this could potentially be restricting in some cases and are open to suggestions on how to guarantee that the input data is not altered while allowing the solver to change its internal state.

### HybridSolver Class
The Hybrid RANSAC implementation requires the use of a `HybridSolver` rather than the `Solver` class. As with the `Solver` class, the `HybridSolver` class implements all functionality to estimate and evaluate minimal models. In addition, it provided additional functionality to enable the use of multiple minimal solvers inside RANSAC. Note that the class does not provide a non-minimal solver implementation as of now (due to the ambiguity in how to define a non-minimal solver for different types of data). The following shows the how to implement a solver (see also the examples provided with RansacLib):
```
class MyHybridSolver {
public:
// Returns the number of minimal solvers.
int num_minimal_solvers() const;

// Returns the number of data points required by each minimal solver.
// Note that even if solver i does not require data of a certain type, the
// corresponding entry in min_sample_sizes[i] still needs to be set to 0.
void min_sample_sizes(std::vector<std::vector<int>>* min_sample_sizes) const;

// Returns the number of data types stored in the solver.
int num_data_types() const;

// Returns the number of data points stored in the solver for each type of
// data.
void num_data(std::vector<int>* num_data) const;

// Returns the prior probabilities for all solvers.
void solver_probabilities(std::vector<double>* solver_probabilites) const;

// A call to one of the minimal solvers implemented inside the solver. The
// input are multiple sets of indices, each between 0 and
// num_data[solver_idx] - 1 that form the random minimal sample drawn by 
// HybridRANSAC. In addition, the index solver_idx of the minimal solver
// selected by HybridRANSAC is provided as input. The overall size of the
// sample is guaranteed to be sample_sizes[solver_idx]. The function is 
// responsible for running the solver_idx-th minimal solver on these
// selected data points. The estimated models are returned in models and 
// the function returns the estimated number of models.
int MinimalSolver(const std::vector<std::vector<int>>& sample,
                  const int solver_idx, ModelVector* models) const;

// Evaluates a given model on the i-th data point of the t-th data type
// and returns the squared error of that correspondence wrt. the model.
double EvaluateModelOnPoint(const Model& model, int t, int i) const;

// Performs least squares refinement of a given input Model model. On
// return, model contains the refined model. sample contains the indices
// of the data points that should be used to refine the model.
// This function has to be implemented, but can simply be empty. In this case
// the input model is left unaltered and no refinement occurs. Not implementing
// this function will decrease the effectiveness of local optimization and RANSAC.
void LeastSquares(const std::vector<std::vector<int>>& sample,
                  Model* model) const;
};
```

## License
RansacLib is licensed under the BSD 3-Clause license. Please see [License](https://github.com/tsattler/RansacLib/blob/master/LICENSE) for details.

If you are using RansacLib, please consider adding the name of your project / company (and a link to your / your project's webpage) to the list of projects using RansacLib below.

## Citing
If you are using the library for (scientific) publications, please cite the following source:
```
@misc{Sattler2019Github,
  title = {{RansacLib - A Template-based *SAC Implementation}},
  author = {Torsten Sattler and others},
  URL = {https://github.com/tsattler/RansacLib},
  year = {2019}
}
```
Please cite also the original publications of the different methods:
* When using the ```LocallyOptimizedMSAC``` implementation in [ransac.h](https://github.com/tsattler/RansacLib/blob/master/RansacLib/ransac.h), please cite
```
@inproceedings{Lebeda2012BMVC,
  title = {{Fixing the Locally Optimized RANSAC}},
  author = {Karel Lebeda and Jiri Matas and Ondrej Chum},
  booktitle = {British Machine Vision Conference (BMVC)},
  year = {2012}
}
```
* When using the ```LocallyOptimizedMSAC``` implementation in [ransac.h](https://github.com/tsattler/RansacLib/blob/master/RansacLib/ransac.h) with `num_lo_steps_` set to 0, please cite
```
@inproceedings{Torr1998ICCV,
  title = {{ Robust computation and parametrization of multiple view relations}},
  author = {Phil H. S. Torr and Andrew Zisserman},
  booktitle = {International Conference on Computer Vision (ICCV)},
  year = {1998}
}
```
* When using the ```LocallyOptimizedHybridMSAC``` implementatio in [ransac.h](https://github.com/tsattler/RansacLib/blob/master/RansacLib/ransac.h), please cite
```
@inproceedings{Camposeco2018CPVR,
title = {{Hybrid Camera Pose Estimation}},
author = {Federico Camposeco and Andrea Cohen and Marc Pollefeys and Torsten Sattler},
booktitle = {Conference on Computer Vision and Pattern Recognition (CVPR)},
year = {2018}
}
```
and the paper by Lebeda et al.

## Contributing
Contributions to RansacLib, e.g., bug reports, improvements, or bug fixes, are very welcome. Please use Github's issues and pull request functionality to contribute. 

When contributing, please adhere to [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

## List of Projects using RansacLib
* Active Search v1.1 uses RansacLib instead of its original RANSAC implementation. Code for Active Search will become available [here](https://github.com/tsattler/vps).
* [radialpose](https://github.com/vlarsson/radialpose) is an implementation of minimal solvers for absolute camera pose estimation for images with radial distortion.
