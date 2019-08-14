# RansacLib

Header-only implementation of RANSAC and its variants.

More details coming soon ...

## Citing
If you are using the library for (scientific) publications, please cite the following source:
```
@misc{Sattler2019Github
  title = {{RansacLib - A Template-based *SAC Implementation}},
  author = {Torsten Sattler},
  URL = {https://github.com/tsattler/RansacLib},
  year = {2019}
}
```
Please cite also the original publications of the different methods. When using the ```LocallyOptimizedMSAC``` implementation in [ransac.h](https://github.com/tsattler/RansacLib/blob/master/RansacLib/ransac.h), please cite
```
@inproceedings{Lebeda2012BMVC
  title = {{Fixing the Locally Optimized RANSAC}},
  author = {Karel Lebeda and Jiri Matas and Ondrej Chum},
  booktitle = {British Machine Vision Conference (BMVC)},
  year = {2019}
}
```

## Testing
To test the implementation, we provide a simple example, namely 2D line fitting. 
In order to build the example, ```CMake``` and ```Eigen 3.3.X``` is required.
* Create a build directory: ```mkdir build```.
* Build the example: ```cd build```, ```cmake ..```, ```make```
* You can run the executable via ```./example/line_estimation```
