# RoutingKit
RoutingKit is a C++ library that provides advanced route planning functionality. 
It was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/).
The **original version of RoutingKit** can be found at https://github.com/RoutingKit/RoutingKit.

**This fork** was used to implement and test several CH based algorithms, including
- Contraction Hierarchies with Label Restrictions (CHLR) by Rice and Tsotras (2010)
- CHLRMaintenance algorithm by Chen et al. (2023)
- modified DCH algorithms (based on Zhang and Yu 2022) supposed to work on CHLR

Notable contents of these implementations are
- `Label` class, implemented in `src/label.cpp` for representing sets of labels within unsigned integers
- `CHLR` building and query in `src/chlr.cpp`
- failed `CHLRMaintenance` implementation attempt in `src/chm.cpp`
- modified `DCH` algorithms in `src/dch.cpp`

In order to execute those algorithms, have a look at
- `src/test_chlr.cpp` for the basic CHLR implementation
- `src/test_dch.cpp` which implements both tests and benchmarking.
  - For executing parts (specifically `DCHPlus` and `DCHMinus`) of it, it is necessary to comment out the witness search part within `src/chlr.cpp`.
  - `DCHPlusMod` is a novel approach/experiment to paritally rebuild the CHLR index without the need of removing the witness search

The project can be built using CMake by running
```
mkdir build
cd build
cmake ..
make
```
