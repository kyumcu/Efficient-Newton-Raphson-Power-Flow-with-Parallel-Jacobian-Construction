# Efficient-Newton-Raphson-Power-Flow-with-Parallel-Jacobian-Construction
Source code for the article "Efficient Newton-Raphson Power Flow with Parallel Jacobian Construction"

## Getting started

This project aims to explore the benefits of psychically aware modeling of electrical grids. 
while modeling physical systems with real world correspondence, the limitations of reality will impose restrictions to system itself.

Parallel Power Flow Solver, implements a NR solver for solving the energy balance equations of model grids by leveraging such limitations.

## Dependencies 

Test data is taken from Matpower, hence a copy of `Matpower/data` dir is required to run the test cases. Easiest way to do this is to set up from;

```
https://matpower.org/about/get-started/
```

Build requires;

- `make`

and 

- `eigen`
```
sudo apt install libeigen3-dev
```

Check if you already have eigen

```
find /usr/include /usr/local/include -type d -name Eigen
```

- `Google Test (GTest)`
```
sudo apt install libgtest-dev
```


## Build

Makefile is responsible from two separate executable, `main` and `main_test`.
Currently `main` has no practical use. But in the future it would be the basis for timing benchmarks.
On the other hand `main_test` is the hearth of the development. 
It executes all tests using `gtest`. 


### A basic usage

Clean before use;
```
make clean
```

In project root;
```
make test
```

will build and run the tests from a clean build and output directory.

```
make main
```
will build the main executable from a clean build and output directory.

### To run the tests

It is required to define the source directory for grid data files. This is done through the `MPOWERDIR` variable in the make file.

```
MPOWERDIR = $(HOME)/matpower7/
```

Set this variable to Matpower root directory. (probably at ~/matpower)

