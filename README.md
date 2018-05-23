# connlib - Connection Library

connlib is a simple library to compute the minimum length required to connect
`N` nodes in a 2D plane.

## Compiling

### Dependencies

* CMake 3.0 or higher
* g++ 5.0 or higher

### Compiling with CMake

Create a new build directory and cd into it, e.g.:

```
mkdir build
cd build
```

Run CMake to configure and make:

```
cmake ..
make -j8
```

## Tests

In the `tests/` directory, there exists a Google Test source to test various
sets of data. To compile that, you will need Google Test installed on your
system.

There are a few extra utilities in `include/connlib/utils.h` to help with the
testing. A simple test example utilizing those methods can be found in
`tests/utils_example.cpp`.


## Tools

There is a sample code that reads from stdin and writes the result to stdout.
This tool is in `tools/connlib_sample.cpp`.


### Input Format

The input format consists simply on a first line containing a single integer `N`
denoting the number of nodes, `N` lines follow, each containing two floats,
`x[n]` and `y[n]`.

Example:

```
3
0.0 0.0
1.0 1.0
2.0 2.0
```

### Output Format

The output format prints `N` lines, the first one with the total length,
followed by one line for each connection, where each connection has the format
`SrcNode.x SrcNode.y DstNode.x DstNode.y Length`. Each number is printed with
6-digit decimal precision.

Example:

```
2.828427
0.000000 0.000000 1.000000 1.000000 1.414214
1.000000 1.000000 2.000000 2.000000 1.414214
```


# Data Type

There exists two basic data types: `Node` and `Edge`

* `Node`: a simple struct to hold the x and y coordinates of a node or vertex,
  containing also an index (for internal usage only). There exists also a
  `NodePtr` type, which is merely an `std::shared_ptr<Node>`, used by the
  `Edge` to store nodes.
* `Edge`: a struct holding two nodes stored as `NodePtr` and the length of a
   connection between them.


# Complexity

* Adjacency list setup: O(N^{2}/2)
* Solver: O(N^{2} log(N)))
* Simplified Overall Complexity: O(N^{2} log(N))

With the overall complexity, the code should perform reasonably well to an order
of magnitude of 10^{3}.
