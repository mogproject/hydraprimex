# Hydra Prime X

*Note: This software is experimental.*

The solver Hydra Prime X finds an exact solution to the following *generalized* [Twin-Width](https://en.wikipedia.org/wiki/Twin-width) problem.
The code is developed based on [Hydra Prime](https://github.com/TheoryInPractice/hydraprime).

#### Annotated Twin-Width

**Input:** An undirected trigraph $G=(V,B \cup R)$ with *black* edges $B$ and *red* edges $R$.

**Output:** A contraction sequence that contracts $G$ to a single vertex and minimizes the maximum red degree during contractions.

----

### Getting Started

1. Read [Input and Output](https://github.com/mogproject/hydraprimex/wiki/Input-and-Output). You may use this [script](https://github.com/mogproject/hydraprimex/blob/main/scripts/create_trigraph.py) to convert file formats.
1. Install a C++ compiler, GNU Make, and CMake.
1. Run `make`.
1. Run executable `build/Release/hydrax`. You may move this file to any path.

----

### Dependencies

- C++ compiler supporting the C++14 standard ([GCC](https://gcc.gnu.org/) recommended)
- [GNU Make](https://www.gnu.org/software/make/)
- [CMake](https://cmake.org/) Version 3.14 or later
- [CLI11](https://github.com/CLIUtils/CLI11)
- [The Kissat SAT Solver 3.0.0](https://github.com/arminbiere/kissat)

### Program usage

| Operation | Command |
|:---|:---|
|Print help message | `hydrax --help`|
|Print version | `hydrax --version`|
|Output optimal contraction sequence | `hydrax PATH` or `hydrax < PATH` <br>(PATH: path to a problem instance file)|
|Output only twin-width | `hydrax --tww PATH` |

### Build usage

Use `make` in this project directory.

| Operation | Command |
|:---|:---|
|Build release version | `make build` $\to$ creates executalbe `/build/Release/hydrax`|
|Build debug version (with logging features etc.) | `make build-debug` $\to$ creates `build/Debug/hydrax`|
|Run unit tests | `make test` |
|Clean build files | `make clean` |
