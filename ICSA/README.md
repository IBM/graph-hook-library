# ICSA Submission Companion

This repository contains the artifacts accompanying the paper compiled at `ICSA/publication/main.tex`. The layout below helps you locate code, data, and supporting materials referenced in the manuscript.

## Folder Structure
- `publication/`: LaTeX source of the ICSA paper and related assets.
- `vf2_vf3/`:  Code for generating Tables II and III and Figure 3.
- `isomorphisms/`:  Code for generating Table IV.
- `dnn/`:  Code for generating Figure 4.
- `ghlchem/`:  Code for generating Figures 5 and 6.
- `cities/`:  Isomorphism of Figure 2
- `listing/`:  Self-contained example in the publication appendix.
- `si/`:  Dataset directory (initially unpopulated).

## Requirements
### Downloading the SI dataset
Before running benchmarks in `vf2_vf3` and `isomorphisms`, download the SI dataset by executing `get_si.sh` in `si/`. This pulls and formats the necessary files.

### Dependencies
GHL is benchmarked against iGraph, NetworkX, GraphTool, and RDKit. Install iGraph, NetworkX, and RDKit via:
```
pip install igraph networkx rdkit
```
GraphTool is not available via pip; a conda environment is recommended.

## vf2_vf3 (C++ project)
Benchmarks comparing the BGL VF2 implementation and the GHL VF3 implementation. Compile GHL (from the project root):
```
mkdir build && cd build
cmake .. -DGHL_SERIALIZATION=ON -DGHL_BINDINGS=ON -DCMAKE_INSTALL_PREFIX:PATH=install
cmake --build . --target install
```
Note the dependencies described in the top level README.

This copies the `vf3_vs_vf2` program to `vf2_vf3/`. Run `run_vf3_vf2_profile.sh` to generate results. `stride` and `start_index` can be used to parallelize the benchmarks across compute nodes if operating in such an environment, otherwise, leaving them unset runs all tests serially. Full benchmarks are long-running (each data point up to 5 minutes; VF2 failed on 696 tests in our experiments, leading to a minimum serialized runtime of 58 hours). Calculated results are included in `vf2_vf3/results/`. Use `report_results.py` to generate Tables II and III and Figure 3.

## isomorphisms
Generates Table IV by calling iGraph, NetworkX, GraphTool and the GHL on the SI dataset. These runs are also long-running, we have therefore included our results in the `results\` directory. This and subsequent benchmarks use GHL as a Python library; install via the following command in the top level directory:
```
pip install -e .
```

Use `generate_analysis.ipynb` to produce Table III once results are available.

## dnn

The `dnn` directory contains the results and generation script for Figure 4. The framework for generating these results is in fact the primary reason we developed the GHL library and is not yet open sourced. Regretably this means that we cannot provide the source code to generate this portion of the results. We hope that the other results make up for this drawback.

## ghlchem
Generates Figures 5 and 6 using GHL as a Python library. Requires that the molecules sdf files be downloaded via the included script `get_mols.sh`.
After this, benchmarking may be run via `python benchmark.py`, which will transparently compile the extending code in ghlchem.cpp and ghlchem.h, run the benchmarks, and generate Figures 5 and 6.

## cities

The `cities` directory contains the isomorphism that converts Figure 2(b) to Figure 2(f).

## listing

Lastly, `listing` contains the self-contained example described in the appendix. It can be called similarly to the ghlchem benchmark.



