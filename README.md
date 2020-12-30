
# Using Uncertainty Data in Chance-Constrained Trajectory Planning
This repository contains the source code used for the paper titled "Using Uncertainty Data in Chance-Constrained Trajectory Planning" presented at the 2019 European Control Conference (ECC). DOI: [10.23919/ECC.2019.8795823](https://doi.org/10.23919/ECC.2019.8795823)

## Dependencies
The following dependencies need to be installed/configured and must be on the MATLAB path:
- [YALMIP](https://yalmip.github.io/) (configured with an appropriate solver like [CPLEX]( https://www.ibm.com/analytics/cplex-optimizer))
- [matlab2tikz](https://github.com/matlab2tikz/matlab2tikz)
- [MagInset](https://www.mathworks.com/matlabcentral/fileexchange/49055-maginset)

## Generating plots
In order to reproduce the simulations and plots of the paper, navigate inside the `case_study` folder and run the `generatePlotsCaseStudy` MATLAB function. The function will run all the necessary simulations and produce all the plots of the paper. The plots will also be saved under a `plots` folder in TikZ format, which can then be readily included in a LaTeX document.

## Citing this work
Please cite the original paper when using any part of this code. BibTeX citation data:
```
@inproceedings{Lefkopoulos2019,
    author    = "V. Lefkopoulos and M. Kamgarpour",
    title     = "Using Uncertainty Data in Chance-Constrained Trajectory Planning",
    booktitle = "2019 European Control Conference ({ECC})",
    year      = "2019",
    pages     = "2264-2269",
}
```
