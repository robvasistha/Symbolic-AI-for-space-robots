# Symbolic AI for Space Robots

##Summary
This repository contains all of the project files for Symbolic AI for Space Robots

This repository makes use of two sub-repositories which are linked below:

https://gitlab.surrey.ac.uk/lj00304/spacerobot_v1/-/tree/master/

https://github.com/brendenpetersen/deep-symbolic-optimization

Symbolic AI for Space robots is a project designed to put deep learning through symbolic optimisation to the test against a Space Robot simulator designed here at the University of Surrey by Lucy Jackson.

## Installation
All of the requirements for installation can be found in the requirements.txt in order to run the python training scripts.
All of the requirements for DSO training can be found in the folder called deep-symbolic-optimisation in a file called requirements.txt

It is recommended to create a virtual environment to host all of these libraries.

This can be done by the following:
```
           conda create -n <ENVNAME> python=3.9

           conda activate <ENVNAME>

           pip install -r requirements.txt

```
**It should be noted that DSO requires Tensorflow 1.14 which is only supported up to Python 3.7 so you may need to create two virtual environments.**

The CSVs containing the results that were used in the graphs for the dissertation can be found in the following folders:
Training/logs/test_200K
deep-symbolic-optimisation/dso/logs 
