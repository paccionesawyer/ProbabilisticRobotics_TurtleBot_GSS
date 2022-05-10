# DSRNN_CrowdNav
This repository contains the code for the final project in Probabilistic Robotics


## Setup
1. Install Python3.6 (The code may work with other versions of Python, but 3.6 is highly recommended).
2. Install the required python package using pip or conda. For pip, use the following command:  
```
pip install -r requirements.txt
```
For conda, please install each package in `requirements.txt` into your conda environment manually and 
follow the instructions on the anaconda website.  

3. Install [OpenAI Baselines](https://github.com/openai/baselines#installation).   
```
git clone https://github.com/openai/baselines.git
cd baselines
pip install -e .
```

4. Install [Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2) library.  


## Getting started
This repository is organized in three parts: 
- `crowd_sim/` folder contains the simulation environment. Details of the simulation framework can be found
[here](crowd_sim/README.md).
- `crowd_nav/` folder contains configurations and non-neural network policies
- `pytorchBaselines/` contains the code for the DSRNN network and ppo algorithm.  
 
Below are the instructions for training and testing policies.

### Change configurations
1. Environment configurations and training hyperparameters: modify `crowd_nav/configs/config.py`


### Run the code
1. Train a policy. 
```
python train.py 
```

2. Test policies.   
```
python test.py 
```

This code is based on the code from Liu, et al.

S. Liu, P. Chang, W. Liang, N. Chakraborty, and K. Driggs-Campbell, “De-
centralized structural-RNN for robot crowd navigation with deep reinforcement
learning,” pp. 3517–3524, May 2021, ISSN: 2577-087X. doi: 10.1109/ICRA48506.
2021.9561595
