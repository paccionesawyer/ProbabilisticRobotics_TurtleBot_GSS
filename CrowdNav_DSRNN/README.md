# DSRNN_CrowdNav
This repository contains the code for the final project in Probabilistic Robotics, adapted from the paper "Decentralized Structural-RNN for Robot Crowd Navigation with Deep Reinforcement Learning" by Liu et al in ICRA 2021. Please refer to their [GitHub](https://github.com/Shuijing725/CrowdNav_DSRNN/) or [paper](https://arxiv.org/abs/2011.04820) for more information.

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

    a. To run the simulation with the different conditions used in this project, change the `robot.mode` setting in the config file. 
       `"normal"` is the baseline Liu et al. algorithm,  `"noisy"` the added noise variation, and  `"kalman"` the filtered algorithm.

### Run the code
1. Train a policy. 
```
python train.py 
```

2. Test policies.   
```
python test.py --test_model [model_path]
```
where `[model_path]` is the path to the directory for the model to test (eg `data/noisy_5e6` or `data/kalman_5e6` for the models created in this project).  


## Novel Code
This section identifies the major changes made to the code for the Probabilistic Robotics project. Full details of changes can be seen in the commit history, and specific changes are commented within each file.

- `CrowdNav_DSRNN/crowd_nav/configs/config.py`: Config options were added to change the robot mode for the variations of this project and to implement the noise.

- `CrowdNav_DSRNN/crowd_sim/envs/crowd_sim.py`: Rendering for robot believed position was added, as well as modifications for noise.

- `CrowdNav_DSRNN/crowd_sim/envs/crowd_sim_dict.py`: The noise implementation was fixed.

- `CrowdNav_DSRNN/crowd_sim/envs/utils/agent.py`: Noise was implemented between the true and believed agent positions. 

- `CrowdNav_DSRNN/crowd_sim/envs/utils/robot.py`: Kalman filter was added, as well as functions to support computation of the noisy and filter-predicted states.

New models were also added to `data` folder.

## Contributions
Augustus Brown compiled and ran all the training and testing of the models, as well as contributing to the implementation of the noisy model variation, as well as the report.

Sawyer Paccione contributed to the development of the noisy and kalman filter model variations, as well as the report.

Stephanie Bentley contributed to development of the kalman filter variation, as well as a significant portion of the report.


## Acknowledgements

This code is based on the code from Liu, et al.

S. Liu, P. Chang, W. Liang, N. Chakraborty, and K. Driggs-Campbell, “De-
centralized structural-RNN for robot crowd navigation with deep reinforcement
learning,” pp. 3517–3524, May 2021, ISSN: 2577-087X. doi: 10.1109/ICRA48506.
2021.9561595
