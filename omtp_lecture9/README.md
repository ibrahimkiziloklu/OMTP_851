# CartPole with DQN and PPO using Stable Baselines 3

This repository contains a Python script to train a reinforcement learning agent to solve the CartPole-v1 environment using two different algorithms: Deep Q-Network (DQN) and Proximal Policy Optimization (PPO). The agent is trained using the Stable Baselines 3 library.

## Table of Contents

1. [Dependencies](#dependencies)
2. [Installation](#installation)
3. [Introduction](#introduction)
4. [Environment](#environment)
5. [Algorithms](#algorithms)
    * [DQN](#dqn)
    * [PPO](#ppo)
6. [Usage](#usage)
7. [Running the Agent](#running-the-agent)
8. [Tensorboard](#Tensorboard)

## Dependencies

The following dependencies are required to run the script:

- Python 3.6 or higher
- OpenAI Gym
- Stable Baselines 3
- Tensorflow
- TensorBoard (optional, for monitoring training progress)

## Installation

To install the required dependencies, you can use the following pip commands:

```
pip install gym
pip install stable-baselines3[extra]
pip install tensorflow
pip install tensorboard
```
In case you encounter the error "NameError: name 'glPushMatrix' is not defined", you can resolve it by installing version 1.5.27 of the `pyglet` library:

```
pip install pyglet==1.5.27
```

This version of `pyglet` is compatible with the code and should fix the 'glPushMatrix' error.

## Introduction

Reinforcement learning is a type of machine learning where an agent learns to make decisions by interacting with an environment. The agent receives feedback in the form of rewards, which are used to adjust its behavior. The goal is to learn a policy that maximizes the expected cumulative reward.

The CartPole problem is a classic control task in reinforcement learning. The goal is to balance a pole on a cart by applying forces to the cart. The agent receives a reward of +1 for every time step it keeps the pole upright, and the episode terminates when the pole falls or the cart moves too far from the center.

## Environment

The environment used in this project is the `CartPole-v1` environment from the OpenAI Gym library. It consists of a cart that can move horizontally with a pole hinged to the cart. The agent can apply forces to the cart, either pushing it to the left or to the right. The agent's goal is to balance the pole for as long as possible.

The observation space of the environment is continuous and has four dimensions:

1. Cart position
2. Cart velocity
3. Pole angle
4. Pole angular velocity

The action space is discrete and has two possible actions:

1. Push the cart to the left
2. Push the cart to the right

## Algorithms

### DQN

Deep Q-Network (DQN) is a reinforcement learning algorithm that combines Q-learning with deep neural networks. DQN uses a neural network to approximate the Q-function, which maps state-action pairs to expected future rewards. The agent selects actions by choosing the one that maximizes the Q-value.

![No training](https://github.com/ibrahimkiziloklu/OMTP_851/blob/main/omtp_lecture9/notrain.gif)
![Trained for 1 million timesteps](https://github.com/ibrahimkiziloklu/OMTP_851/blob/main/omtp_lecture9/1million.gif)

### PPO

Proximal Policy Optimization (PPO) is a policy gradient algorithm for reinforcement learning. It aims to optimize the agent's policy by maximizing the expected cumulative reward while ensuring that the new policy does not deviate too far from the old policy. This is achieved using a clipped objective function that penalizes large policy updates.

## Usage

To train a new model, set the `algorithm` variable in the `main` function to either `'dqn'` or `'ppo'`. Set the `train_model_flag` variable to `True` for training or `False` for loading a pre-trained model.

```python
def main():
    algorithm = 'ppo'  # Choose between 'dqn' and 'ppo'
    total_timesteps = 100000
    model_filename = f'{algorithm}_cartpole_{total_timesteps}'
    train_model_flag
```    
    
## Tensorboard
