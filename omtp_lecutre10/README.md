# README
For the installation of this exercise, we refer you to this [Reinforcement Learning with the Franka Panda robot pybullet simulation](https://github.com/simonbogh/rl_panda_gym_pybullet_exampleher%20Panda-Gym) repository.
In order to save a .gif of the simulation, you need to install the ```imageio``` package:
```
pip install imageio
```

After the installation, first we test the environment by running a random agent with the following command:
```
python panda_reach_random_agent.py
```
Note that to save the save the .gif, you need to make list of the frames rendered by the environment. After using ```imageio.mimwrite``` we can saved the .gif file.
So it would like something like this:
```
import os
import imageio
... # other imports
frames = [] # list of frames. defined inside the run_random_agent function
... # other parts of the code
frames.append(env.render(mode='rgb_array'))
... # other parts of the code
for i in range(50): # take 50 frames
    ... # other parts of the code
    frames.append(env.render(mode='rgb_array'))
... # other parts of the code
imageio.mimwrite('random_agent.gif', frames)
```
Here is the gif of the random agent:

![Random Agent](https://github.com/ibrahimkiziloklu/OMTP_851/blob/main/omtp_lecutre10/random_agent.gif)

Then we can train the agent with the following command:

```pyhton panda_reach_train_agent.py```
This will train the for 200000 steps and saves the model in the current directory with the current in the name.

With the same method, we can save the .gif and test the agent. Here is the gif of the trained agent for 200 steps:

```pyhton panda_reach_test_agent.py```

![Trained Agent](https://github.com/ibrahimkiziloklu/OMTP_851/blob/main/omtp_lecutre10/test_agent.gif)

## Tensorboard
We can also use tensorboard to see the training process. With same way as the previous exercise, we can run the following command:
```
tensorboard --logdir ./runs
```
Then we can see the training process in the tensorboard. Here is the screenshot of the tensorboard:
this tensorboard shows two trainingprocesses. One is for 200000 steps and the other one is for 100000 steps. We can see that the training process is converging to the goal.

![Tensorboard](https://github.com/ibrahimkiziloklu/OMTP_851/blob/main/omtp_lecutre10/tensorboard_lec10.png)

## References
[1] [Reinforcement Learning with the Franka Panda robot pybullet simulation](https://github.com/simonbogh/rl_panda_gym_pybullet_exampleher%20Panda-Gym)





