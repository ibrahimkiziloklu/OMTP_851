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
```
Here is the gif of the random agent:
![Random Agent](https://github.com/ibrahimkiziloklu/OMTP_851/blob/main/omtp_lecutre10/random_agent.gif)
Then we can train the agent with the following command:

```pyhton panda_reach_train_agent.py```
This will train the for 200000 steps and saves the model in the current directory with the current in the name.




