import gym
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN, PPO


def create_env():
    """Create and return the CartPole-v1 environment."""
    env = gym.make('CartPole-v1')
    return env


def train_dqn_model(env, total_timesteps=300000, log_dir='./logs'):
    """Train a DQN model with the specified environment and return the trained model."""
    model = DQN("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    model.learn(total_timesteps=total_timesteps, tb_log_name='dqn_cartpole')
    return model


def train_ppo_model(env, total_timesteps=100000, log_dir='./logs'):
    """Train a PPO model with the specified environment and return the trained model."""
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    model.learn(total_timesteps=total_timesteps, tb_log_name='ppo_cartpole')
    return model


def load_dqn_model(model_filename, env):
    """Load a pre-trained DQN model from the specified file."""
    model = DQN.load(model_filename, env=env)
    return model


def load_ppo_model(model_filename, env):
    """Load a pre-trained PPO model from the specified file."""
    model = PPO.load(model_filename, env=env)
    return model


def run_agent(model, env):
    """Run the agent in the environment using the trained model."""
    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()


def main():
    algorithm = 'dqn'  # Choose between 'dqn' and 'ppo'
    total_timesteps = 150000
    model_filename = f'{algorithm}_cartpole_{total_timesteps}'
    train_model_flag = True
    env = create_env()

    if train_model_flag:
        if algorithm == 'dqn':
            model = train_dqn_model(env = env,total_timesteps=total_timesteps)
        elif algorithm == 'ppo':
            model = train_ppo_model(env = env,total_timesteps=total_timesteps)
        else:
            raise ValueError("Invalid algorithm. Choose either 'dqn' or 'ppo'.")
        model.save(model_filename)
    else:
        if algorithm == 'dqn':
            model = load_dqn_model(model_filename, env)
        elif algorithm == 'ppo':
            model = load_ppo_model(model_filename, env)
        else:
            raise ValueError("Invalid algorithm. Choose either 'dqn' or 'ppo'.")

    #run_agent(model, env)


if __name__ == '__main__':
    main()
