import gymnasium as gym
import panda_gym
import os
import time
from stable_baselines3 import DDPG, HerReplayBuffer

env = gym.make("PandaThrow-v3")
env.reset()

t_val = int(time.time())

models_dir = f"models/DDPG_2_{t_val}"
logdir = "logs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(logdir):
    os.makedirs(logdir)

rb_kwargs = {'goal_selection_strategy' : 'future',
             'n_sampled_goal' : 4}

policy_kwargs = {'net_arch' : [512, 512, 512], 
                 'n_critics' : 2}

model = DDPG('MultiInputPolicy', env=env, verbose=1, learning_rate = 1e-3,
             buffer_size=100000, batch_size= 2048, tau= 0.01, gamma= 0.95,
             replay_buffer_class=HerReplayBuffer, replay_buffer_kwargs = rb_kwargs,
             policy_kwargs = policy_kwargs, tensorboard_log=logdir)

TIMESTEPS = 10000
iters = 0
for i in range(1,100):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"DDPG_2_{t_val}")
    model.save(f"{models_dir}/{TIMESTEPS*i}")
