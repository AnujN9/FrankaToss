# from stable_baselines3.common import env_checker
import gymnasium as gym
import panda_gym
# from panda_gym.pybullet import PyBullet
import numpy as np
import time
from stable_baselines3 import DDPG

env = gym.make("PandaThrow-v3", render_mode="human")

models_dir = "models/DDPG_2"
logdir = "logs"

model_path = f"{models_dir}/140000.zip"
model = DDPG.load(model_path, env=env)

episodes = 10

vec_env = model.get_env()
obs = vec_env.reset()

for ep in range(episodes):
    obs = vec_env.reset()
    dones = False
    while not dones:
        vec_env.render()
        action, _states = model.predict(obs, deterministic = True)
        obs, rewards, dones, infos = vec_env.step(action)
        print(dones)
        time.sleep(.5)

vec_env.close()
