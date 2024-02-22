import gymnasium as gym
import panda_gym
import time
from stable_baselines3 import DDPG

env = gym.make("PandaThrow-v3", render_mode="human")
env.reset()

models_dir = "models/DDPG_2"
logdir = "logs"

model_path = f"{models_dir}/140000.zip"
model = DDPG.load(model_path, env=env)

episodes = 10

vec_env = model.get_env()
obs = vec_env.reset()

for ep in range(episodes):
    obs = vec_env.reset()
    done = False
    while not done:
        vec_env.render()
        action, _states = model.predict(obs, deterministic = True)
        obs, reward, done, info = vec_env.step(action)
        time.sleep(.05)

vec_env.close()
