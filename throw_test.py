import gymnasium as gym
import panda_gym
import time
from stable_baselines3 import SAC
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc

env = gym.make("PandaThrow-v3", render_mode="human", control_type="joints")
# env = gym.make("PandaThrow-v3", render_mode="human")
env.reset()

# models_dir = "models/SAC_1708896918"
models_dir = "models/SAC_latest"
logdir = "logs"

# 880000 was working well for 918
# 570000 was working well for latest
model_path = f"{models_dir}/540000.zip"
model = SAC.load(model_path, env=env)

episodes = 10

vec_env = model.get_env()
obs = vec_env.reset()

for ep in range(episodes):
    obs = vec_env.reset()
    done = False
    while not done:
        vec_env.render()
        action, _states = model.predict(obs, deterministic = True)
        print(f"action: {action}")
        obs, reward, done, info = vec_env.step(action)
        time.sleep(1.0)

vec_env.close()
