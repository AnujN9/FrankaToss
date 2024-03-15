import gymnasium as gym
import panda_gym
import time
import numpy as np
from stable_baselines3 import SAC
import csv

# env = gym.make("PandaThrow-v3", render_mode="human", control_type="joints")
env = gym.make("PandaThrow-v3", render_mode="human")
env.reset()

# models_dir = "models/SAC_1710493867"
# models_dir = "models/SAC_latest"
models_dir = "models/SAC_1710487905"
logdir = "logs"

# 590000 was working well for 905
model_path = f"{models_dir}/590000.zip"
model = SAC.load(model_path, env=env)

episodes = 10

vec_env = model.get_env()
obs = vec_env.reset()
count = 0
first = 0
step = 0

with open('ee_pose.csv', 'w') as file:
    writer = csv.writer(file)
    for ep in range(episodes):
        if first < 1:
            writer.writerow([f"{obs['observation'][0,0]},{obs['observation'][0,1]},"
                            +f"{obs['observation'][0,2]},{obs['observation'][0,3]},"
                            +f"{obs['observation'][0,4]},{obs['observation'][0,5]},"
                            +f"{obs['observation'][0,6]}"])
            first += 1
        obs = vec_env.reset()
        done = False
        print("Iterate")
        print(f"{obs['observation'][0]}")
        while not done:
            vec_env.render()
            time.sleep(0.8)
            print(step)
            action, _states = model.predict(obs, deterministic = True)
            obs, reward, done, info = vec_env.step(action)
            step += 1
            if count < 1:
                print(f"{obs['observation'][0]}")   
                writer.writerow([f"{obs['observation'][0,0]},{obs['observation'][0,1]},"
                            +f"{obs['observation'][0,2]},{obs['observation'][0,3]},"
                            +f"{obs['observation'][0,4]},{obs['observation'][0,5]},"
                            +f"{obs['observation'][0,6]}"])
        count += 1
    vec_env.close()
