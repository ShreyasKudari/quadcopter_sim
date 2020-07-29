import gym
import gym_drone
env = gym.make('drone-v0')
env.reset()
for _ in range(1000000):
    #env.render()
    env.step(env.action_space.sample()) # take a random action
env.close()
