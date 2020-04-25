
#%% 
import gym

#%%
# MountainCar-v0
# CartPole-v0
# * Hopper-v1
# MsPacman-v0
# SpaceInvaders-v0
# MountainCarContinuous-v0

env = gym.make('MountainCarContinuous-v0')

for i_episode in range(20):
    observation = env.reset()
    for t in range(100):
        env.render()
        print(observation)
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break




env.close()





#%%
