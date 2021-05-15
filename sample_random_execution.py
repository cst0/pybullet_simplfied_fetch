import gym
from randomagent import RandomAgent
from simple_grasping.standard_interfaces import *

def main():
    agent = RandomAgent()
    env = gym.make('SimpleFetch-v0')
    env.place_objects([
            Block.SMALL,
#            Block.MEDIUM,
#            Block.LARGE
        ])
    #env.place_objects([Block.SMALL], [Pose(1, 1, 1)])
    observation = env.reset()
    finish = False
    while not finish:
        action = agent.choose_action(observation)
        observation, _, finish, _ = env.step(action)
        if finish:
            print("Told to finish-- loop will now terminate")

if __name__ == '__main__':
    main()
