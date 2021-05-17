import gym
from sequentialagent import SequentialAgent
from simple_grasping.standard_interfaces import *

def main():
    agent = SequentialAgent()
    env = gym.make('SimpleFetch-v0')
    env.place_objects([
            Block.SMALL,
            Block.MEDIUM,
            Block.LARGE
        ])
    env.reset()
    observation = env.worldstate
    finish = False
    while True:
        action = agent.choose_action(observation)
        observation, _, finish, _ = env.step(action)
        observation = env.worldstate
        if finish:
            print("Told to finish-- loop will now terminate")

if __name__ == '__main__':
    main()
