import gym
from sequentialagent import SequentialAgent
from simple_grasping.standard_interfaces import Block, Pose

def main():
    agent = SequentialAgent()
    env = gym.make('SimpleFetch-v0')
    env.place_objects([Block.LARGE], [Pose(0,0,0)])
    observation = env.reset()
    finish = False
    while not finish:
        action = agent.choose_action(observation)
        observation, _, finish, _ = env.step(action)
        if finish:
            print("Told to finish-- loop will now terminate")

if __name__ == '__main__':
    main()
