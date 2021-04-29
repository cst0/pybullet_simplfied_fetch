import gym
from randomagent import RandomAgent
import simple_grasping

def main():
    env = gym.make('SimpleFetch-v0')

    observation = env.reset()
    agent = RandomAgent()
    finish = False
    while not finish:
        action = agent.choose_action(observation)
        observation, _, finish, _ = env.step(action)

if __name__ == '__main__':
    main()
