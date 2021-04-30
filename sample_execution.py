import gym
from randomagent import RandomAgent

def main():
    agent = RandomAgent()
    env = gym.make('SimpleFetch-v0')
    observation = env.reset()
    finish = False
    while not finish:
        action = agent.choose_action(observation)
        observation, _, finish, _ = env.step(action)
        if finish:
            print("Told to finish-- loop will now terminate")

if __name__ == '__main__':
    main()
