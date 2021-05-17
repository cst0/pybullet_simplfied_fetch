import time
import gym
# from sequentialagent import SequentialAgent
from simple_grasping.standard_interfaces import *
from time import sleep
from agents import ActorCriticPolicy
import matplotlib.pyplot as plt 
def main():
    episode = 0
    time_step = 0
    reward_sum = 0
    done_arr = []
    curr_task_completion_array = []
    reward_arr = []
    avg_reward = []
    timestep_arr = []
    episode_arr = []
    done = False

    agent = ActorCriticPolicy(3, 9)
    env = gym.make('SimpleFetch-v0')
    env.place_objects([
            Block.SMALL,
            Block.MEDIUM,
            Block.LARGE
        ])
    obs = env.reset()
    print(obs)
    # sleep(5)
    while True:
        a = agent.select_action(obs)

        # print("action at 0: ", a)
        # action = agent.choose_action(observation)
        obs, reward, done, _ = env.step(a)
        time_step += 1
        reward_sum += reward
        # print("done: ", done)
        agent.set_rewards(reward)

        if time_step > 50 or done:
            # print("time step is: ", time_step)
            # finish agent
            if done:
                done_arr.append(1)
                curr_task_completion_array.append(1)
                print("Done!")
                sleep(4)
            elif time_step > 50:
                done_arr.append(0)
                curr_task_completion_array.append(0)

            print("\n\nfinished episode = " + str(episode) + " with " + str(reward_sum) + "\n")

            reward_arr.append(reward_sum)
            avg_reward.append(np.mean(reward_arr[-40:]))
            timestep_arr.append(time_step)

            done = 1
            agent.finish_episode()

            episode += 1
            time_step = 0

            env.reset()
            reward_sum = 0

            # env_flag = 0

            # if not is_final_env:
            #     env_flag = check_training_done_callback(reward_arr, done_arr)

            # quit after some number of episodes
            if episode > 1000:
                agent.save_model(0, 0, 1)
                break
    
    plt.plot(avg_reward)
    plt.savefig("reward.png")

if __name__ == '__main__':
    main()
