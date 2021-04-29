from gym.envs.registration import register

register(
    id='SimpleFetch-v0',
    entry_point='simple_grasping.envs:SimpleFetchEnv'
)
