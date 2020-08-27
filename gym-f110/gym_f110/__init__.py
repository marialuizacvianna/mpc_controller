from gym.envs.registration import register

register(id='f110-v0',
    entry_point='gym_f110.envs:F110Env',
)
