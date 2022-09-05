from tensorforce.environments import Environment
import numpy as np

class BasketBallEnv(Environment):

    def __init__(self):
        super().__init__()

    def states(self):
        return dict(type='float', shape=(8,))

    def actions(self):
        return dict(type='int', num_values=4)

    # Optional: should only be defined if environment has a natural fixed
    # maximum episode length; otherwise specify maximum number of training
    # timesteps via Environment.create(..., max_episode_timesteps=???)
    # def max_episode_timesteps(self):
    #     return super().max_episode_timesteps()

    # Optional additional steps to close environment
    def close(self):
        super().close()

    def reset(self):
        state = np.random.random(size=(8,))
        return state

    def execute(self, actions):
        next_state = np.random.random(size=(8,))
        terminal = False  # Always False if no "natural" terminal state
        reward = np.random.random()
        return next_state, terminal, reward

if __name__ == "__main__":
    from tensorforce.agents import TensorforceAgent
    # Goal is to understand how the evn and agent work and what do they need as parameters
    env = Environment.create(environment=BasketBallEnv)
    agent = TensorforceAgent.create(environment=env)