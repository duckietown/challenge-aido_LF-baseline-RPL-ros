import argparse


def _str2bool(string):
    string = string.lower()
    if string == "1" or string == "true":
        return True
    if string == "0" or string == "false":
        return False
    raise Exception("unkown boolean value '" + string + "'")


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", type=_str2bool, default=True)
    parser.add_argument("--nb_episodes", type=int, default=500)
    return parser


import argparse


def get_ddpg_args_train():
    args = argparse.Namespace()
    args.seed = 123  # Sets Gym, PyTorch and Numpy seeds
    args.exploration_timesteps = 1e4
    args.start_timesteps = 2e4  # How many time steps purely random policy is run for
    args.eval_freq = 5e4  # How often (time steps) we evaluate
    args.max_timesteps = 2.5e5  # Max time steps to run environment for
    args.save_models = True  # Whether or not models are saved
    args.expl_noise = 0.1  # Std of Gaussian exploration noise
    args.batch_size = 100  # Batch size for both actor and critic
    args.discount = 0.99  # Discount factor
    args.tau = 0.005  # Target network update rate
    args.policy_noise = 0.2  # Noise added to target policy during critic update
    args.noise_clip = 0.5  # Range to clip target policy noise
    args.policy_freq = 2  # Frequency of delayed policy updates
    args.env_timesteps = 500  # Frequency of delayed policy updates
    args.replay_buffer_max_size = 1000000  # Maximum number of steps to keep in the replay buffer
    args.controller_timesteps = 0
    args.model_filename = f"DDPG_{args.seed}"

    return args
