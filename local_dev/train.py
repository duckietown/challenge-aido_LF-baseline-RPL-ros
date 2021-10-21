import time

from tensorboardX import SummaryWriter
import numpy as np
from args import get_ddpg_args_train
from utils import seed
from env import launch_env
from utils import *
from rosagent import ROSAgent
import torch
import os


class BurnInRLAgent:
    def predict(self, obs):
        return np.array([0, 0])


class RandomAgent:
    def predict(self, obs):
        return np.random.uniform(-1, 1, (2,))


args = get_ddpg_args_train()
writer = SummaryWriter(f"./tensorboard/{args.model_filename}")

seed(args.seed)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

env = launch_env()

state_dim = env.observation_space.shape
action_dim = env.action_space.shape[0]
max_action = float(env.action_space.high[0])

# env = ActionWrapper(env)
# env = DtRewardWrapper(env)
# env = SteeringToWheelVelWrapper(env)
# env = MotionBlurWrapper(env)
rosagent = ROSAgent(dont_init_rl=True)
ddpg_agent = rosagent.rl_policy
rosagent.rl_policy = BurnInRLAgent()
burnin_agent = BurnInRLAgent()
random_agent = RandomAgent()


def send_obs(obs):
    rosagent._publish_img(obs, wait_for_callback=True)


def sample_action(obs):
    rl_action_clean = None

    if total_timesteps < args.exploration_timesteps:
        print("BURN IN")
        rl_action = burnin_agent.predict(obs)
    elif total_timesteps < args.start_timesteps:
        print("RANDOM")
        rl_action = random_agent.predict(obs)
    else:
        print("RL")
        rl_action = ddpg_agent.predict(obs)
        rl_action_clean = rl_action.copy()

        if args.expl_noise != 0:
            noise = np.random.normal(0, args.expl_noise, size=env.action_space.shape[0])
            rl_action += noise.clip(env.action_space.low, env.action_space.high)

    rl_action_scaled = rl_action  # 0.5 * (self.rl_action + 1)
    action = rosagent.action + rl_action_scaled
    return action, rl_action, rl_action_clean


# Initialize policy's writer
rosagent.rl_policy.writer = writer

replay_buffer = ReplayBuffer(args.replay_buffer_max_size)
total_timesteps = 0
timesteps_since_eval = 0
episode_num = 0
done = True
episode_reward = None
reward_diff = None

env_counter = 0
obs = env.reset()

while rosagent.action is None:
    send_obs(obs)

while total_timesteps < args.max_timesteps:
    print(total_timesteps)

    start = time.time()
    if done:
        if total_timesteps != 0:
            ddpg_agent.train(
                replay_buffer,
                episode_timesteps,
                args.batch_size,
                args.discount,
                args.tau,
                only_critic=total_timesteps < args.start_timesteps,
            )

        # Evaluate episode
        if timesteps_since_eval >= args.eval_freq:
            timesteps_since_eval %= args.eval_freq
            writer.add_scalar(
                "eval.reward", evaluate_policy(env, rosagent, ddpg_agent, writer), total_timesteps
            )

            if args.save_models:
                ddpg_agent.save(f"{args.model_filename}_{total_timesteps}", directory="./rl_agent/weights")

                """
                try:
                    shutil.copytree("/duckietown/catkin_ws/results/tensorboard",
                                    "/duckietown/catkin_ws/results/tensorboard_{}".format(total_timesteps))
                except Exception as e:
                    traceback.print_exc()
                """

        # Reset environment
        env_counter += 1
        obs = env.reset()
        send_obs(obs)

        if episode_reward is not None:
            writer.add_scalar("train.episode_reward", episode_reward, total_timesteps)
            writer.add_scalar("train.episode_reward_diff", reward_diff, total_timesteps)
        done = False
        episode_reward = 0
        reward_diff = 0
        episode_timesteps = 0
        episode_num += 1

    controller_action, rl_action, rl_action_clean = sample_action(obs)

    # Perform action
    # rescale action from -1 and 1 to 0 and 1
    new_obs, reward, done, _ = env.step(rosagent.action)
    writer.add_scalar("train.reward", reward, total_timesteps)
    critic_reward = ddpg_agent.critize(obs, rl_action)
    writer.add_scalar("train.critic_reward", critic_reward, total_timesteps)
    reward_diff += abs(reward - critic_reward)
    send_obs(new_obs)

    writer.add_scalar("train.controller.action.absvl", np.abs(controller_action[0]), total_timesteps)
    writer.add_scalar("train.controller.action.absvr", np.abs(controller_action[1]), total_timesteps)
    writer.add_scalar("train.controller.action.vl", controller_action[0], total_timesteps)
    writer.add_scalar("train.controller.action.vr", controller_action[1], total_timesteps)

    if total_timesteps < args.start_timesteps:
        writer.add_scalar("train.rl.action.absvl", np.abs(rl_action[0]), total_timesteps)
        writer.add_scalar("train.rl.action.absvr", np.abs(rl_action[1]), total_timesteps)
        writer.add_scalar("train.rl.action.vl", rl_action[0], total_timesteps)
        writer.add_scalar("train.rl.action.vr", rl_action[1], total_timesteps)
        # writer.add_scalar("train.rl.action.scaled_vl", rosagent.rl_action_scaled[0], total_timesteps)
        # writer.add_scalar("train.rl.action.scaled_vr", rosagent.rl_action_scaled[1], total_timesteps)
    else:
        writer.add_scalar("train.rl.action.absvl", np.abs(rl_action_clean[0]), total_timesteps)
        writer.add_scalar("train.rl.action.absvr", np.abs(rl_action_clean[1]), total_timesteps)
        writer.add_scalar("train.rl.action.vl", rl_action_clean[0], total_timesteps)
        writer.add_scalar("train.rl.action.vr", rl_action_clean[1], total_timesteps)

    if episode_timesteps >= args.env_timesteps:
        done = True

    done_bool = 0 if episode_timesteps + 1 == args.env_timesteps else float(done)
    episode_reward += reward

    # Store data in replay buffer
    replay_buffer.add(obs, new_obs, controller_action, rl_action, reward, done_bool)

    episode_timesteps += 1
    total_timesteps += 1
    rosagent.total_timesteps = total_timesteps
    timesteps_since_eval += 1

    obs = new_obs

if args.save_models:
    print("SAVED FINAL MODEL")
    ddpg_agent.save(args.model_filename, directory="./rl_agent/weights")
    print("YOU CAN NOW SAFELY KILL THIS PROCESS")
exit()
