import json
from pathlib import Path

import torch
import numpy as np
import random
from rl_agent.object_wrappers import normalizeWrapper, cropTransposeWrapper


def seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)


# Simple replay buffer
class ReplayBuffer(object):
    def __init__(self, max_size=10000, replay_buffer_path=None, replay_buffer_size=None):
        self.storage = []
        self.max_size = max_size

    # Expects tuples of (state, next_state, action, reward, done)
    def add(self, state, next_state, controller_action, action, reward, done):
        state = cropTransposeWrapper(state)
        next_state = cropTransposeWrapper(next_state)
        if len(self.storage) < self.max_size:
            self.storage.append((state, next_state, controller_action, action, reward, done))
        else:
            # Remove random element in the memory beforea adding a new one
            i = random.randrange(len(self.storage))
            self.storage.pop(i)
            self.storage.append((state, next_state, controller_action, action, reward, done))

    def sample(self, batch_size=100, flat=True):
        ind = np.random.randint(0, len(self.storage), size=batch_size)
        states, next_states, controller_actions, actions, rewards, dones = [], [], [], [], [], []

        for i in ind:
            state, next_state, controller_action, action, reward, done = self.storage[i]
            state = normalizeWrapper(state)
            next_state = normalizeWrapper(next_state)
            if flat:
                states.append(np.array(state, copy=False).flatten())
                next_states.append(np.array(next_state, copy=False).flatten())
            else:
                states.append(np.array(state, copy=False))
                next_states.append(np.array(next_state, copy=False))
            controller_actions.append(np.array(controller_action, copy=False))
            actions.append(np.array(action, copy=False))
            rewards.append(np.array(reward, copy=False))
            dones.append(np.array(done, copy=False))

        # state_sample, action_sample, next_state_sample, reward_sample, done_sample
        return {
            "state": np.stack(states),
            "next_state": np.stack(next_states),
            "controller_action": np.stack(controller_actions),
            "action": np.stack(actions),
            "reward": np.stack(rewards).reshape(-1, 1),
            "done": np.stack(dones).reshape(-1, 1),
        }


def evaluate_policy(env, ros_agent, ddpg_agent, writer, eval_episodes=10, max_timesteps=500):
    old_rectifier = ros_agent.rl_policy
    ros_agent.rl_policy = ddpg_agent

    ddpg_agent.actor.eval()
    ddpg_agent.critic.eval()

    avg_reward = 0.0
    for i in range(eval_episodes):
        obs = env.reset()
        done = False
        step = 0
        while not done and step < max_timesteps:
            ros_agent._publish_img(obs, wait_for_callback=True)

            writer.add_scalar("eval.controller.action.absvr", np.abs(ros_agent.action[1]), step)
            writer.add_scalar("eval.rl.action.absvl", np.abs(ros_agent.rl_action[0]), step)
            writer.add_scalar("eval.controller.action.absvl", np.abs(ros_agent.action[0]), step)
            writer.add_scalar("eval.rl.action.absvr", np.abs(ros_agent.rl_action[1]), step)
            writer.add_scalar("eval.controller.action.vl", ros_agent.action[0], step)
            writer.add_scalar("eval.controller.action.vr", ros_agent.action[1], step)
            writer.add_scalar("eval.rl.action.vl", ros_agent.rl_action[0], step)
            writer.add_scalar("eval.rl.action.vr", ros_agent.rl_action[1], step)

            obs, reward, done, _ = env.step(ros_agent.action)
            avg_reward += reward
            step += 1

    avg_reward /= eval_episodes

    ddpg_agent.actor.train()
    ddpg_agent.critic.train()

    ros_agent.rl_policy = old_rectifier

    return avg_reward
