import functools
import operator

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .object_wrappers import imgWrapper

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# Implementation of Deep Deterministic Policy Gradients (DDPG)
# Paper: https://arxiv.org/abs/1509.02971

class ActorCNN(nn.Module):
    def __init__(self, action_dim, max_action):
        super(ActorCNN, self).__init__()

        # ONLY TRU IN CASE OF DUCKIETOWN:
        flat_size = 32 * 9 * 14

        self.lr = nn.LeakyReLU()
        self.tanh = nn.Tanh()
        self.sigm = nn.Sigmoid()

        self.conv1 = nn.Conv2d(3, 32, 8, stride=2)
        self.conv2 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv3 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv4 = nn.Conv2d(32, 32, 4, stride=1)

        self.bn1 = nn.BatchNorm2d(32)
        self.bn2 = nn.BatchNorm2d(32)
        self.bn3 = nn.BatchNorm2d(32)
        self.bn4 = nn.BatchNorm2d(32)

        self.dropout = nn.Dropout(.5)  # remove it

        self.lin1 = nn.Linear(flat_size, 512)
        self.lin2 = nn.Linear(512, action_dim, bias=False)
        self.lin2.weight = torch.nn.Parameter(torch.zeros(self.lin2.weight.shape))

        self.max_action = max_action

    def forward(self, x):
        x = self.bn1(self.lr(self.conv1(x)))
        x = self.bn2(self.lr(self.conv2(x)))
        x = self.bn3(self.lr(self.conv3(x)))
        x = self.bn4(self.lr(self.conv4(x)))
        x = x.view(x.size(0), -1)  # flatten
        # x = self.dropout(x)
        x = self.lr(self.lin1(x))

        # this is the vanilla implementation
        # but we're using a slightly different one
        # x = self.max_action * self.tanh(self.lin2(x))

        # because we don't want our duckie to go backwards
        x = self.lin2(x)
        # switch to tanh
        x[:, 0] = self.tanh(
            x[:, 0])  # self.max_action * self.sigm(x[:, 0]) # because we don't want the duckie to go backwards
        x[:, 1] = self.tanh(x[:, 1])

        return x


class CriticCNN(nn.Module):
    def __init__(self, action_dim):
        super(CriticCNN, self).__init__()

        flat_size = 32 * 9 * 14

        self.lr = nn.LeakyReLU()

        self.conv1 = nn.Conv2d(3, 32, 8, stride=2)
        self.conv2 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv3 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv4 = nn.Conv2d(32, 32, 4, stride=1)

        self.bn1 = nn.BatchNorm2d(32)
        self.bn2 = nn.BatchNorm2d(32)
        self.bn3 = nn.BatchNorm2d(32)
        self.bn4 = nn.BatchNorm2d(32)

        self.dropout = nn.Dropout(.5)  # remove it

        self.lin1 = nn.Linear(flat_size, 256)
        self.lin2 = nn.Linear(256 + action_dim, 128)
        self.lin3 = nn.Linear(128, 1)

    def forward(self, states, actions):
        x = self.bn1(self.lr(self.conv1(states)))
        x = self.bn2(self.lr(self.conv2(x)))
        x = self.bn3(self.lr(self.conv3(x)))
        x = self.bn4(self.lr(self.conv4(x)))
        x = x.view(x.size(0), -1)  # flatten
        x = self.lr(self.lin1(x))
        x = self.lr(self.lin2(torch.cat([x, actions], 1)))  # c
        x = self.lin3(x)

        return x


class DDPG(object):
    def __init__(self, state_dim=(480, 640, 3), action_dim=2, max_action=1.0, net_type="cnn", critic_chkp=None, writer=None):
        super(DDPG, self).__init__()
        assert net_type in ["cnn", "pid"]

        self.state_dim = state_dim
        self.flat = False
        self.actor = ActorCNN(action_dim, max_action).to(device)
        self.actor_target = ActorCNN(action_dim, max_action).to(device)

        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=1e-6)

        self.critic = CriticCNN(action_dim).to(device)
        if critic_chkp:
            self.critic.load_state_dict(torch.load(critic_chkp))

        self.critic_target = CriticCNN(action_dim).to(device)

        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters())

        self.writer = writer

    def predict(self, state):
        # just making sure the state has the correct format, otherwise the prediction doesn't work
        state = imgWrapper(state)
        assert state.shape[0] == 3
        v = None  # torch.FloatTensor(v).to(device)
        omega = None  # torch.FloatTensor(omega).to(device)

        if self.flat:
            state = torch.FloatTensor(state.reshape(1, -1)).to(device)
        else:
            state = torch.FloatTensor(np.expand_dims(state, axis=0)).to(device)

        return self.actor(state).cpu().data.numpy().flatten()

    def critize(self, state, action):
        state = imgWrapper(state)
        assert state.shape[0] == 3

        state = torch.FloatTensor(np.expand_dims(state, axis=0)).to(device)
        action = torch.FloatTensor(np.expand_dims(action, axis=0)).to(device)

        return self.critic(state, action).cpu().data.numpy().flatten()

    def train(self, replay_buffer, iterations, batch_size=64, discount=0.99, tau=0.001, only_critic=False):
        for it in range(iterations):

            # Sample replay buffer
            sample = replay_buffer.sample(batch_size, flat=self.flat)
            state = torch.FloatTensor(sample["state"]).to(device)
            action = torch.FloatTensor(sample["action"]).to(device)
            next_state = torch.FloatTensor(sample["next_state"]).to(device)
            done = torch.FloatTensor(1 - sample["done"]).to(device)
            reward = torch.FloatTensor(sample["reward"]).to(device)

            # Compute the target Q value
            target_Q = self.critic_target(next_state, self.actor_target(next_state))
            target_Q = reward + (done * discount * target_Q).detach()

            # Get current Q estimate
            current_Q = self.critic(state, action)
            self.writer.add_scalar("train.rl.critic.reward", torch.mean(current_Q).cpu().data.numpy(), it)

            # Compute critic loss
            critic_loss = F.mse_loss(current_Q, target_Q)
            self.writer.add_scalar("train.rl.critic.loss", critic_loss.cpu().data.numpy(), it)

            # Optimize the critic
            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()

            # Compute actor loss
            if not only_critic:
                actor_loss = -self.critic(state, self.actor(state)).mean()
                self.writer.add_scalar("train.rl.actor.loss", actor_loss.cpu().data.numpy(), it)

                # Optimize the actor
                self.actor_optimizer.zero_grad()
                actor_loss.backward()
                self.actor_optimizer.step()

                for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                    target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)

            # Update the frozen target models
            for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)

    def save(self, filename, directory):
        torch.save(self.actor.state_dict(), '{}/{}_actor.pth'.format(directory, filename))
        torch.save(self.critic.state_dict(), '{}/{}_critic.pth'.format(directory, filename))

    def load(self, filename, directory):
        self.actor.load_state_dict(torch.load('{}/{}_actor.pth'.format(directory, filename), map_location=device))
        self.critic.load_state_dict(torch.load('{}/{}_critic.pth'.format(directory, filename), map_location=device))