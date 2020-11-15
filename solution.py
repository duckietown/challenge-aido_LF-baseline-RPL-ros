#!/usr/bin/env python3

import io
import time

import numpy as np
from aido_schemas import (Context, Duckiebot1Commands, Duckiebot1Observations, EpisodeStart,
                          GetCommands, LEDSCommands, protocol_agent_duckiebot1, PWMCommands, RGB, wrap_direct)
from PIL import Image

from rosagent import ROSAgent
import torch
import os

class ROSTemplateAgent:
    def __init__(self):
        # Now, initialize the ROS stuff here:
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # roslaunch_path = os.path.join(os.getcwd(), "template.launch")
        # self.launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_path])
        # self.launch.start()

        # Start the ROSAgent, which handles publishing images and subscribing to action
        pass

    def init(self, context: Context):
        context.info("init()")
        self.check_gpu_available(context)
        self.agent = ROSAgent()

    def check_gpu_available(self, context: Context):
        available = torch.cuda.is_available()
        req = os.environ.get('AIDO_REQUIRE_GPU', None)
        context.info(f'torch.cuda.is_available = {available!r} AIDO_REQUIRE_GPU = {req!r}')
        context.info('init()')
        if available:
            i = torch.cuda.current_device()
            count = torch.cuda.device_count()
            name = torch.cuda.get_device_name(i)
            context.info(f'device {i} of {count}; name = {name!r}')
        else:
            if req is not None:
                msg = 'I need a GPU; bailing.'
                context.error(msg)
                raise Exception(msg)

    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info("Starting episode %s." % data)

    def on_received_observations(self, context: Context, data: Duckiebot1Observations):
        jpg_data = data.camera.jpg_data
        obs = jpg2rgb(jpg_data)
        self.obs_to_agent(obs)

    def obs_to_agent(self, obs):
        self.agent._publish_img(obs)
        self.agent._publish_info()

    def on_received_get_commands(self, context: Context, data: GetCommands):
        if not self.agent.initialized:
            pwm_left, pwm_right = [0,0]
        else:
            # TODO: let's use a queue here. Performance suffers otherwise.
            # What you should do is: *get the last command*, if available
            # otherwise, wait for one command.
            while not self.agent.updated:
                time.sleep(0.01)

            pwm_left, pwm_right = self.agent.action
            self.agent.updated = False

        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = Duckiebot1Commands(pwm_commands, led_commands)

        context.write("commands", commands)

    def finish(self, context):
        context.info("finish()")


def jpg2rgb(image_data):
    """ Reads JPG bytes as RGB"""
    im = Image.open(io.BytesIO(image_data))
    im = im.convert("RGB")
    data = np.array(im)
    assert data.ndim == 3
    assert data.dtype == np.uint8
    return data


if __name__ == "__main__":
    agent = ROSTemplateAgent()
    wrap_direct(agent, protocol_agent_duckiebot1)