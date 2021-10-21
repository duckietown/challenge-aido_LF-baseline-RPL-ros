# agent = ROSTemplateAgent()
import time

from env import launch_env
import rospy
from solution import ROSTemplateAgent

env = launch_env()

template = ROSTemplateAgent()
done = True


while not rospy.is_shutdown():
    if done:
        obs = env.reset()
        template.obs_to_agent(obs)
        template.agent.r.sleep()

    action = template.agent.action
    print(action)
    obs, rew, done, _ = env.step(action)
    env.render()
    template.obs_to_agent(obs)
    template.agent.r.sleep()
