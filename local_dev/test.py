#agent = ROSTemplateAgent()
from env import launch_env
import rospy
from solution import ROSTemplateAgent

env = launch_env()
template = ROSTemplateAgent()
done = True
while not rospy.is_shutdown():
    if done:
        obs = env.reset()
        template.publish_obs_to_agent(obs)
        template.agent.sleep()

    obs, rew, done, _ = env.step(template.agent.action)
    env.render()
    template.publish_obs_to_agent(obs)
    template.agent.sleep()