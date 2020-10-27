import numpy as np

def cropTransposeWrapper(state):
    from PIL import Image

    return np.array(Image.fromarray(state.astype(np.uint8)).resize((120, 160))).transpose(2, 0, 1)

def normalizeWrapper(state):
    state_lo=np.min(state)
    state_hi=np.max(state)
    return (state - state_lo) / (state_hi - state_lo)

def imgWrapper(state):
    from PIL import Image
    state = np.array(Image.fromarray(state.astype(np.uint8)).resize((120, 160)))

    state = state.astype(np.float64)
    state_lo=np.min(state)
    state_hi=np.max(state)
    state = (state - state_lo) / (state_hi - state_lo)

    return state.transpose(2, 0, 1)


def dtRewardWrapper(reward):
    if reward == -1000:
        reward = -10
    elif reward > 0:
        reward += 10
    else:
        reward += 4

    return reward


# this is needed because at max speed the duckie can't turn anymore
def actionWrapper(action):
    action = [action[0] * 0.8, action[1]]
    return action


def steeringToWheelVelWrapper(action):
    """
    Converts policy that was trained with [velocity|heading] actions to
    [wheelvel_left|wheelvel_right] to comply with AIDO evaluation format
    """

    gain=1.0,
    trim=0.0,
    radius=0.0318,
    k=27.0,
    limit=1.0,
    wheel_dist=0.102


    vel, angle = action

    # assuming same motor constants k for both motors
    k_r = k
    k_l = k

    # adjusting k by gain and trim
    k_r_inv = (gain + trim) / k_r
    k_l_inv = (gain - trim) / k_l

    omega_r = (vel + 0.5 * angle * wheel_dist) / radius
    omega_l = (vel - 0.5 * angle * wheel_dist) / radius

    # conversion from motor rotation rate to duty cycle
    u_r = omega_r * k_r_inv
    u_l = omega_l * k_l_inv

    # limiting output to limit, which is 1.0 for the duckiebot
    u_r_limited = max(min(u_r, limit), -limit)
    u_l_limited = max(min(u_l, limit), -limit)

    vels = np.array([u_l_limited, u_r_limited])
