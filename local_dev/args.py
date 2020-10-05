import argparse

def _str2bool(string):
    string = string.lower()
    if string == "1" or string == "true":
        return True
    if string == "0" or string == "false":
        return False
    raise Exception("unkown boolean value '"+string+"'")

def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", type=_str2bool, default=True)
    parser.add_argument("--nb_episodes", type=int, default=500)
    return parser