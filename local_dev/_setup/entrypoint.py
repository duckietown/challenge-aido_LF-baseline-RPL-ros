#!/usr/bin/python3
import re
import sys
import threading

from args import get_parser

import env
import os

import subprocess

with open("./run_and_start.sh", "r") as f:
    command = re.sub(r'python.*?\n', '', f.read())  # TODO is this really the best way to be compliant with the fork's parent?

def roscore():
    proc = subprocess.Popen(command, shell=True, executable='/bin/bash', universal_newlines=True)
    proc.wait()
    print("\n\n!!!\nROSCORE CRASHED. EXITING NOW!\n!!!\n\n", file=sys.stderr)
    exit(-128)

thread = threading.Thread(target=roscore)
thread.start()

parsed = get_parser().parse_args()
print(parsed)

if get_parser().parse_args().test:
    os.environ["SOLUTION_TEST"] = "1"
    print("\n\n\n\n\n\n\n\nTESTING\n\n\n\n\n\n\n\n")
    import test
else:
    print("\n\n\n\n\n\n\n\nTRAINING\n\n\n\n\n\n\n\n")
    os.environ["SOLUTION_TEST"] = "0"
    import train