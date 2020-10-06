#!/usr/bin/python3
from args import get_parser

import env

if get_parser().parse_args().test:
    import test
else:
    import train