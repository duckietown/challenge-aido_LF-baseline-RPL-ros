# Local lightweight evaluation

This directory allows for local evaluation of policies, along with local in-simulator training.

To do so, simply modify the args.py, test.py, and train.py files with the required changes for your algorithm.
You can also modify the env's instanciation in env.py.

`_setup` is considered a private directory, only used to do the dispatching of the docker.

To do all this, the Dockerfile in this directory builds itself over the one in the parent repo. It assumes the parent 
dockerfile has been built as `agentdocker`.

You can use `make run` to build all the necessary docker layers and then launch the training or testing (depending on what you
asked for in `args.py`). The test script will bind itself to your X server and output the simulator's screen directly on your
computer.

This is not needed for a train loop, however, and the train loop shouldn't call `env.render()`. But if you want to view the 
simulator's render during training, you can still call it. It will act just like it would if you were running these scripts 
locally on your host.

# Submission

The contents of `local_dev` should not be submitted to AIDO: they are only in this repo for training and testing.

The actual contents of your submission are in the other directories of this repo.