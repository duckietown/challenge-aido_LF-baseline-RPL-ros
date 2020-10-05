# Local lightweight evaluation

This directory allows for local evaluation of policies, along with local in-simulator training.

To do so, simply modify the args.py, test.py, and train.py files with the required changes for your algorithm.
You can also modify the env's instanciation in env.py.

_setup is considered a private directory, only used to do the dispatching of the docker.

To do all this, the Dockerfile in this directory builds itself over the one in the parent repo. It assumes the parent 
dockerfile has been built as `agentdocker`.

To actually run this docker with visuals, one must pipe the host's display and display server: `docker run -it --rm     --user=$(id -u $USER):$(id -g $USER)     --env="DISPLAY"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --gpus all trainer
`

This is not needed for a train loop, however, and the train loop shouldn't call `env.render()`