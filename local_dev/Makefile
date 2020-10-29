SHELL=/bin/bash

user = $(USER)
u_user = $(shell id -u $(user))
g_user = $(shell id -g $(user))
current_dir = $(shell pwd)

trainer:
	docker build --build-arg user=$(user) -t trainer .

agent:
	cd ..; docker build -t agentdocker .

build: agent trainer

just_run:
	chmod 777 ./tensorboard && chmod -R 777 $(current_dir)/../rl_agent
	docker run -it --rm -v /etc/passwd:/etc/passwd -v $(current_dir)/tensorboard:/code/tensorboard -v $(shell dirname $(current_dir))/rl_agent:/code/rl_agent --user=$(u_user):$(g_user) --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --gpus all trainer

run: build just_run

submit:
	cd ..; dts challenges submit
