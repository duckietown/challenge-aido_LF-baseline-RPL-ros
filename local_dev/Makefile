SHELL=/bin/bash

user = $(USER)
u_user = $(shell id -u $(user))
g_user = $(shell id -g $(user))

trainer:
	docker build --build-arg user=$(user) -t trainer .

agent:
	cd ..; docker build -t agentdocker .

build: agent trainer

just_run:
	docker run -it --rm -v /etc/passwd:/etc/passwd --user=$(u_user):$(g_user) --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --gpus all trainer

run: build just_run

submit:
	cd ..; dts challenges submit
