trainer:
	docker build -t trainer .

agent:
	cd ..; docker build -t agentdocker .

build: agent trainer

run:
	docker run -it --rm     --user=$(id -u $USER):$(id -g $USER)     --env="DISPLAY"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --gpus all trainer

submit:
	cd ..; dts challenges submit
