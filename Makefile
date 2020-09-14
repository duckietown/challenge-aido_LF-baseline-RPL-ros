repo=challenge-aido_lf-template-ros
branch=$(shell git rev-parse --abbrev-ref HEAD)
arch=amd64
tag=duckietown/$(repo):$(branch)-$(arch)

build:
	docker build --pull -t $(tag) --build-arg ARCH=$(arch) .

build-no-cache:
	docker build --pull  -t $(tag) --build-arg ARCH=$(arch)--no-cache .

push: build
	docker push $(tag)
