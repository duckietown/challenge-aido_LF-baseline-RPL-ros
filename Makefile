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

test-data1-direct:
	./solution.py < test_data/in1.json > test_data/out1.json

test-data1-docker:
	docker run -i $(tag) < test_data/in1.json > test_data/out1.json

