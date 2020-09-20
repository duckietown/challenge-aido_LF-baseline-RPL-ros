
AIDO_REGISTRY ?= docker.io
PIP_INDEX_URL ?= https://pypi.org/simple

build_options=\
 	--build-arg AIDO_REGISTRY=$(AIDO_REGISTRY)\
 	--build-arg PIP_INDEX_URL=$(PIP_INDEX_URL)

repo=challenge-aido_lf-template-ros
branch=$(shell git rev-parse --abbrev-ref HEAD)
arch=amd64
branch=daffy
tag=$(AIDO_REGISTRY)/duckietown/$(repo):$(branch)-$(arch)


update-reqs:
	pur --index-url $(PIP_INDEX_URL) -r requirements.txt -f -m '*' -o requirements.resolved
	aido-update-reqs requirements.resolved

build: update-reqs
	docker build --pull -t $(tag) --build-arg ARCH=$(arch) $(build_options) .

build-no-cache: update-reqs
	docker build --pull  -t $(tag) --build-arg ARCH=$(arch) $(build_options) --no-cache .

push: build
	docker push $(tag)

submit-bea:
	dts challenges submit --impersonate 1639 --challenge all --retire-same-label