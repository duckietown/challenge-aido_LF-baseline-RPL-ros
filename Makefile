

base_tag=duckietown/challenge-aido1_lf1-template-ros-deps

build-base:
	docker build -t $(base_tag) -f Dockerfile.deps .

push-base:
	docker push $(base_tag)


submit: submit-LF submit-LFV

submit-LF:
	dts challenges submit --challenge aido1_LF1_r3-v3

submit-LFV:
	dts challenges submit --challenge aido1_LFV_r1-v3
