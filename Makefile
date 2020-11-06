

build: 
	dts build_utils aido-container-build --ignore-dirty --ignore-untagged

push: build
	dts build_utils aido-container-push

submit-bea:
	dts challenges submit --impersonate 1639 --challenge all --retire-same-label

