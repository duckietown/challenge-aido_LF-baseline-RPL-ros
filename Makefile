

build:
	dts build_utils aido-container-build --use-branch daffy --ignore-dirty --ignore-untagged

push: build
	dts build_utils aido-container-push --use-branch daffy

submit-bea:
	dts challenges submit --impersonate 1639 --challenge all --retire-same-label

