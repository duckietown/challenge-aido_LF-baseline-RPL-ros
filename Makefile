

build:
	dts build_utils aido-container-build --use-branch daffy --ignore-dirty --ignore-untagged --push


submit-bea:
	dts challenges submit --impersonate 1639 --challenge 'aido-LF*' --retire-same-label

