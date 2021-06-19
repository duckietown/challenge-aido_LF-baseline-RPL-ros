
#!/bin/bash

# This script will install pytorch, torchvision, torchtext and spacy on nano.
# If you have any of these installed already on your machine, you can skip those.

apt-get -y update

#Installing PyTorch
#For latest PyTorch refer original Nvidia Jetson Nano thread - https://devtalk.nvidia.com/default/topic/1049071/jetson-nano/pytorch-for-jetson-nano/.
wget https://nvidia.box.com/shared/static/wa34qwrwtk9njtyarwt5nvo6imenfy26.whl -O torch-1.7.0-cp36-cp36m-linux_aarch64.whl
apt-get install -y libopenblas-base
pip3 install Cython
pip3 install numpy
pip3 install torch-1.7.0-cp36-cp36m-linux_aarch64.whl

#Installing torchvision
#For latest torchvision refer original Nvidia Jetson Nano thread - https://devtalk.nvidia.com/default/topic/1049071/jetson-nano/pytorch-for-jetson-nano/.
apt-get install -y  libjpeg-dev zlib1g-dev
git clone --branch v0.5.0 https://github.com/pytorch/vision torchvision   # see below for version of torchvision to download
cd torchvision
python3 setup.py install
cd ../  # attempting to load torchvision from build dir will result in import error


echo "done installing PyTorch, torchvision"
