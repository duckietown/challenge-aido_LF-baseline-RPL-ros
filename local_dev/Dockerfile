FROM agentdocker
# inspired by https://github.com/duckietown/challenge-aido_LF-baseline-duckietown/blob/daffy/1_develop/DockerfileSim
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display
RUN apt-get update && apt install freeglut3-dev xvfb -y && git clone https://github.com/duckietown/gym-duckietown.git && cd gym-duckietown && git checkout daffy && \
    pip3 install -e . && mkdir -p /data/config && git clone https://github.com/duckietown/duckiefleet /data/config
COPY _setup .
COPY . .
RUN rm -rf _setup && rm Dockerfile && mkdir /.ros && chmod -R 777 /.ros
CMD ["python3", "entrypoint.py"]