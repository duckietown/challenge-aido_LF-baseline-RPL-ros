FROM agentdocker

# inspired by https://github.com/duckietown/challenge-aido_LF-baseline-duckietown/blob/daffy/1_develop/DockerfileSim
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display
RUN apt-get update && apt install freeglut3-dev xvfb -y && git clone https://github.com/duckietown/gym-duckietown.git && cd gym-duckietown && git checkout daffy && \
    pip3 install -e .

# Set up our own user inside the docker. Inspired by http://wiki.ros.org/docker/Tutorials/GUI
ARG user
ENV USERNAME $user
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

COPY _setup .
COPY . .
RUN rm -rf _setup && rm Dockerfile

RUN pip3 uninstall dataclasses -y

CMD ["python3", "entrypoint.py"]