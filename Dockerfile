# Definition of Submission container

ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}

FROM duckietown/dt-car-interface:${BASE_TAG} AS dt-car-interface

FROM duckietown/dt-core:${BASE_TAG}

COPY --from=dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface ./${CATKIN_WS_DIR}/src/dt-car-interface

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python-tk \
         python-wheel \
         python-pip && \
     rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt
COPY requirements.txt .
RUN pip install -r requirements.txt

# For ROS Agent - Need to upgrade Pillow for Old ROS stack
RUN pip install pillow --user --upgrade

# let's copy all our solution files to our workspace
# if you have more file use the COPY command to move them to the workspace
COPY solution.py ./

# For ROS Agent - Additional Files
COPY rosagent.py ./
COPY template.launch ./

RUN /bin/bash -c "export PYTHONPATH="/usr/local/lib/python2.7/dist-packages:$PYTHONPATH""

# For ROS Agent - pulls the default configuration files 
# Think of this as the vehicle name
ENV HOSTNAME=default
ENV VEHICLE_NAME=default
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

# let's see what you've got there...

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

RUN /bin/bash -c "source ${CATKIN_WS_DIR}/devel/setup.bash"

ENV DISABLE_CONTRACTS=1
CMD ["python", "solution.py"]