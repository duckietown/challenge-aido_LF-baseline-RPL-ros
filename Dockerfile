# Definition of Submission container

ARG ARCH=arm64v8
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}
ARG AIDO_REGISTRY=docker.io

FROM ${AIDO_REGISTRY}/duckietown/dt-car-interface:${BASE_TAG} AS dt-car-interface

FROM ${AIDO_REGISTRY}/duckietown/challenge-aido_lf-template-ros:${BASE_TAG} AS template

FROM ${AIDO_REGISTRY}/duckietown/dt-core:${BASE_TAG} AS base

WORKDIR /code

COPY --from=dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface

COPY --from=template /data/config /data/config
COPY --from=template /code/rosagent.py .

WORKDIR /code


# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python3-tk \
         python3-wheel \
         python3-pip  \
         software-properties-common && \
     rm -rf /var/lib/apt/lists/*

# RUN apt-get update -y && \
#   add-apt-repository ppa:deadsnakes/ppa -y && \
#   apt-get update -y && \
#   apt-get install -y python3.7-dev && \
#   ln -sf /usr/bin/python3.7 /usr/bin/python3



# Before installing
RUN echo PYTHONPATH=$PYTHONPATH
RUN pip install pipdeptree
RUN pipdeptree
RUN pip list


ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}

COPY requirements.pin.txt ./
COPY build_on_nano.sh ./
RUN if [ "ARCH" = "amd64" ] ; then pip3 install --use-feature=2020-resolver -r requirements.pin.txt -f https://download.pytorch.org/whl/torch_stable.html ; else ./build_on_nano.sh ; fi


COPY requirements.* ./
# RUN cat requirements.* > .requirements.txt
RUN  pip3 install --use-feature=2020-resolver -r requirements.txt

RUN pip3 uninstall dataclasses -y

RUN echo PYTHONPATH=$PYTHONPATH
RUN pipdeptree
RUN pip list

RUN pip3 uninstall -y dataclasses

# For ROS Agent - Need to upgrade Pillow for Old ROS stack
#RUN pip3 install pillow --user --upgrade


RUN mkdir /code/submission_ws

COPY submission_ws/src /code/submission_ws/src
COPY launchers /code

COPY rl_agent rl_agent

# let's copy all our solution files to our workspace
# if you have more file use the COPY command to move them to the workspace
COPY solution.py ./

ENV HOSTNAME=agent
ENV VEHICLE_NAME=agent
ENV ROS_MASTER_URI=http://localhost:11311


RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${CATKIN_WS_DIR}/devel/setup.bash  && \
    catkin build --workspace /code/catkin_ws && \
    catkin build --workspace /code/submission_ws


# Note: here we try to import the solution code
# so that we can check all of the libraries are imported correctly
RUN /bin/bash -c "source ${CATKIN_WS_DIR}/devel/setup.bash && python3 -c 'from solution import *'"

ENV DISABLE_CONTRACTS=1
CMD ["bash", "run_and_start.sh"]
