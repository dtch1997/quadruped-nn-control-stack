FROM ubuntu:18.04

SHELL ["/bin/bash", "-c"]

# convert to "full" ubuntu
# RUN echo Y | xargs unminimize

# create user for vscode
RUN addgroup --gid 1000 vscode && \
  adduser --uid 1000 --gid 1000 --disabled-password --gecos "" vscode && \
  apt-get update && apt-get install -y sudo && \
  echo vscode ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/vscode && \
  chmod 0440 /etc/sudoers.d/vscode
USER vscode
WORKDIR /home/vscode

RUN sudo apt-get update && sudo apt-get install -y \
  build-essential \
  libboost-all-dev \
  liblcm-dev \
  cmake \
  libeigen3-dev \
  libyaml-cpp-dev \
  git