FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y sudo locales && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8
RUN apt-get update && apt-get install -y git wget curl cmake ninja-build build-essential \
    python3 python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspaces/x500-stack
