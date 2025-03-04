FROM mcr.microsoft.com/devcontainers/base:jammy

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y build-essential checkinstall zlib1g-dev libssl-dev
RUN wget https://github.com/Kitware/CMake/releases/download/v3.31.5/cmake-3.31.5.tar.gz && \
    tar -zxf cmake-3.31.5.tar.gz && \
    cd cmake-3.31.5 && \
    ./bootstrap && \
    make && \
    make install