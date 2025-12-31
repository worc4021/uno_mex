FROM ubuntu:noble

# General purpose build tools
RUN export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt install --no-install-recommends -y \
        ca-certificates gnupg2 openssh-client git sudo wget \
        python3 python3-pip python3-venv xvfb mold libgomp1 \
        gcc-14 g++-14 gfortran-14 cmake ninja-build build-essential \
    && apt-get clean \
    && apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/* \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 100 \
    && update-alternatives --install /usr/bin/gfortran gfortran /usr/bin/gfortran-14 100 \
    && update-alternatives --install /usr/bin/ld ld /usr/bin/mold 100 

RUN wget https://github.com/Kitware/CMake/releases/download/v4.2.1/cmake-4.2.1-linux-x86_64.sh \
    && sh cmake-4.2.1-linux-x86_64.sh --skip-license --prefix=/usr/local \
    && rm cmake-4.2.1-linux-x86_64.sh \
    && update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 100

ADD https://github.com/OpenMathLib/OpenBLAS.git /build/OpenBLAS

WORKDIR /build/OpenBLAS
RUN mkdir build; \
    make FFLAGS="-fPIC" CFLAGS="-fPIC" NO_SHARED=1; \
    make FFLAGS="-fPIC" CFLAGS="-fPIC" NO_SHARED=1 PREFIX=/usr/local install

ARG USERNAME=vscode

RUN deluser --remove-home ubuntu 2> /dev/null \
    && adduser --shell /bin/bash --disabled-password --uid 1000 --gecos "" "$USERNAME" \
    && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}   