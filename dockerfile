# syntax=docker/dockerfile:1

FROM ubuntu:24.04

ARG UID
ARG GID

# verilator install
RUN apt update \
&& apt-get -y install git help2man perl python3 make autoconf g++ flex bison ccache \
&& apt-get -y install libgoogle-perftools-dev numactl perl-doc \
&& apt-get -y install libfl2 \
&& apt-get -y install libfl-dev \
&& apt-get -y install xz-utils \
&& apt-get -y install verilator \
&& apt-get -y install libnss3-dev \
&& apt-get -y install libx11-xcb1 libgtk-3-0 libxss1 libcanberra-gtk3-module libgbm-dev \


&& apt-get install -y xterm \

# && git clone https://github.com/verilator/verilator  \
# && unset VERILATOR_ROOT \
# && cd verilator && git pull && git checkout v5.014 && autoconf && ./configure && make -j2 && make install \

# silicon compiler install
&& apt-get -y install python3-dev python3-pip python3-venv \
#&& git clone -b v0.36.3 https://github.com/siliconcompiler/siliconcompiler.git \
#&& mv siliconcompiler /home/ubuntu \

&& DEBIAN_FRONTEND=noninteractive TZ=Europe/Madrid apt-get -y install tzdata \
#&& apt-get -y install gcc-riscv64-unknown-elf \
#&& apt-get -y install gcc-riscv64-linux-gnu \
&& apt-get -y install gtkwave \
&& apt-get -y install wget \
&& apt-get -y install device-tree-compiler \
&& apt-get -y install zlib1g-dev \

&& wget https://github.com/shioyadan/Konata/releases/download/v0.34/konata-linux-x64.tar.gz \
&& tar -xzf konata-linux-x64.tar.gz \
&& rm konata-linux-x64.tar.gz \
&& mv konata-linux-x64 opt \


&& wget https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2025.01.20/riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz \
&& tar -xf riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz \
&& rm riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz \
&& mv riscv opt

ENV PATH="$PATH:/opt/konata-linux-x64"
ENV RISCV=/opt/riscv
### LD LIBRARY PATH ###
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RISCV/lib
### RISCV BINARY PATH ###
ENV PATH=$PATH:$RISCV/bin


RUN apt-get install -y sudo \
    && useradd -m -s /bin/bash silcomp \
    && echo 'silcomp ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

USER silcomp

WORKDIR /home/silcomp/

RUN python3 -m venv .venv \
    && .venv/bin/pip install --upgrade pip \
    && .venv/bin/pip install siliconcompiler

ENV VIRTUAL_ENV=/home/silcomp/.venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

