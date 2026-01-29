# syntax=docker/dockerfile:1

FROM ubuntu:24.04

ARG UID
ARG GID

# verilator install
RUN apt update
RUN apt-get -y install git help2man perl python3 make autoconf g++ flex bison ccache
RUN apt-get -y install libgoogle-perftools-dev numactl perl-doc
RUN apt-get -y install libfl2
RUN apt-get -y install libfl-dev
RUN apt-get -y install xz-utils
RUN apt-get -y install verilator
RUN apt-get -y install libnss3-dev
RUN apt-get -y install libx11-xcb1 libgtk-3-0 libxss1 libcanberra-gtk3-module libgbm-dev


RUN apt-get install -y xterm

# RUN git clone https://github.com/verilator/verilator
# RUN unset VERILATOR_ROOT
# RUN cd verilator && git pull && git checkout v5.014 && autoconf && ./configure && make -j2 && make install

# silicon compiler install
RUN apt-get -y install python3-dev python3-pip python3-venv
RUN git clone -b v0.36.3 https://github.com/siliconcompiler/siliconcompiler.git

RUN DEBIAN_FRONTEND=noninteractive TZ=Europe/Madrid apt-get -y install tzdata
#RUN apt-get -y install gcc-riscv64-unknown-elf
#RUN apt-get -y install gcc-riscv64-linux-gnu
RUN apt-get -y install gtkwave
RUN apt-get -y install wget
RUN apt-get -y install device-tree-compiler
RUN apt-get -y install zlib1g-dev

RUN wget https://github.com/shioyadan/Konata/releases/download/v0.34/konata-linux-x64.tar.gz
RUN tar -xzf konata-linux-x64.tar.gz
RUN rm konata-linux-x64.tar.gz
RUN mv konata-linux-x64 opt

ENV PATH="$PATH:/opt/konata-linux-x64"

RUN wget https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2025.01.20/riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz
RUN tar -xf riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz
RUN rm riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz
RUN mv riscv opt
ENV RISCV=/opt/riscv
### LD LIBRARY PATH ###
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RISCV/lib
### RISCV BINARY PATH ###
ENV PATH=$PATH:$RISCV/bin


# Update the package list, install sudo, create a non-root user, and grant password-less sudo permissions
#RUN apt update && \
#    apt install -y sudo

#RUN grep -q ":$GID:" /etc/group || groupadd -g $GID silcomp 
#RUN adduser --uid $UID --gid $GID --disabled-password --gecos "" silcomp && \
#    echo 'silcomp ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

# Set the non-root user as the default user
USER ubuntu

# Set the working directory
WORKDIR /home/ubuntu/

