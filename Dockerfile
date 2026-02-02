FROM ghcr.io/siliconcompiler/sc_tools:v0.36.4

RUN apt-get update \
    && apt-get -y install git help2man perl python3 make autoconf g++ flex bison ccache \
    && apt-get -y install libgoogle-perftools-dev numactl perl-doc \
    && apt-get -y install libfl2 \
    && apt-get -y install libfl-dev \
    && apt-get -y install xz-utils \
    && apt-get -y install verilator \
    && apt-get -y install libnss3-dev \
    && apt-get -y install libx11-xcb1 libgtk-3-0 libxss1 libcanberra-gtk3-module libgbm-dev \
    && apt-get install -y xterm \
    && apt-get -y install sudo \
    # silicon compiler install
    && apt-get -y install python3-dev python3-pip python3-venv \
    && DEBIAN_FRONTEND=noninteractive TZ=Europe/Madrid apt-get -y install tzdata \
    && apt-get -y install gtkwave \
    && apt-get -y install wget \
    && apt-get -y install device-tree-compiler \
    && apt-get -y install zlib1g-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Download and install Konata
RUN wget https://github.com/shioyadan/Konata/releases/download/v0.34/konata-linux-x64.tar.gz \
    && tar -xzf konata-linux-x64.tar.gz \
    && rm konata-linux-x64.tar.gz \
    && mv konata-linux-x64 /opt/konata-linux-x64

# Download and install RISC-V toolchain
RUN wget https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2025.01.20/riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz \
    && tar -xf riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz \
    && rm riscv64-elf-ubuntu-24.04-gcc-nightly-2025.01.20-nightly.tar.xz \
    && mv riscv /opt/riscv

# Environment variables
ENV PATH="$PATH:/opt/konata-linux-x64"
ENV RISCV=/opt/riscv
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RISCV/lib
ENV PATH=$PATH:$RISCV/bin

# Create virtual environment and install siliconcompiler
RUN mkdir -p /root/.local/bin \
    && mkdir -p /root/.local/lib \
    && python3 -m venv /opt/.venv \
    && /opt/.venv/bin/pip install --upgrade pip \
    && /opt/.venv/bin/pip install siliconcompiler

# Add venv to PATH
ENV PATH="/opt/.venv/bin:$PATH"
ENV PATH="/root/.local/bin:$PATH"
ENV LD_LIBRARY_PATH="/root/.local/lib:$LD_LIBRARY_PATH"

WORKDIR /workspace

#RUN sc-install -group asic