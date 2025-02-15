# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.177.0/containers/cpp/.devcontainer/base.Dockerfile

# [Choice] Debian / Ubuntu version: debian-10, debian-9, ubuntu-20.04, ubuntu-18.04
ARG VARIANT="ubuntu-22.04"

FROM mcr.microsoft.com/vscode/devcontainers/cpp:0-$VARIANT
# https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
ENV TOOLCHAINURL="https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel"
ENV TOOLCHAINARCHIVE="arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz"
ENV TOOLCHAINDIR="arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi"

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends cmake clangd usbutils ninja-build build-essential wget ca-certificates python3-pip \
    gdb-multiarch automake autoconf libtool libftdi-dev libusb-1.0-0-dev pkg-config clang-format libhidapi-dev libftdi1 libftdi1-2


RUN ln -s /lib/x86_64-linux-gnu/libncursesw.so.6 /lib/x86_64-linux-gnu/libncursesw.so.5 && \
    ln -s /lib/x86_64-linux-gnu/libtinfo.so.6 /lib/x86_64-linux-gnu/libtinfo.so.5
    
RUN mkdir /workspace
WORKDIR /workspace

COPY 99-picoprobe.rules /etc/udev/rules.d/99-picoprobe.rules

RUN usermod -aG plugdev vscode
