FROM docker.io/library/ubuntu:focal-20210416

RUN apt-get update && apt-get install -y locales \
    && localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y --no-install-recommends tzdata

ENV TZ=Asia/Shanghai \
    LANG=en_US.utf8

# environment related tools
RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    dnsutils \
    git \
    htop \
    inetutils-ping \
    inetutils-traceroute \
    iproute2 \
    jq \
    libcurl4 \
    liblzma5 \
    net-tools \
    netcat \
    nload \
    openssl \
    python3 \
    python3-pip \
    sysstat \
    telnet \
    unzip \
    vim \
    wget \
    zip

# develop related
RUN apt-get update && apt-get install -y \
    clang-12 \
    clangd-12 \
    clang-format-12 \
    cmake \
    graphviz \
    gdb \
    google-perftools \
    libgoogle-perftools-dev

ENV CC=/usr/bin/clang-12 \
    CXX=/usr/bin/clang++-12

COPY binary-tools /opt/binary-tools
RUN cd /opt/binary-tools \
    && dpkg -i mongodb-database-tools-ubuntu2004-x86_64-100.3.1.deb \
    && tar -zxvf mongodb-linux-x86_64-ubuntu2004-4.4.5.tgz \
    && cp mongodb-linux-x86_64-ubuntu2004-4.4.5/bin/mongo* /usr/local/bin/ \
    && tar -zxvf etcd-v3.4.15-linux-amd64.tar.gz \
    && cp etcd-v3.4.15-linux-amd64/etc* /usr/local/bin/

RUN pip3 install cpplint conan pymongo autopep8

ENV CONAN_USER_HOME=/workspace/.conan/
ENV ETCDCTL_API=3
