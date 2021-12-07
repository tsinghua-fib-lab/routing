FROM ubuntu:20.04

# 运行./scripts/release.sh
# 从编译好的routing_server文件构建可执行镜像

RUN apt-get update \
  && apt-get install -y --no-install-recommends libgoogle-perftools-dev \
  && rm -rf /var/lib/apt/lists/*

COPY release/bin/routing_server /routing_server

ENTRYPOINT [ "/routing_server" ]
