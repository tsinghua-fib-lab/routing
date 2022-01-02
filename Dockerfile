FROM python:3.9-slim

COPY . /
# 注意：pypi源为阿里云内网镜像源，如果要在公网下运行请注释掉相关的两行
RUN PIP_NO_CACHE_DIR=1 pip install \
    -i http://mirrors.cloud.aliyuncs.com/pypi/simple/ \
    --trusted-host mirrors.cloud.aliyuncs.com \
    -r /requirements.txt

ENTRYPOINT [ "/routing_server.py" ]
