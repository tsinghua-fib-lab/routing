# 路径规划

## 使用方式

*以下均为使用`Remote Container`下的使用方式。*

### 初始化

第一次使用时，需要安装相应的Python包：
```bash
pip install -r requirements.txt
```

### 运行

```bash
./routing_server.py -h  # 按照提示输入相关参数即可运行
```

也可以使用如下`docker-compose.yml`运行（请将本文件的内容添加到其他docker-compose文件内，或者手动创建虚拟网络、删除注释、运行）：
```yml
version: "3"

networks:
  routing-tier:
  # 其他需要访问routing服务的容器需要连接这一虚拟网络
    name: routing-tier
    driver: bridge
    # 如果要使用已创建好的虚拟网络，则删除这行注释
    # external: true

services:
  routing:
    image: git.tsingroc.com:5050/sim/routing:latest
    ports:
    # 避免向公网暴露端口
      - 127.0.0.1:20218:20218
    command:
      - --mongo_uri=${MONGO_URI}
      - --map=${MONGO_DB_MAP}.${MONGO_COL_MAP}
      - --listen=0.0.0.0:20218
    networks:
      - routing-tier
```
具体运行方式如下：
1. 新建文件，命名为`docker-compose.yml`，并在其中填入上述内容；
2. 修改设置相关变量；
3. 执行`docker-compose pull`以更新本地镜像；
4. 执行`docker-compose up [-d]`以启动服务。
