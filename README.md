# 路径规划

**使用中需要保证`protos`的版本与`map-loader-cxx`中一致，否则会出现未知运行时问题。**

## 使用方式

*以下均为使用`Remote Container`下的使用方式。*

### 初始化

第一次使用时，需要初始化`conan`：
```bash
bash init_conan.sh
```
这一步将初始化C++包管理工具`conan`，并指定编译使用的C++标准为`gnu17`。

### 编译

#### 脚本

```bash
bash scripts/compile.sh
```

### 手动

首先，根据`conanfile.txt`中指定的依赖，编译安装依赖并生成供CMake使用的`build/conanbuildinfo.cmake`：
```bash
cd build/
conan install .. --build=missing
```
这里会下载、编译、安装所有给定的依赖到`CONAN_USER_HOME`中（该环境变量在`Dockerfile`中指定）。

并在最高层`CMakeLists.txt`中加入：
```cmake
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)
```

其次，在项目中所有需要依赖`conan`管理的库的相关`CMakeLists.txt`中加入：
```cmake
target_link_libraries(target_name PRIVATE|PUBLIC CONAN_PKG::xxxxxx)
```
`CONAN_PKG::`后接conanfile.txt中的package名。

最后，按照一般CMake编译流程进行编译。

### 运行

```bash
./build/bin/routing_server --flagfile flags/server_beijing3.flag  # 如不使用flagfile，参考flagfile中的方式输入参数
```

也可以使用如下`docker-compose.yml`运行：
```yml
version: "3"

networks:
  routing-tier:
  # 其他需要访问routing服务的容器需要连接这一虚拟网络
    name: routing-tier
    driver: bridge

services:
  routing:
    image: git.tsingroc.com:5050/sim/routing:latest
    ports:
    # 避免向公网暴露端口
      - 127.0.0.1:20218:20218
    command:
      - --grpc_listen=0.0.0.0:20218
      - --mongo_col_map=${MONGO_COL_MAP}
      - --mongo_db_map=${MONGO_DB_MAP}
      - --mongo_uri=${MONGO_URI}
      - --routing_cost_type=time
    networks:
      - routing-tier
```
具体运行方式如下：
1. 新建文件，命名为`docker-compose.yml`，并在其中填入上述内容；
2. 修改设置相关变量；
3. 执行`docker-compose pull`以更新本地镜像；
4. 执行`docker-compose up [-d]`以启动服务。

### 注意事项
`road_graph.cc`中提供了宏`IGNORE_MAP_REVISION`用于关闭运行时限行版本检查，在近期（2021年9月）的比赛中默认开启。
