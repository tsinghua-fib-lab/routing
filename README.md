# 路径规划

## 使用方式

*以下均为使用`Remote Container`下的使用方式。*

### 初始化

第一次使用时，需要初始化`conan`：
```bash
bash scripts/init_conan.sh
```
这一步将初始化C++包管理工具`conan`，并指定编译使用的C++标准为`gnu17`。

### 编译

#### 脚本

```bash
# 由于grpc/1.39.0包位于rl1上，所以第一次执行时需要连接校园网
bash compile.run
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
conan_target_link_libraries(routing_map_loader)
```

最后，按照一般CMake编译流程进行编译。

### 运行
