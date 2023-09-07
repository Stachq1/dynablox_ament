Dynablox
---

[Qingwen ZHANG](kin-zhang.github.io) is working on to remove the ROS, only leave the core of the Dynablox.

To our dufomap benchmark also.

Don't run it now

## Dependencies

ROS-full: for boost and Eigen

### glog gflag (only for debug)
glog gflag for debug only, will remove on release version
```sh
sh -c "$(wget -O- https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/Dockerfiles/latest_glog_gflag.sh)"
```

### yaml-cpp
Please set the FLAG, check this issue if you want to know more: https://github.com/jbeder/yaml-cpp/issues/682, [TOOD inside the CMakeLists.txt](https://github.com/jbeder/yaml-cpp/issues/566)

If you install in Ubuntu 22.04, please check this commit: https://github.com/jbeder/yaml-cpp/commit/c86a9e424c5ee48e04e0412e9edf44f758e38fb9 which is the version could build in 22.04

```sh
cd ${Tmp_folder}
git clone --branch yaml-cpp-0.6.0 --single-branch https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp
env CFLAGS='-fPIC' CXXFLAGS='-fPIC' cmake -Bbuild
cmake --build build --config Release
sudo cmake --build build --config Release --target install
```

An issue mentioned here, and I needed set 0.6.0 version to get rid of error, check [this comment](https://stackoverflow.com/a/75293221/9281669)
### Build
```bash
mkdir build && cd build
cmake .. && make
```

## Run
```
./dynablox_run /home/kin/workspace/DUFOMap/data/KITTI_00 ../assets/config.yaml
```