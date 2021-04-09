#### Copy 3rd party libs 

```bash
cp -r /usr/local/driveworks/samples/3rdparty 3rdparty/
```

#### Build command 

```bash
src/catkin/bin/catkin_make_isolated -DVIBRANTE_PDK:STRING=$HOME/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DDPX/DRIVEOS/drive-t186ref-linux -DTRT_VERSION:STRING=6.3.1.3 -DCMAKE_TOOLCHAIN_FILE=$HOME/Downloads/Toolchain-V5L.cmake -DCMAKE_EXE_LINKER_FLAGS="${CMAKE_EXE_LINKER_FLAGS} -L/usr/local/driveworks/targets/aarch64-Linux/lib -Wl,-rpath,/usr/local/driveworks/targets/aarch64-Linux/lib -L$SYSROOT/usr/local/cuda-10.2/targets/aarch64-linux/lib -Wl,-rpath,$SYSROOT/usr/local/cuda-10.2/targets/aarch64-linux/lib -L$SYSROOT/usr/lib/aarch64-linux-gnu/openblas -Wl,-rpath,$SYSROOT/usr/lib/aarch64-linux-gnu/openblas" --install
```
