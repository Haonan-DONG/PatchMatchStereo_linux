# PatchMatch Stereo - linux version
- [PatchMatchStereo_14](https://github.com/ethan-li-coding/PatchMatchStereo). Linux Version with cmake file for PatchMatch Stereo from Yingsong Li.
- [PatchMatchStereo_CPU](https://github.com/nebula-beta/PatchMatch). CPU Implementation for PatchMatchStereo, with naive, row-col sweep and checkboard propagation strategy from nebula-beta.
- [PatchMatchStereo_CUDA](https://github.com/nebula-beta/PatchMatchCuda). CUDA Implementation for PMS, with checkboard propagation from nebula-beta.

## How to use

```shell
git clone https://github.com/Haonan-DONG/PatchMatchStereo_linux.git
cd PatchMatchStereo_linux/PatchMatchStereo_14/
mkdir build && cd build
cmake ..
make -j
```

And we will get a binary in the /build/, just test it like：

```shell
./main ..\Data\Reindeer\view1.png ..\Data\Reindeer\view5.png 0 128
```
