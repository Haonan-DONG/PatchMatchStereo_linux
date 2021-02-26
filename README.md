# PatchMatch Stereo - linux version
Linux Version with cmake file for PatchMatch Stereo.
The code's author is Yingsong Li, directory to <https://github.com/ethan-li-coding/PatchMatchStereo>.
Many thanks to him for this excellent work for learning stereo vision.

## How to use

```shell
git clone https://github.com/Haonan-DONG/PatchMatchStereo_linux.git
cd PatchMatchStereo_linux
mkdir build && cd build
cmake ..
make -j
```

And we will get a binary in the /build/, just test it like：

```shell
./main ..\Data\Reindeer\view1.png ..\Data\Reindeer\view5.png 0 128
```
The Data can be download at Li's github.