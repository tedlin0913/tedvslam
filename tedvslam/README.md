Dependencies
```
sudo apt install libeigen3-dev
```
```
git clone https://github.com/dorian3d/DBoW2.git
cd DBoW2/
mkdir build
cd build
cmake -GNinja ..
ninja
sudo ninja install
```

```
sudo apt-get install libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev
wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz


wget https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.zip

```

```
cd external
```

```
sudo apt update && sudo apt install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip

```