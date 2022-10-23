# FLiCR: A Fast and Lightweight LiDAR Point Cloud Compression Based on Lossy RI
## Related Papers
 - FLiCR: A Fast and Lightweight LiDAR Point Cloud Compression Based on Lossy RI (SEC '22)
 - Poster: Making Edge-assisted LiDAR Perceptions Robust to Lossy Point Cloud Compression (SEC '22)

## Installation on Ubuntu20.04
### Dependencies
 - OpenCV4
   ```
     sudo apt install libopencv-dev python3-opencv
   ```
- PCL
    ```
    sudo apt install libpcl-dev libpcl-conversions-dev
    ```
- spdlog
    ```
    git clone https://github.com/gabime/spdlog.git && cd spdlog
    git checkout v1.8.2
    mkdir build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    ```
 - cxxopt
    ```
    git clone https://github.com/jarro2783/cxxopts.git && cd cxxopts
    mkdir build && cd build && cmake .. && make -j
    sudo make install
    ```
  - Boost
    ```
    sudo apt-get install libboost-all-dev
    ```
  - OpenMP
    ```
    sudo apt install libomp-dev
    ```

### Build
```
cd PROJECT_DIR
mkdir builds && cd builds
cmake .. && make -j $(nproc)
```

## Examples
All the example codes are under `example/`, and the example commands are in `scripts/`.
