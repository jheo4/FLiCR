./tile_test --input ../sample_data/0.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 0 --max_range 80 -x 4500 -y 64
./ri_point_printer -i ../sample_data/1.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 81 -x 4500 -y 64
./Lloyd_Quantizer --tr /home/jin/mnt/Data/KITTI/original/4500/training --tr_num 100 --tt /home/jin/mnt/Data/KITTI/original/4500/testing --tt_num 100 --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 0.5 --max_range 80.1 -x 4500 -y 64 -b 8 -i 10
./Lloyd_Quantizer --tr /home/jin/mnt/Data/KITTI/original/4500/training --tr_num 100 --tt /home/jin/mnt/Data/KITTI/original/4500/testing --tt_num 100 --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1.8 --max_range 80 -x 4500 -y 64 -b 8 -i 10
./Uniform_Tile_Quantizer --tt /home/jin/mnt/Data/KITTI/original/4500/testing --tt_num 100 --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 0 --max_range 80 -x 4500 -y 64 --xt 9 --yt 2

./Tile_Subsampling -i ../sample_data/3.bin --xt 9 --yt 2 --nf 0.8
