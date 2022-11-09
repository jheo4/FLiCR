../bin/pc_visualizer -i ../sample_data/0.bin -s 1000
../bin/pc2ri_visualizer -i ../sample_data/0.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -d 1
../bin/ri_intmap_visualizer -i ../sample_data/0.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -d 1
