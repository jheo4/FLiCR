rm *.bin

# normalized ri/intmap
../bin/pc_downsampler -i ../sample_data/0.bin -o nout0.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -n 1 -d 1
# ri/intmap as float values
../bin/pc_downsampler -i ../sample_data/0.bin -o out0.bin  --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -d 1

../bin/pc_quality -r ../sample_data/0.bin -c nout0.bin -m 80
../bin/pc_quality -r ../sample_data/0.bin -c out0.bin -m 80

../bin/pc_visualizer -i nout0.bin -s 1000
../bin/pc_visualizer -i out0.bin -s 1000
