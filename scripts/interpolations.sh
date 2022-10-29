../bin/ri_intmap_downsampler -i ../sample_data/0.bin -o out_ri.png -m out_int.png --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -n 1 -d 1 # normalized ri/intmap
../bin/ri_intmap_downsampler -i ../sample_data/0.bin -o out_ri.yaml -m out_int.yaml --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -d 1 # ri/intmap as float values

