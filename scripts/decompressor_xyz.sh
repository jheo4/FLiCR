../bin/decompressor_xyz -i out.bin -o rec0.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 512 -y 64 -c lz77 -d 1
../bin/decompressor_xyz -i ./out.bin -o rec0.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 512 -y 64 -c rle_ri -d 1
