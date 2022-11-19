rm *.bin
rm *.png
rm *.yaml

../bin/pc_downsample_interpolator -i ../sample_data/0.bin -o interpolated_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 --wnd_size 4 --intr_per_wnd 2 --circular 1 --grad_thresh 2 --policy random -d 1
../bin/pc_downsampler_cv_interpolator -i ../sample_data/0.bin -o interpolated_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64  -a linear  --intr_x 320 --intr_y 64 -d 1

# normalized ri/intmap
../bin/ri_intmap_downsampler -i ../sample_data/0.bin -o out_ri.png -m out_int.png --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -n 1 -d 1
# ri/intmap as float values
../bin/ri_intmap_downsampler -i ../sample_data/0.bin -o out_ri.yaml -m out_int.yaml --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -x 256 -y 64 -d 1

# my interpolation
../bin/ri_intmap_interpolator -i out_ri.yaml -m out_int.yaml -o intr_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 --wnd_size 4 --intr_per_wnd 2 --circular 1 --grad_thresh 2 --policy random -d 1
# cv interpolation
../bin/cv_interpolator -i out_ri.yaml -m out_int.yaml -x 384 -y 64 -a linear -o linear_intr_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -d 1
../bin/cv_interpolator -i out_ri.yaml -m out_int.yaml -x 384 -y 64 -a cubic -o cubic_intr_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -d 1
../bin/cv_interpolator -i out_ri.yaml -m out_int.yaml -x 384 -y 64 -a lz4 -o lz4_intr_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -d 1
../bin/cv_interpolator -i out_ri.yaml -m out_int.yaml -x 384 -y 64 -a area -o area_intr_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -d 1
../bin/cv_interpolator -i out_ri.yaml -m out_int.yaml -x 384 -y 64 -a nearest -o nearest_intr_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -d 1

# quality check
../bin/ri_intmap_pc_converter -i out_ri.yaml -m out_int.yaml -o src_pc.bin --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 --min_range 1 --max_range 80 -d 1
../bin/pc_quality -r ../sample_data/0.bin -c src_pc.bin -m 80

../bin/pc_quality -r ../sample_data/0.bin -c intr_pc.bin -m 80
../bin/pc_quality -r ../sample_data/0.bin -c linear_intr_pc.bin -m 80
../bin/pc_quality -r ../sample_data/0.bin -c cubic_intr_pc.bin -m 80
../bin/pc_quality -r ../sample_data/0.bin -c lz4_intr_pc.bin -m 80
../bin/pc_quality -r ../sample_data/0.bin -c area_intr_pc.bin -m 80

