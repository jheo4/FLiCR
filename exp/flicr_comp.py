import os

compressor   = '/home/jin/mnt/github/FLiCR/bin/compressor_xyzi2xyz'
decompressor = '/home/jin/mnt/github/FLiCR/bin/decompressor_xyz'

num_training = 7481

widths  = [256, 512, 1024, 2048]
height = 64

orig_path = '/home/jin/mnt/Data/KITTI/original/4500/training/'
output_root         = '/home/jin/mnt/github/FLiCR/exp/output/'
temp_root           = '/home/jin/mnt/github/FLiCR/exp/output/temp/'


for width in widths:
    output_dir = output_root + str(width) + "/training/"
    temp_dir   = temp_root + str(width) + "/training/"

    if not os.path.exists(output_dir):
       os.makedirs(output_dir)
    if not os.path.exists(temp_dir):
       os.makedirs(temp_dir)

    print(width)
    # Training
    for i in range(0, num_training):
        fn = str(i).zfill(6) + ".bin"

        file      = orig_path + fn
        comp_file = temp_dir + fn
        out_file  = output_dir + fn

        comp_command = f"{compressor} -i {file} -o {comp_file} \
                --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 \
                --min_range 1 --max_range 80 -x {width} -y {height} -c lz77"
        os.system(comp_command)

        decomp_command = f"{decompressor} -i {comp_file} -o {out_file} \
                --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 \
                --min_range 1 --max_range 80 -x {width} -y {height} -c lz77"
        os.system(decomp_command)
