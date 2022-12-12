import os
import shutil

interpolator = '~/mnt/github/FLiCR/bin/pc_downsample_interpolator'

num_training = 7481
num_testing  = 7518

height = 64
widths  = [2048, 1024, 512, 256]
inter_ints = [2]
wnd_ints = [1]
wnd_size = 8

orig_dir = '/home/jin/mnt/Data/KITTI/original/'

training_data_path = '/home/jin/mnt/Data/KITTI/original/4500/training/'

for width in widths:
    for inter_int, wnd_int in zip(inter_ints, wnd_ints):
        # output_dir      = '/home/jin/mnt/Data/KITTI/interpolation/' + str(int(width*inter_int)) + '/'
        output_dir      = '/home/jin/mnt/Data//KITTI/interpolation0.5/' + str(int(width*inter_int)) + '/'
        training_output = output_dir + "training/"
        print(training_output)

        if not os.path.exists(training_output):
           os.makedirs(training_output)

        # Training
        for i in range(0, num_training):
            fn = str(i).zfill(6) + ".bin"
            file     = training_data_path + fn
            out_file = training_output + fn

            command = f"{interpolator} -i {file} -o {out_file} \
                    --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 \
                    --min_range 1 --max_range 80 \
                    -x {width} -y {height} \
                    --wnd_size {wnd_size} --intr_per_wnd {int(wnd_size*wnd_int)} --circular 1 \
                    --grad_thresh 0.5 --policy random"
            # print(command)
            os.system(command)
'''
        # Testing
        for i in range(0, num_testing):
            fn = str(i).zfill(6) + ".bin"
            file =     testing_data_path + fn
            out_file = testing_output + fn

            command = f"{interpolator} -i {file} -o {out_file} \
                    --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 \
                    --min_range 1 --max_range 80 \
                    -x {width} -y {height} \
                    --wnd_size {wnd_size} --intr_per_wnd {int(wnd_size*wnd_int)} --circular 1 \
                    --grad_thresh 0.5 --policy random"

            # print(command)
            os.system(command)
'''
