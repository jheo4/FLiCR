import os
import shutil

interpolator = '~/mnt/github/FLiCR/bin/pc_downsampler_cv_interpolator'

num_training = 7481
num_testing  = 7518

height = 64
widths  = [2048, 1024, 512, 256]
inter_ints = [1.25, 1.5, 1.75]

algorithms = ["linear", "nearest", "lz4", "cubic", "area"]

training_data_path = '/home/jin/mnt/Data/KITTI/original/4500/training/'
testing_data_path  = '/home/jin/mnt/Data/KITTI/original/4500/testing/'

for algorithm in algorithms:
    for width in widths:
        for inter_int in inter_ints:
            inter_width = int(width*inter_int)
            output_dir      = '/home/jin/mnt/Data/KITTI/cv_inter/' + algorithm + "/" + str(inter_width) + '/'
            training_output = output_dir + "training/"
            testing_output  = output_dir + "testing/"

            if not os.path.exists(training_output):
               os.makedirs(training_output)
            if not os.path.exists(testing_output):
               os.makedirs(testing_output)

            # Training
            for i in range(0, num_training):
                print(f"training {algorithm}, {width}, {inter_int} {i}th")
                fn = str(i).zfill(6) + ".bin"
                file     = training_data_path + fn
                out_file = training_output + fn

                command = f"{interpolator} -i {file} -o {out_file} \
                        --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 \
                        --min_range 1 --max_range 80 \
                        -x {width} -y {height} \
                        -a {algorithm} --intr_x {inter_width} --intr_y {height}"
                # print(command)
                os.system(command)
'''
            # Testing
            for i in range(0, num_testing):
                print(f"testing {algorithm}, {width}, {inter_int} {i}th")
                fn = str(i).zfill(6) + ".bin"
                file =     testing_data_path + fn
                out_file = testing_output + fn

                command = f"{interpolator} -i {file} -o {out_file} \
                        --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 --pitch_offset -88 \
                        --min_range 1 --max_range 80 \
                        -x {width} -y {height} \
                        -a {algorithm} --intr_x {inter_width} --intr_y {height}"
                # print(command)
                os.system(command)
'''
