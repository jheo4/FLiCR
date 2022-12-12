import os

downsampler = '~/mnt/github/FLiCR/bin/pc_downsampler'

num_training = 7481
num_testing  = 7518

widths  = [256, 512, 1024, 2048]
height = 64

training_data_path = '/home/jin/mnt/Data/KITTI/original/4500/training/'
testing_data_path  = '/home/jin/mnt/Data/KITTI/original/4500/testing/'
output_dir      = '/home/jin/mnt/Data/KITTI/original/'


for width in widths:
    training_output = output_dir + str(width) + "/training/"
    testing_output  = output_dir + str(width) + "/testing/"

    if not os.path.exists(training_output):
       os.makedirs(training_output)
    if not os.path.exists(testing_output):
       os.makedirs(testing_output)

    # Training
    for i in range(0, num_training):
        fn = str(i).zfill(6) + ".bin"
        file     = training_data_path + fn
        out_file = training_output + fn


        command = f"{downsampler} -i {file} -o {out_file} --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 \
                --pitch_offset -88 --min_range 1 --max_range 80 -x {width} -y {height}"
        print(command)
        os.system(command)
'''
    # Testing
    for i in range(0, num_testing):
        fn = str(i).zfill(6) + ".bin"
        file =     testing_data_path + fn
        out_file = testing_output + fn

        command = f"{downsampler} -i {file} -o {out_file} --yaw_fov 360 --pitch_fov 26.8 --yaw_offset 180 \
                --pitch_offset -88 --min_range 1 --max_range 80 -x {width} -y {height}"
        print(command)
        os.system(command)
'''
