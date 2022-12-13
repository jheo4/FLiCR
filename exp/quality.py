import os
import subprocess

quality_calculator = '/home/jin/mnt/github/FLiCR/bin/pc_quality'

num_training = 7481

widths  = [2048, 1024, 512, 256]

ref_path = '/home/jin/mnt/Data/KITTI/original/4500/training/'
comp_root = '/home/jin/mnt/Data/KITTI/original/'


for width in widths:
    file = open(f"quality_{width}_res.csv", 'w')
    comp_path = comp_root + str(width) + "/training/"

    print(f"{width}... ")
    file.write(f"{width}\n")
    for i in range(0, num_training):
        fn = str(i).zfill(6) + '.bin'
        ref_file  = ref_path + fn
        comp_file = comp_path + fn

        command = []
        command.append(quality_calculator)
        command.append("-r")
        command.append(ref_file)
        command.append("-c")
        command.append(comp_file)
        # command.append("--c_xyz")
        # command.append("1")
        command.append("-m")
        command.append("80")

        result_stdout = subprocess.Popen(command, stdout=subprocess.PIPE).stdout
        result = result_stdout.read().strip()
        result = result.decode('utf-8')

        file.write(result + "\n")
        result_stdout.close()
    file.close()

