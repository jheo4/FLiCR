import os
import subprocess

comparison = '/home/jin/mnt/github/FLiCR/bin/size_comparison'

num_training = 7481

widths  = [2048, 1024, 512, 256]

ref_path = '/home/jin/mnt/Data/KITTI/original/4500/training/'
comp_root = '/home/jin/mnt/Data/KITTI/original/'

file = open("size_diff.csv", 'w')

for width in widths:
    comp_path = comp_root + str(width) + "/training/"

    print(f"{width}... ")
    file.write(f"{width}\n")
    for i in range(0, num_training):
        fn = str(i).zfill(6) + '.bin'
        ref_file  = ref_path + fn
        comp_file = comp_path + fn

        command = []
        command.append(comparison)
        command.append("-a")
        command.append(ref_file)
        command.append("-b")
        command.append(comp_file)

        result_stdout = subprocess.Popen(command, stdout=subprocess.PIPE).stdout
        result = result_stdout.read().strip()
        result = result.decode('utf-8')

        file.write(result + "\n")
        result_stdout.close()
    file.write(f"\n")

file.close()
