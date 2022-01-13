import os
import time


num_of_scan = 0
scan_path = '/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/'
ply_path  = '/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/ply/'
out_path  = '/home/jin/mnt/Data/kitti/City/draco/cl1/'


for base, dirs, files in os.walk(scan_path):
    for file in files:
        num_of_scan += 1

print(num_of_scan)
num_of_scan = 100


enc_time = 0
for i in range(0, num_of_scan):
    in_xyz_fn  = "xyz_" + str(i).zfill(10) + ".ply"
    enc_xyz_fn = "enc_xyz_" + str(i).zfill(10) + ".drc"

    st = time.time()

    os.system('/home/jin/mnt/github/PCC_Comp/draco/build_dir/draco_encoder-1.4.1'+
              ' -i '+ply_path+in_xyz_fn +
              ' -cl 1' +
              ' -point_cloud' +
              ' -o ' +out_path+enc_xyz_fn
              )

    et = time.time()
    enc_time += (et-st)


# ./draco_decoder-1.4.1 -i out2.drc -o pc_mesh.ply
dec_time = 0
for i in range(0, num_of_scan):
    enc_xyz_fn = "enc_xyz_" + str(i).zfill(10) + ".drc"
    dec_xyz_fn = "dec_xyz_" + str(i).zfill(10) + ".ply"

    st = time.time()

    os.system('/home/jin/mnt/github/PCC_Comp/draco/build_dir/draco_decoder-1.4.1'+
              ' -i '+out_path+enc_xyz_fn +
              ' -o '+out_path+dec_xyz_fn
              )

    et = time.time()
    dec_time += (et-st)

enc_time = enc_time / num_of_scan
dec_time = dec_time / num_of_scan
print("Encoding Time: " + str(enc_time * 1000) + " ms")
print("Decoding Time: " + str(dec_time * 1000) + " ms")

