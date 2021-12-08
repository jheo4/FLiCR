import os
import time


num_of_scan = 0
scan_path = '/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/'
ply_path  = '/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/ply/'
out_path  = '/home/jin/mnt/Data/kitti/City/tmc13/1cm/'


for base, dirs, files in os.walk(scan_path):
    for file in files:
        num_of_scan += 1

print(num_of_scan)
num_of_scan = 100

enc_time = 0
for i in range(0, num_of_scan):
    in_xyz_fn  = "xyz_" + str(i).zfill(10) + ".ply"
    enc_xyz_fn = "enc_xyz_" + str(i).zfill(10) + ".bin"

    st = time.time()

    os.system('/home/jin/mnt/github/PCC_Comp/mpeg-pcc-tmc13/build/tmc3/tmc3'+' --mode=0'+
              ' --uncompressedDataPath='+ply_path+in_xyz_fn +
              ' --compressedStreamPath='+out_path+enc_xyz_fn +
              ' --outputPrecisionBits=0' +
              ' --srcUnitLength=1' +
              ' --srcUnit=1' +
              ' --inputScale=100'
              )

    et = time.time()
    enc_time += (et-st)


dec_time = 0
for i in range(0, num_of_scan):
    enc_xyz_fn = "enc_xyz_" + str(i).zfill(10) + ".bin"
    dec_xyz_fn = "dec_xyz_" + str(i).zfill(10) + ".ply"

    st = time.time()

    os.system('/home/jin/mnt/github/PCC_Comp/mpeg-pcc-tmc13/build/tmc3/tmc3'+' --mode=1'+
              ' --compressedStreamPath='+out_path+enc_xyz_fn +
              ' --reconstructedDataPath='+out_path+dec_xyz_fn +
              ' --outputUnitLength=0' +
              ' --outputBinaryPly=0'
              )

    et = time.time()
    dec_time += (et-st)

enc_time = enc_time / num_of_scan
dec_time = dec_time / num_of_scan
print("Encoding Time: " + str(enc_time * 1000) + " ms")
print("Decoding Time: " + str(dec_time * 1000) + " ms")

