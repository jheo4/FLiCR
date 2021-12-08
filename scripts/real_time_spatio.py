import os
import time


num_of_scan = 0
scan_path = '/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/'
ply_path  = '/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/ply/'
out_path  = '/home/jin/mnt/Data/kitti/City/realtime/'


for base, dirs, files in os.walk(scan_path):
    for file in files:
        num_of_scan += 1

print(num_of_scan)
num_of_scan = 100


enc_time = 0
files = ''
for i in range(0, num_of_scan):
    in_xyz_fn  = str(i).zfill(10) + ".bin"
    files = files + in_xyz_fn + " "

st = time.time()
cmd = '/home/jin/mnt/github/pcc_comp/Real-Time-Spatio-Temporal-LiDAR-Point-Cloud-Compression/src/pcc_stream_encoder' + \
      ' -p 0.18 -y 0.45 -f binary -l 4 -t 0.5' + \
      ' --out ' + out_path+'frames.tar.gz' + \
      ' --input-path ' + scan_path + \
      ' --input-files ' + files
#print(cmd)
os.system(cmd)
et = time.time()
enc_time = et-st

dec_time = 0
st = time.time()
cmd = '/home/jin/mnt/github/pcc_comp/Real-Time-Spatio-Temporal-LiDAR-Point-Cloud-Compression/src/pcc_stream_decoder ' + \
      ' -p 0.18 -y 0.45 -f binary -l 4' + \
      ' --input ' + out_path+'frames.tar.gz'
os.system(cmd)
et = time.time()
dec_time = et-st

enc_time = enc_time / num_of_scan
dec_time = dec_time / num_of_scan
print("Encoding Time: " + str(enc_time * 1000) + " ms")
print("Decoding Time: " + str(dec_time * 1000) + " ms")
