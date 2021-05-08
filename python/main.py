from load_dataset import load_dataset
from load_frame_tracklets import load_frame_tracklets
from drawer import *

date = "2011_09_26"
drive = "0002"

dataset = load_dataset(date, drive)
tracklet_rects, tracklet_types = load_frame_tracklets(len(list(dataset.velo)),
        'data/{}/{}_drive_{}_sync/tracklet_labels.xml'.format(date, date, drive))

frame = 10

display_frame_statistics(dataset, tracklet_rects, tracklet_types, frame)
