../bin/lz77_enc -i ../sample_data/0.bin -o out.bin -d 1
../bin/lz77_dec -i ./out.bin -o rec0.bin -d 1

../bin/pcl_enc -i ../sample_data/0.bin -o out.bin -d 1
../bin/pcl_dec -i ./out.bin -o rec0.bin -d 1

../bin/rle_enc -i ../sample_data/0.bin -o out.bin -d 1
../bin/rle_dec -i ./out.bin -o rec0.bin -d 1

