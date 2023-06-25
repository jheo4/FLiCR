import subprocess

def execute_program(cmd):
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            return result.stdout
        else:
            return result.stderr
    except FileNotFoundError:
        return "Error: The program '../bin/ri_histo_counter' could not be found."


levels = 40

data = {}
for i in range(1, levels+1):
    data[i] = 0

for i in range(0, 1000):
    command = [
                '../bin/ri_histo_counter',
                '-i', '/home/jin/mnt/Data/KITTI/original/4500/training/' + str(i).zfill(6) + '.bin',
                '--yaw_fov', '360',
                '--pitch_fov', '26.8',
                '--yaw_offset', '180',
                '--pitch_offset', '88',
                '--min_range', '1',
                '--max_range', '81',
                '-x', '4500',
                '-y', '64',
                '-l', str(levels)
                ]
    output = execute_program(command)
    lines = output.split('\n')

    for line in lines:
        if line != "":
            key, value = line.split(', ')

            key = int(key)+1
            value = int(value)

            # Add the key-value pair to the dictionary
            data[key] += value

# Print the key-value pairs
for key, value in data.items():
    print(f"{key}, {value}")

