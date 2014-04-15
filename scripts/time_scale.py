import sys
header = sys.argv[5]
right_start = int(sys.argv[1])
right_end = int(sys.argv[2])
left_start = int(sys.argv[3])
left_end = int(sys.argv[4])
for i in range(right_start, right_end):
	file = header + '_right' + i
	with open(file, "rb") as fin:
		fin.seek(-2, 2)				 # Jump to the second last byte.
		while input.read(1) != "\n": # Until EOL is found...
			fin.seek(-2, 1)			 # ...jump back the read byte plus one more.
		last = fin.readline()		 # Read last line.
