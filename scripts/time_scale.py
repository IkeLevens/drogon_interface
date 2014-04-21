import sys
import os

right_start = int(sys.argv[1])
right_end = int(sys.argv[2])
left_start = int(sys.argv[3])
left_end = int(sys.argv[4])
fileStart = sys.argv[5]
outputLength = 8.0
#scale = [10,10,10,3.16,3.45,4.5,4.59,4.31,10,3.79,4.3,4.4,4.45,4.7,10,10]
#for i in range(1,16):
#	scale[i] /= 3.16
#	scale[i] += outputLength
for i in range(right_start, right_end + 1):
	print i
	inputName = fileStart + '_right_' + str(i) + '.trj'
	outputName = fileStart + '_right_' + str(i) + '.ts'
	with open(inputName, 'r') as fin:
		fin.seek(-2, 2)				# Jump to the second last byte.
		while fin.read(1) != "\n":	# Until EOL is found...
			fin.seek(-2, 1)			# ...jump back the read byte plus one more.
		last = fin.readline()		# Read last line.
		executionTime = float(last.split(',', 1)[0])
		print executionTime
		fin.seek(0,0)
		with open(outputName, 'w') as fout:
			lines = 0
			for line in fin:
				if lines > 0:
					fout.write(str(float(line.split(',', 1)[0]) * outputLength / executionTime) + ',')
					fout.write(line.split(',', 1)[1])
					lines += 1
				else:
					fout.write(line)
					lines += 1
for i in range(left_start, left_end + 1):
	print i
	inputName = fileStart + '_left_' + str(i) + '.trj'
	outputName = fileStart + '_left_' + str(i) + '.ts'
	with open(inputName, 'r') as fin:
		fin.seek(-2, 2)				# Jump to the second last byte.
		while fin.read(1) != "\n":	# Until EOL is found...
			fin.seek(-2, 1)			# ...jump back the read byte plus one more.
		last = fin.readline()		# Read last line.
		executionTime = float(last.split(',', 1)[0])
		print executionTime
		fin.seek(0,0)
		with open(outputName, 'w') as fout:
			lines = 0
			for line in fin:
				if lines > 0:
					fout.write(str(float(line.split(',', 1)[0]) * outputLength / executionTime) + ',')
					fout.write(line.split(',', 1)[1])
					lines += 1
				else:
					fout.write(line)
					lines += 1
