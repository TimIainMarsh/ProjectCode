	
for files in range(449):
	lines = []
	f = open('mesh'+str(files)+'.ply','r')
	for line in f:
		x = line.split(' ')
		if len(x) == 4:
			continue
		else:
			lines.append(line)


	lines = lines[10:]
	count = 0
	vertex = len(lines)
	for i in range(len(lines)):
		lines.append(str(i) + " " + str(i+1) + "     \n" )
		count +=1
	lines.append(str(vertex) + " " + str(0) + "     \n" )

	f2 = open("fixed/Zmesh" + str(files) + ".ply","w")

	f2.write("ply\n")
	f2.write("format ascii 1.0\n")
	f2.write("comment PCL generated\n")
	f2.write("element vertex " + str(vertex) + "\n")
	f2.write("property float x"+ "\n")
	f2.write("property float y"+ "\n")
	f2.write("property float z"+ "\n")
	f2.write("element edge " + str(count)+ "\n")
	f2.write("property int vertex1"+ "\n")
	f2.write("property int vertex2"+ "\n")
	f2.write("end_header"+ "\n")

	for length in lines:
		f2.write(length)

	print("finished " + str(files))