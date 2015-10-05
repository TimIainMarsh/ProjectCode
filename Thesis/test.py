f2 = open("fixed/objtest.obj","w")

for files in range(100):
	lines = []
	try:
		f = open('mesh'+str(files)+'.ply','r')
		for line in f:
			x = line.split(' ')
			if len(x) == 4:
				continue
			else:
				lines.append(line)
	except FileNotFoundError:
		continue


	lines = lines[10:]
	count = 0

	f2.write("o Object."+ str(files)+"\n")

	f2.write("# List of geometric vertices, with (x,y,z[,w]) coordinates, w is optional and defaults to 1.0.\n")
	for l in lines:
		f2.write("v " + l)

	face = "l"
	for i in range(1,len(lines)):
		face = face + " " + str(i)

	f2.write("")

	f2.write("# Polygonal face element (see below)"+"\n")

	f2.write("")

	f2.write(face+"\n")

	f2.write("# End of object\n")

	print("finished with " + str(files))