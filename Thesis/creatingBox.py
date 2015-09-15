import random
f = open("box.obj","w")
maxX = maxY = maxZ = 10
minX = minY = minZ = 0
count = 0
while count < 10000:
	x = str(10.0)
	y = str(random.randrange(minY * 100,maxY * 100)/100.0)
	z = str(random.randrange(minZ * 100,maxZ * 100)/100.0)
	f.write("v "+x + " " + y + " " + z +"\n")

	x = str(random.randrange(minX * 100,maxX * 100)/100.0)
	y = str(10.0)
	z = str(random.randrange(minZ * 100,maxZ * 100)/100.0)
	f.write("v "+x + " " + y + " " + z +"\n")

	x = str(random.randrange(minX * 100,maxX * 100)/100.0)
	y = str(random.randrange(minY * 100,maxY * 100)/100.0)
	z = str(10.0)
	f.write("v "+x + " " + y + " " + z +"\n")

	# x = str(random.randrange(minX * 100,maxX * 100)/100.0)
	# y = str(random.randrange(minY * 100,maxY * 100)/100.0)
	# z = str(0.5)
	# f.write("v "+x + " " + y + " " + z +"\n")

	x = str(0.0)
	y = str(random.randrange(minY * 100,maxY * 100)/100.0)
	z = str(random.randrange(minZ * 100,maxZ * 100)/100.0)
	f.write("v "+x + " " + y + " " + z +"\n")

	x = str(random.randrange(minX * 100,maxX * 100)/100.0)
	y = str(0.0)
	z = str(random.randrange(minZ * 100,maxZ * 100)/100.0)
	f.write("v "+x + " " + y + " " + z +"\n")

	x = str(random.randrange(minX * 100,maxX * 100)/100.0)
	y = str(random.randrange(minY * 100,maxY * 100)/100.0)
	z = str(0.0)
	f.write("v "+x + " " + y + " " + z +"\n")

	count += 1