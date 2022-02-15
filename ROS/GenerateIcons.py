

file_path = "/home/rafa/AerostackUI/MapApp/static/scripts/map/img/icons/LandPoint.svg"

#read input file
fin = open(file_path, "rt")

#read file contents to string
data = fin.read()

#replace all occurrences of the required string
#data = data.replace('style="fill:#000000;', 'style="fill:#FF0000;')

print(data)

#close the input file
fin.close()

"""
#open the input file in write mode
fin = open("data.svg", "wt")

#overrite the input file with the resulting data
fin.write(data)

#close the file
fin.close()
"""