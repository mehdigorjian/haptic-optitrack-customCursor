import os
fileName = "grip4.obj"
filePath = "Models/" + fileName
file1 = open(filePath, 'r')

newFilePath = os.path.splitext(
    filePath)[0] + '1' + os.path.splitext(filePath)[1]
file2 = open(newFilePath, 'w')

for line in file1.readlines():
    if (line.startswith('v') or line.startswith('f') or line.startswith('#')):
        file2.write(line)

file2.close()
file1.close()

if os.path.exists(filePath):
    os.remove(filePath)
else:
    print("The file does not exist")
