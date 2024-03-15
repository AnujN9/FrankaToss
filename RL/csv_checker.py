import csv

with open('ee_pose.csv', mode ='r')as file:
  csvFile = csv.reader(file, delimiter=' ')
  i = 0
  for lines in csvFile: 
        print(lines[0])