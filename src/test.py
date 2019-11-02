import csv 

with open("output.csv", "w") as f: 
    writer = csv.writer(f)
    writer.writerow(["1", "2", "3", "4"])
f.close()