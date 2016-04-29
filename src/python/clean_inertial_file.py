# -- imports --


# -- file paths --
fin = open('/Users/paul/Downloads/Dinodave swim_selection_1.txt')
fout = open('/Users/paul/Workspace/PenguinTracking/data/inertial_01.csv', 'w')

content = f.readlines()
for line in content:
    tokens = line.split(' ')
    print tokens