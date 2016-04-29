# -- imports --
import time
import datetime


# -- file paths --
fin = open('/Users/paul/Downloads/Dinodave swim_selection_1.txt')
fout = open('/Users/paul/Workspace/PenguinTracking/data/inertial_01.csv', 'w')

content = fin.readlines()[1:]
for line in content:
    tokens = line.split('\t')
    # convert datetime to posix time
    mdate = tokens[8]
    mtime = tokens[9]
    mmsec = tokens[10]
    tstring = ''.join((mdate, ' ', mtime))
    tposix = time.mktime(datetime.datetime.strptime(tstring, "%d/%m/%Y %H:%M:%S").timetuple())
    tposix = tposix + float(mmsec)

    # write pre-processed data to file
    to_file = '%.4f, %s, %s, %s, %s, %s, %s, %s, %s\n' % (tposix, tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6], tokens[7])
    fout.write(to_file)

fout.close()