# -- imports --
import time
import datetime


# -- file paths --
fin = open('/Users/paul/Downloads/Julian1_20151212-143221.csv')
fout = open('/Users/paul/Workspace/PenguinTracking/data/gps_01.csv', 'w')

content = fin.readlines()[1:]
for line in content:
    tokens = line.split(',')
    # convert datetime to posix time
    mdate = tokens[0]
    mtime = tokens[1]
    tstring = ''.join((mdate, ' ', mtime))
    tposix = time.mktime(datetime.datetime.strptime(tstring, "%Y/%m/%d %H:%M:%S").timetuple())
    # write pre-processed data to file
    to_file = '%.4f, %s, %s, %s\n' % (tposix, tokens[2], tokens[3], tokens[4])
    fout.write(to_file)

fout.close()