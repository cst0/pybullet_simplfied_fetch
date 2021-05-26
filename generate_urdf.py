#!/usr/bin/env python3
import os

def parse_file(filepath:str):
    filepath = os.path.abspath(filepath)
    inputfile = open(filepath, 'r')
    outputfile = open(''.join(filepath.split('.')[:-1]) + '.urdf', 'w+')
    for line in inputfile.readlines():
        line = line.replace('$(cwd)', os.getcwd()+'/simple_grasping/resources')
        outputfile.write(line)

    print('done!')
    inputfile.close()
    outputfile.close()


def main():
    """ remove the xacro dependency by hacking together an implementation that's good enough for what we need. """
    parse_file('./simple_grasping/resources/simplefetch.xacro')


if __name__ == '__main__':
    main()

