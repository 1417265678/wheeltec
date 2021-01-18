#!/usr/bin/env python
# encoding: utf-8
if __name__ == '__main__':
    with open('point_time.txt', 'r+') as f:
        s = [i[:-1].split(',') for i in f.readlines()]
    print(s)
    print(str(s[1]))