# coding=utf-8
__author__ = 'andong'

# 统计指定目录下所有汉字

import os
import re

output_file = 'list.txt'
dir_path = r'E:\split\GB6763-FS'
exp = re.compile('^GB(\d*)_R.bmp')
char_list = list()

for filename in os.listdir(dir_path):
    result = exp.match(filename)
    if result:
        print(filename)
        char_list.append(int(result.group(1)))

char_list.sort()
f = open(os.path.join(dir_path, output_file), 'w', encoding='utf-8')
for char in char_list:
    f.write('GB' + str(char) + '\n')
f.close()