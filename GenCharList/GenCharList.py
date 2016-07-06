# coding=utf-8
__author__ = 'andong'

import os
import re

data_dir = r'E:\split\GB6763-YH639'
full_list_file = 'fulllist.txt'
test_list_file = 'testlist.txt'
result_dir = r'E:\output'

# 统计指定目录下所有汉字

full_list_file_path = os.path.join(data_dir, full_list_file)
test_list_file_path = os.path.join(data_dir, test_list_file)
exp = re.compile('^GB(\d*)_R.bmp')
char_list = list()

for filename in os.listdir(data_dir):
    result = exp.match(filename)
    if result:
        char_list.append(int(result.group(1)))

char_list.sort()
f = open(full_list_file_path, 'w', encoding='utf-8')
for char in char_list:
    f.write('GB' + str(char) + '\n')
f.close()

# 从列表中去除已有的汉字

result_files = os.listdir(result_dir)
inf = open(full_list_file_path, 'r', encoding='utf-8')
chars = [line.strip() for line in inf]
filtered = chars[:]
for char_name in chars:
    for file in result_files:
        if (file.startswith(char_name + '_') or file.startswith(char_name + 'F')) and not file.endswith('.txt'):
            filtered.remove(char_name)
            break

inf.close()
if os.path.exists(test_list_file_path):
    backup_file_path = os.path.join(data_dir, test_list_file.split('.')[0] + '.backup')
    if os.path.exists(backup_file_path):
        os.remove(backup_file_path)
    os.rename(test_list_file_path, backup_file_path)

outf = open(test_list_file_path, 'w', encoding='utf-8')
for char_name in filtered:
    outf.write(char_name + '\n')
outf.close()
