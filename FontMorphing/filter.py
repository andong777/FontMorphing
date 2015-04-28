# coding=utf-8
__author__ = 'andong'

# 从列表中去除已有的汉字

import os

list_file_path = r'E:\split\GB6763-YH639\testlist.txt'
result_dir_path = r'E:\output'
suffix = '_5_0.jpg'

inf = open(list_file_path, 'r', encoding='utf-8')
dir_path, list_file = os.path.split(list_file_path)
chars = [line.strip() for line in inf]
filtered = []
for char_name in chars:
    file_name = os.path.join(result_dir_path, char_name + suffix)
    if not os.path.exists(file_name):
        filtered.append(char_name)

inf.close()
# os.remove(list_file_path)
os.rename(list_file_path, os.path.join(dir_path, list_file.split('.')[0] + '.backup'))

outf = open(os.path.join(dir_path, list_file_path), 'w', encoding='utf-8')
for char_name in filtered:
    outf.write(char_name + '\n')
outf.close()
