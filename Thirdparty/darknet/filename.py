# coding=utf-8
import os

input_ = open('./00imagefile.txt', 'r')
output  = open('src.txt', 'w')

for line in input_:
    output.write('src/'+ line)
    
output.close()
input_.close()


