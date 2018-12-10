#!/usr/bin/env python
"""
    Helper script to rename the original sdf files containing the cones of the
    map so that the name of the cones contain their color
"""


input_file = open('model.sdf', 'r')
output_file = open('models_output.sdf', 'w')

k = 0
lines = []
name_indexes = []   # indexes of the <link name=...> lines

# Read the file
for line in input_file:
    lines.append(line)
    if ('link' in line) and ('name' in line):
        name_indexes.append(k)
    k += 1

# Modify the name of the links
k = 0
for i in name_indexes:
    if 'blue' in lines[i+5]:
        color = 'blue'
    elif 'yellow' in lines[i+5]:
        color = 'yellow'
    elif 'big' in lines[i+5]:
        color = 'orange'

    lines[i] = "    <link name='cone_{}_{}'>\n".format(color, k)
    k += 1

# Write in the output file
for line in lines:
    output_file.write(line)


input_file.close()
output_file.close()
