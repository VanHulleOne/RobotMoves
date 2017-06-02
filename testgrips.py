# -*- coding: utf-8 -*-
"""
Created on Fri Jun  2 10:41:19 2017

@author: lvanhulle
"""

"""
testgrips.py
"""

import re

zAll = []
# match a string that enclosed in square brackets with three decimal numbers
# ex: [5.660, -0.000, 69.600]
# Store the z value into a group
reg = r"\[-?\d+\.\d+, -?\d+\.\d+, (-?\d+\.\d+)\]"

with open('path_bad.mod', 'r') as file:
    for line in file:
        match = re.search(reg, line)
        if match:
            zAll.append(float(match.group(1)))

# Find the ends to remove those z points
top = max(zAll)
bottom = min(zAll)

# Filter out the top and bottom values
zFillet = [a for a in zAll if a !=top and a!=bottom]

# remove the duplicates
zNoRepeat = [zFillet[0]]
for p in zFillet:
    if p != zNoRepeat[-1]:
        zNoRepeat.append(p)

# The middle is the bottom vs top fillet
middle = int(len(zNoRepeat)/2)

zBottom = zNoRepeat[:middle]
zTop = zNoRepeat[middle:]

diffs = []
steps = []


for i in range(middle-1):
    # Find the difference between steps
    b = zBottom[i] - zBottom[i+1]
    t = zTop[i+1] - zTop[i]
    steps.append(round(t, 7))
    diffs.append(round(abs(b-t), 7))
    if abs(b-t) > 0.002:
        print('Difference between top and bottom found at offset:', i)
        break
else:
    # If the break does not exit the for loop early fun the else statement
    print('All Steps Match')