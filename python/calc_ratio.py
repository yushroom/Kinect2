import sys
from scipy.misc import imread, imsave

img = imread(sys.argv[1])
total = img.shape[0]*img.shape[1]
cnt = 0
for i in img:
    for j in i:
        if j != 0:
            cnt += 1
for [i,j] in img:
    print(j)
print('%d/%d'%(cnt, total))