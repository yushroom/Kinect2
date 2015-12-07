import sys
from scipy.misc import imread, imsave

try:
    img1_path   = sys.argv[1]
    img2_path   = sys.argv[2]
    rt_path     = sys.argv[3]
except:
    print('Usage: diffImg.py $img1 $img2 $result')
    exit()

try:
    a = imread(img1_path)
    b = imread(img2_path)
except:
    print('Error: file not exist.')
    exit()

if a.shape != b.shape:
    print('Error: images have different shapes')
    exit() 
imsave(rt_path, a-b)

