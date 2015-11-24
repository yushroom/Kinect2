import sys
import struct
from scipy.misc import imread, imsave
import numpy as np

def dump_image_with_range(depth, path, width, height, lb, ub):
    a = np.array(depth).reshape( (height, width) ).astype(np.float)
    a[a < lb] = lb
    a[a > ub] = ub
    a -= lb
    a *= (255.0 / (ub-lb))
    imsave(path, a.astype(np.uint8))

try:
    src_path    = sys.argv[1]
    dst_path    = sys.argv[2]
    width       = int(sys.argv[3])
    height      = int(sys.argv[4])
    lb          = int(sys.argv[5])
    ub          = int(sys.argv[6])
except:
    print('Usage: gendepthmap.py $src $dest_folder $width $height $lowbound $upbound')
    exit()
src = open(src_path, 'rb')
idx = 0
while True:
    print(idx)
    idx += 1
    trunk = src.read(width*height*2)
    if not trunk:
        break
    depth = struct.unpack_from('<%dH'%(width*height), trunk)
    dump_image_with_range(depth,'%s\depth_%04d.png'%(dst_path, idx),width,height,lb,ub)
