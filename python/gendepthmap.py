import sys
import struct
from scipy.misc import imread, imsave
import numpy as np

def dump_image(depth, path, width, height):
    a = np.array(depth).reshape( (height, width) ).astype(np.float)
    imsave(path, a)


try:
    src_path = sys.argv[1]
    dst_path = sys.argv[2]
    width = int(sys.argv[3])
    height = int(sys.argv[4])

except:
    print('Usage: und.py $src $dest $width $height $und_table')
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
    #dump_image(depth,dst_path + 'depth_' + str(idx) + '.png',width,height)
    dump_image(depth,'%s\depth_%04d.png'%(dst_path, idx),width,height)
