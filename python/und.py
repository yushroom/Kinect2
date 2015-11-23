import sys
import struct

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def read_und_lookup_table(file):
    content = file.readlines()
    und_table = [None]*(width*height)
    for line in content:
        a = int(line.split()[0])
        b = int(line.split()[1])
        c = float(line.split()[2])
        d = float(line.split()[3])
        und_table[a+b*width] = (c, d)
    return und_table

def bilinear(x, y, src):
    ix0 = int(x) 
    iy0 = int(y) 
    ix1 = int(ix0 + 1) 
    iy1 = int(iy0 + 1)
    qx1 = x - ix0 
    qy1 = y - iy0
    qx0 = 1.0 - qx1 
    qy0 = 1.0 - qy1

    weight = 0.0 
    value = 0.0
    w = (qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1)
    xx = (ix0, ix1, ix0, ix1)
    yy = (iy0, iy0, iy1, iy1)

    for j in range(4):
        if xx[j] < 0 or xx[j] >= width or yy[j] < 0 or yy[j] >= height:
            continue
        idx = xx[j] + yy[j] * width
        if src[idx] == 0:
            continue
        value += src[idx] * w[j]
        weight += w[j]
    return int(0 if weight == 0.0 else value / weight)

try:
    src_path = sys.argv[1]
    dst_path = sys.argv[2]
    width = int(sys.argv[3])
    height = int(sys.argv[4])
    und_path = sys.argv[5]

except:
    print('Usage: und.py $src $dest $width $height $und_table')
    exit()

try:
    src = open(src_path, 'rb')
    dst = open(dst_path, 'wb')
    und = open(und_path, 'r')
except:
    print('Fail to open file')
    exit()

und_table = read_und_lookup_table(und)

idx = 0
while True:
    print(idx)
    idx += 1
    trunk = src.read(width*height*2)
    if not trunk:
        break
    depth = struct.unpack_from('<%dH'%(width*height), trunk)
    depth_und = [0]*(width*height)
    for i in range(width*height):
        depth_und[i] = bilinear(und_table[i][0], und_table[i][1], depth)
        depth_und[i] = clamp(depth_und[i], 0, 65535)
    depth_und_bytes = struct.pack('<%dH'%(width*height), *depth_und)
    dst.write(depth_und_bytes)
    #print(len(depth_und_bytes))
dst.close()