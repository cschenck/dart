#!/usr/bin/env python

import os
import numpy as np
import struct
import sys

def make_face(v1, v2, v3):
    v1 = np.array(v1)
    v2 = np.array(v2)
    v3 = np.array(v3)
    n = np.cross((v2 - v1),(v3 - v1))
    n /= np.linalg.norm(n)
    return [n[0], n[1], n[2],
            v1[0], v1[1], v1[2],
            v2[0], v2[1], v2[2],
            v3[0], v3[1], v3[2],]


def make_cube(pt, res):
    pts = [pt,
           (pt[0] + res, pt[1], pt[2]),
           (pt[0] + res, pt[1] + res, pt[2]),
           (pt[0], pt[1] + res, pt[2]),
           (pt[0], pt[1] + res, pt[2] + res),
           (pt[0] + res, pt[1] + res, pt[2] + res),
           (pt[0] + res, pt[1], pt[2] + res),
           (pt[0], pt[1], pt[2] + res),]
    ret = []
    ret.append(make_face(pts[0], pts[1], pts[2]))
    ret.append(make_face(pts[0], pts[1], pts[3]))
    ret.append(make_face(pts[7], pts[0], pts[3]))
    ret.append(make_face(pts[7], pts[3], pts[4]))
    ret.append(make_face(pts[1], pts[6], pts[5]))
    ret.append(make_face(pts[1], pts[5], pts[2]))
    ret.append(make_face(pts[3], pts[2], pts[5]))
    ret.append(make_face(pts[3], pts[5], pts[4]))
    ret.append(make_face(pts[7], pts[6], pts[1]))
    ret.append(make_face(pts[7], pts[1], pts[0]))
    ret.append(make_face(pts[6], pts[7], pts[4]))
    ret.append(make_face(pts[6], pts[4], pts[5]))
    return ret

global_index = 1
def get_index(vset, v):
    global global_index
    if v not in vset:
        vset[v] = global_index
        global_index = global_index + 1
    return vset[v]  


input_fp = sys.argv[1]
output_fp = sys.argv[2]

bts = open(input_fp, "rb").read()
dim = (struct.unpack("<L", bts[:4])[0], struct.unpack("<L", bts[4:8])[0], struct.unpack("<L", bts[8:12])[0])
offset = (struct.unpack("f", bts[12:16])[0], struct.unpack("f", bts[16:20])[0], struct.unpack("f", bts[20:24])[0])
resolution = struct.unpack("f", bts[24:28])[0]
data = np.fromstring(bts[28:], dtype=np.float32)
data = data.reshape(dim)

faces = []
for x in range(data.shape[0]):
    for y in range(data.shape[1]):
        for z in range(data.shape[2]):
            v = data[x,y,z]
            if v <= resolution*10:
                faces += make_cube((x, y, z), resolution)


verts = dict()
fcs = []
for face in faces:
    v1 = get_index(verts, tuple(face[3:6]))
    v2 = get_index(verts, tuple(face[6:9]))
    v3 = get_index(verts, tuple(face[9:12]))
    fcs.append([v1, v2, v3])


fp = open(output_fp, "w")

for v in verts:
    fp.write("v %f %f %f\n" % (v[0], v[1], v[2]))
#for f in fcs:
#    fp.write("f %d %d %d\n" % (f[0], f[1], f[2]))

fp.close()
