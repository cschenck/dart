

#include "util.h"

__device__ __host__
float3 uvd_to_xyz(float3 uvd, float2 focal_length, float2 pp, int width, int height)
{
    float3 ret;
    ret.x = (uvd.x-pp.x)*uvd.z/focal_length.x;
    ret.y = (uvd.y-pp.y)*uvd.z/focal_length.y;
    ret.z = uvd.z;
    return ret;
}

