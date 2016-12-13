
#include "util.h"

#include "pointcloud_processor.h"


__global__ void gpu_computeCloudMask(const ushort* depth, const int width, const int height, const float2 fl, const float2 pp, const float conversion, const float4 plane, int* mask) 
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x >= width || y >= height)
        return;

    // Run the gauntlet. Set to 0 and return if it's filtered, otherwise set to 1.
    const int index = x + y*width;
    const float z = conversion*depth[index];
    if(z > DEFAULT_DEPTH_FILTER)
    {
        mask[index] = 0;
        return;
    }

    float3 xyz = uvd_to_xyz(make_float3(x, y, z), fl, pp, width, height);
    float d = plane.x*xyz.x + plane.y*xyz.y + plane.z*xyz.z + plane.w;
    if(d < PLANE_DISTANCE_THRESHOLD)
    {
        mask[index] = 0;
        return;
    }
    
    mask[index] = 1;
}


void PointcloudProcessor::computeCloudMask(const dart::DepthSource<ushort,uchar3>* source)
{
    uint width = source->getDepthWidth();
    uint height = source->getDepthHeight();
    if(_mask.length() != width*height)
        _mask.resize(width*height);
    float2 fl = source->getFocalLength();
    float2 pp = source->getPrincipalPoint();
    const ushort* ptr = source->getDeviceDepth();
    dim3 block(16,8,1);
    dim3 grid( ceil( width / (float)block.x), ceil( height / (float)block.y ));
    
    float4 plane = _table_plane;
    plane /= sqrtf(plane.x*plane.x + plane.y*plane.y + plane.z*plane.z);

    gpu_computeCloudMask<<<grid,block>>>(ptr,width,height,fl, pp, source->getScaleToMeters(), plane, _mask.devicePtr());
}
