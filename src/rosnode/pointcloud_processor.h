#ifndef __POINTCLOUD_PROCESSOR_H__
#define __POINTCLOUD_PROCESSOR_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector_types.h>

#include "depth_sources/depth_source.h"
#include "geometry/SE3.h"

#define PLANE_DISTANCE_THRESHOLD 0.02
#define PLANE_MAX_DIS 1.5
#define PLANE_MIN_AREA 0.25
#define PLANE_MAX_AREA 1.0

class PointcloudProcessor
{
public:
    PointcloudProcessor();
    ~PointcloudProcessor();
    
    bool findTable(const dart::DepthSource<ushort,uchar3>* source);
    bool findObjectOnTable(const dart::DepthSource<ushort,uchar3>* source);

private:
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr _constructPointcloud(const dart::DepthSource<ushort,uchar3>* source);

    dart::SE3 _table_pose;
    float3 _table_size;
    float4 _table_plane;
    bool _found_table;
};





#endif
