#ifndef __POINTCLOUD_PROCESSOR_H__
#define __POINTCLOUD_PROCESSOR_H__

#include <vector>

#include <cuda_runtime.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector_types.h>
#include <vector_functions.h>

#include "depth_sources/depth_source.h"
#include "geometry/SE3.h"
#include "tracker.h" // <-- Somewhere in here is a cuda include that I can't seem to track down but is needed here.
#include "util/mirrored_memory.h"

#define OBJECT_TABLE_THRESHOLD 0.05
#define PLANE_DISTANCE_THRESHOLD 0.01
#define PLANE_MAX_DIS 1.5
#define PLANE_MIN_AREA 0.25
#define PLANE_MAX_AREA 1.0
#define DEFAULT_DEPTH_FILTER 1.5
#define OBJECT_CLUSTER_TOLERANCE 0.02
#define OBJECT_MIN_CLUSTER_SIZE 2000

using namespace std;

class PointcloudProcessor
{
public:
    PointcloudProcessor();
    ~PointcloudProcessor();
    
    bool findTable(const dart::DepthSource<ushort,uchar3>* source);
    bool findObjectsOnTable(const dart::DepthSource<ushort,uchar3>* source);
    
    const dart::SE3& getTablePose() const { return _table_pose; }
    const float3& getTableSize() const { return _table_size; }
    int numObjectsOnTable() const { return _object_poses.size(); }
    const dart::SE3& getObjectPose(int i) const { return _object_poses[i]; }
    const float3& getObjectSize(int i) const { return _object_sizes[i]; }
    dart::SE3 projectOntoTable(const dart::SE3& pose);
    float3 projectOntoTable(const float3& pt_orig);
    void computeCloudMask(const dart::DepthSource<ushort,uchar3>* source);
    const int* getDeviceMask() const { return _mask.devicePtr(); }
    const int* getHostMask() { _mask.syncDeviceToHost(); return _mask.hostPtr(); }
    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr table_points;

private:
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr _constructPointcloud(const dart::DepthSource<ushort,uchar3>* source, float filter_depth=DEFAULT_DEPTH_FILTER);
    vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> _findObjectClusters(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pts);

    dart::SE3 _table_pose;
    float3 _table_size;
    float4 _table_plane;
    bool _found_table;
    
    vector<dart::SE3> _object_poses;
    vector<float3> _object_sizes;
    
    dart::MirroredVector<int> _mask;
};





#endif
