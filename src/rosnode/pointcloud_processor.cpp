
#include <math.h>
#include <stdio.h>

#include <Eigen/Geometry> 
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "util.h"

#include "pointcloud_processor.h"


PointcloudProcessor::PointcloudProcessor() :
_found_table(false)
{
}

PointcloudProcessor::~PointcloudProcessor()
{
}

struct NullDeleter {template<typename T> void operator()(T*) {} };

bool PointcloudProcessor::findTable(const dart::DepthSource<ushort,uchar3>* source)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pts = _constructPointcloud(source);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZINormal>());
    
    // First downsize the point cloud.    
    pcl::VoxelGrid<pcl::PointXYZINormal> vg;
    vg.setInputCloud(pts);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    
    // Now find the plane.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZINormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(PLANE_DISTANCE_THRESHOLD);
    seg.setMaxIterations(1000);
    
    _found_table = false;

    while(!_found_table)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        
        if(inliers->indices.size() < 100)
            break;
        
        // Extract the planar inliers from the input cloud
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::ExtractIndices<pcl::PointXYZINormal> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        
        // Cluster the points in the plane to find separate planes.
        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
        tree->setInputCloud(cloud_plane);
        vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_plane);
        ec.extract(cluster_indices);

        for(unsigned int i = 0; i < cluster_indices.size() && !_found_table; ++i)
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZINormal>());
            pcl::ExtractIndices<pcl::PointXYZINormal> extract_cluster;
            extract_cluster.setInputCloud(cloud_plane);
            pcl::PointIndices::Ptr idxs(&cluster_indices[i], NullDeleter());
            extract_cluster.setIndices(idxs);
            extract_cluster.setNegative(false);
            extract_cluster.filter(*cloud_cluster);
        
            // Compute principal axis.
            Eigen::Vector4f centroid;
            Eigen::Matrix3f evecs;
            Eigen::Vector3f evals;
            Eigen::Matrix3f covariance_matrix;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            
            // Throw out this plane if it's too far from the camera.
            if(pow(centroid[0], 2) + pow(centroid[1], 2) + pow(centroid[2], 2) > pow(PLANE_MAX_DIS, 2))
                continue;
             
            pcl::computeCovarianceMatrix(*cloud_cluster, centroid, covariance_matrix);
            pcl::eigen33(covariance_matrix, evecs, evals);
            
            // Sort the eigen vectors in descending order by their eigen values.
            for(int i = 0; i < 3; ++i)
            {
                for(int j = i + 1; j < 3; ++j)
                {
                    if(evals[j] > evals[i])
                    {
                        double ev = evals[j];
                        Eigen::Vector3f vec;
                        vec << evecs.col(j);
                        evals[j] = evals[i];
                        evecs.col(j) << evecs.col(i);
                        evals[i] = ev;
                        evecs.col(i) << vec;
                    }
                }
            }
            
            // Make the Z axis be the cross-product of X and Y.
            evecs.col(2) << evecs.col(0).cross(evecs.col(1)).normalized();
            // Make Y point in the same direction as the camera Z but on the plane.
            evecs.col(1) = (Eigen::Vector3f(0,0,1) - evecs.col(2).dot(Eigen::Vector3f(0,0,1))*evecs.col(2)).normalized();
            // Finally, make X the cross-product of Y and Z.
            evecs.col(0) << evecs.col(1).cross(evecs.col(2)).normalized();
            
            Eigen::Vector3f min_p;
            min_p[0] = min_p[1] = min_p[2] = numeric_limits<double>::infinity();
            Eigen::Vector3f max_p;
            max_p[0] = max_p[1] = max_p[2] = -numeric_limits<double>::infinity();
            
            for(size_t i = 0; i < cloud_cluster->points.size(); ++i)
            {
                Eigen::Vector3f pt(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z);
                pt = pt.transpose()*evecs;
                
                min_p[0] = min(pt[0], min_p[0]);
                min_p[1] = min(pt[1], min_p[1]);
                min_p[2] = min(pt[2], min_p[2]);
                max_p[0] = max(pt[0], max_p[0]);
                max_p[1] = max(pt[1], max_p[1]);
                max_p[2] = max(pt[2], max_p[2]);
            }
            
            // Throw out this plane if it's not the right size.
            double area = (max_p[0] - min_p[0])*(max_p[1] - min_p[1]);
            if(area < PLANE_MIN_AREA || area > PLANE_MAX_AREA)
                continue;
            
            Eigen::Vector3f pt(centroid[0], centroid[1], centroid[2]);
            pt = pt.transpose()*evecs;
            double shift_x = (min_p[0] + max_p[0])/2.0 - pt[0];
            double shift_y = (min_p[1] + max_p[1])/2.0 - pt[1];
            centroid[0] += shift_x*evecs(0,0) + shift_y*evecs(0,1);
            centroid[1] += shift_x*evecs(1,0) + shift_y*evecs(1,1);
            centroid[2] += shift_x*evecs(2,0) + shift_y*evecs(2,1);
            
            if(_found_table)
            {
                printf("Found more than one plane for the table. Don't know what to do.");
                return false;
            }
            _found_table = true;
            _table_size = make_float3(max_p[0] - min_p[0], max_p[1] - min_p[1], 0.01);
            _table_pose = dart::SE3(make_float4(evecs(0,0), evecs(1,0), evecs(2,0), centroid[0]),
                                    make_float4(evecs(0,1), evecs(1,1), evecs(2,1), centroid[1]),
                                    make_float4(evecs(0,2), evecs(1,2), evecs(2,2), centroid[2]));
            _table_plane = make_float4(evecs(0,2), evecs(1,2), evecs(2,2), 
                -1.0*(centroid[0]*evecs(0,2) + centroid[1]*evecs(1,2) + centroid[2]*evecs(2,2)));
                        

            
            
        }
        
        extract.setNegative(true);
        extract.filter(*cloud_plane);
        cloud_filtered.swap(cloud_plane);
    
    }
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr PointcloudProcessor::_constructPointcloud(const dart::DepthSource<ushort,uchar3>* source)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pts(new pcl::PointCloud<pcl::PointXYZINormal>());
    uint width = source->getDepthWidth();
    uint height = source->getDepthHeight();
    pts->points.resize(width*height);
    float2 fl = source->getFocalLength();
    float2 pp = source->getPrincipalPoint();
    const ushort* ptr = source->getDepth();
    for(int i = 0; i < height; ++i)
    {
        for(int j = 0; j < width; ++j)
        {
            int k = i*width + j;
            double depth = 1.0*source->getScaleToMeters()*ptr[k];

            pcl::PointXYZINormal& p = pts->points[k];
            if(depth > 0.0)
            {
                float3 pt = uvd_to_xyz(make_float3(j, i, depth), fl, pp, width, height);
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z;
            }
            else
            {
                p.x = 0.0;
                p.y = 0.0;
                p.z = -1.0;
            }
            p.data_c[0] = i;
            p.data_c[1] = j;
        }
    }
    pts->width = width;
    pts->height = height;
    
    return pts;
}
