
#include <iostream>
#include <limits>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>
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
_found_table(false), _mask(0)
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

        for(unsigned int iii = 0; iii < cluster_indices.size() && !_found_table; ++iii)
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZINormal>());
            pcl::ExtractIndices<pcl::PointXYZINormal> extract_cluster;
            extract_cluster.setInputCloud(cloud_plane);
            pcl::PointIndices::Ptr idxs(&cluster_indices[iii], NullDeleter());
            extract_cluster.setIndices(idxs);
            extract_cluster.setNegative(false);
            extract_cluster.filter(*cloud_cluster);
            if(cloud_cluster->points.size() < 500)
                continue;
        
            // Compute principal axis.
            Eigen::Vector4f centroid;
            Eigen::Matrix3f evecs;
            Eigen::Vector3f evals;
            Eigen::Matrix3f covariance_matrix;
            
                
            // TODO debug
            /*
            cloud_cluster->points.clear();
            cloud_cluster->points.resize(3000);
            //evecs.row(0) = Eigen::Vector3f(0.996479, -0.0642191, -0.0538977);
            //evecs.row(1) = Eigen::Vector3f(-0.0139775,  -0.761123,   0.648457);
            //evecs.row(2) = Eigen::Vector3f( 0.0826661,   0.645421,   0.759341);
            evecs.row(0) = Eigen::Vector3f(1, 0, 0);
            evecs.row(1) = Eigen::Vector3f(0,  0,   1);
            evecs.row(2) = Eigen::Vector3f( 0,   -1,   0);
            for(int i = 0; i < evecs.rows(); ++i)
                evecs.row(i) = evecs.row(i).normalized();
            if(evecs(2,2) > 0)
            {
                evecs.row(1) *= -1;
                evecs.row(2) *= -1;
            }
            cout << evecs << endl;
            _table_size = make_float3(0.847947, 0.482838, 0.01);
            centroid = Eigen::Vector4f(0.0162999, 0.0848301, 0.739287, 0);
            for(int k = 0; k < cloud_cluster->points.size(); ++k)
            {
                Eigen::Vector3f pt2 = Eigen::Vector3f((1.0*rand()/RAND_MAX - 0.5)*_table_size.x, (1.0*rand()/RAND_MAX - 0.5)*_table_size.y, 0);
                pt2 = pt2.transpose()*evecs + centroid.head(3).transpose();
                cloud_cluster->points[k].x = pt2[0];
                cloud_cluster->points[k].y = pt2[1];
                cloud_cluster->points[k].z = pt2[2];
                cloud_cluster->points[k].data_c[2] = 0;
            }
            //*/
            
            //evecs = Eigen::MatrixXf::Zero(3,3);
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            // Throw out this plane if it's too far from the camera.
            if(pow(centroid[0], 2) + pow(centroid[1], 2) + pow(centroid[2], 2) > pow(PLANE_MAX_DIS, 2))
                continue;
             
            ///*    
            Eigen::MatrixXf ptmat(3, cloud_cluster->points.size());
            for(int i = 0; i < cloud_cluster->points.size(); ++i)
                ptmat.col(i) << Eigen::Vector3f(cloud_cluster->points[i].x-centroid[0], cloud_cluster->points[i].y-centroid[1], cloud_cluster->points[i].z-centroid[2]);
            Eigen::JacobiSVD<Eigen::MatrixXf> svd = ptmat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeThinV);
            evecs = svd.matrixU().transpose();
            //*/
             
            /*
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
            //*/
            
            /*
            Eigen::Vector3d normd(0,0,0);
            for(int i = 0; i < 10000; ++i)
            {
                int j1 = rand() % cloud_cluster->points.size();
                int j2 = rand() % cloud_cluster->points.size();
                int j3 = rand() % cloud_cluster->points.size();
                // Check to make sure no point was sampled twice.
                if(j1 == j2 || j1 == j3)
                {
                    --i;
                    continue;
                }
                Eigen::Vector3d p1(cloud_cluster->points[j1].x, cloud_cluster->points[j1].y, cloud_cluster->points[j1].z);
                Eigen::Vector3d p2(cloud_cluster->points[j2].x, cloud_cluster->points[j2].y, cloud_cluster->points[j2].z);
                Eigen::Vector3d p3(cloud_cluster->points[j3].x, cloud_cluster->points[j3].y, cloud_cluster->points[j3].z);
                // Check to make sure the lines aren't colinear.
                Eigen::Vector3d l1 = (p2 - p1).normalized();
                Eigen::Vector3d l2 = (p3 - p1).normalized();
                if(l1 == l2)
                {
                    --i;
                    continue;
                }
                Eigen::Vector3d n = (l1.cross(l2)).normalized();
                normd += n;
            }
            normd /= 10000;
            Eigen::Vector3f norm(normd[0], normd[1], normd[2]);
            evecs.row(2) = norm;
            evecs.row(0) = Eigen::Vector3f(0,0,1).cross(evecs.row(2)).normalized();
            evecs.row(1) = evecs.row(2).cross(evecs.row(0)).normalized();
            //*/
            
            // Make the Z axis be the cross-product of X and Y.
            //evecs.col(2) << evecs.col(0).cross(evecs.col(1)).normalized();
            //evecs.col(2) << Eigen::Vector3f(-coefficients->values[0], -coefficients->values[1], -coefficients->values[2]).normalized();
            
            
            ///*
            // Check for left-handed coordinates.
            if(evecs.row(0).cross(evecs.row(1)).dot(evecs.row(2)) < 0)
                evecs.row(1) *= -1;
            // Make sure z points up and not down.
            if(evecs(2,2) > 0)
            {
                evecs.row(1) *= -1;
                evecs.row(2) *= -1;
            }
            // Rotate around the z axis so that y points direcly away from the camera.
            evecs.row(0) = Eigen::Vector3f(0,0,1).cross(evecs.row(2)).normalized();
            evecs.row(1) = evecs.row(2).cross(evecs.row(0)).normalized();
            // Normalize the basis vectors.
            for(int i = 0; i < evecs.rows(); ++i)
                evecs.row(i) = evecs.row(i).normalized();
            cout << evecs << endl;
            cout << centroid.transpose() << endl;
            
            Eigen::Vector3f min_p;
            min_p[0] = min_p[1] = min_p[2] = numeric_limits<double>::infinity();
            Eigen::Vector3f max_p;
            max_p[0] = max_p[1] = max_p[2] = -numeric_limits<double>::infinity();
            
            Eigen::Matrix3f inve = evecs.inverse();
            for(size_t i = 0; i < cloud_cluster->points.size(); ++i)
            {
                Eigen::Vector3f pt(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z);
                pt = (pt - centroid.head(3)).transpose()*inve;
                
                min_p[0] = min(pt[0], min_p[0]);
                min_p[1] = min(pt[1], min_p[1]);
                min_p[2] = min(pt[2], min_p[2]);
                max_p[0] = max(pt[0], max_p[0]);
                max_p[1] = max(pt[1], max_p[1]);
                max_p[2] = max(pt[2], max_p[2]);
            }
            
            // Throw out this plane if it's not the right size.
            double area = (max_p[0] - min_p[0])*(max_p[1] - min_p[1]);
            //if(area < PLANE_MIN_AREA || area > PLANE_MAX_AREA)
            //    continue;
            
            Eigen::Vector3f pt(0, 0, 0);
            double shift_x = (min_p[0] + max_p[0])/2.0 - pt[0];
            double shift_y = (min_p[1] + max_p[1])/2.0 - pt[1];
            centroid[0] += shift_x*evecs(0,0) + shift_y*evecs(1,0);
            centroid[1] += shift_x*evecs(0,1) + shift_y*evecs(1,1);
            centroid[2] += shift_x*evecs(0,2) + shift_y*evecs(1,2);
            _table_size = make_float3(max_p[0] - min_p[0], max_p[1] - min_p[1], 0.01);
            cout << _table_size.x << ", " << _table_size.y << ", " << _table_size.z << endl;
            //*/
            
            if(_found_table)
            {
                printf("Found more than one plane for the table. Don't know what to do.");
                return false;
            }
            _found_table = true;
            _table_plane = make_float4(evecs(2,0), evecs(2,1), evecs(2,2), 
                -1.0*(centroid[0]*evecs(2,0) + centroid[1]*evecs(2,1) + centroid[2]*evecs(2,2)));
            Eigen::Matrix3f etran = evecs.transpose();
            _table_pose = dart::SE3(make_float4(etran(0,0), etran(0,1), etran(0,2), centroid[0]),
                                    make_float4(etran(1,0), etran(1,1), etran(1,2), centroid[1]),
                                    make_float4(etran(2,0), etran(2,1), etran(2,2), centroid[2]));
                        
            // The z-axis of the table should be up, which means projecting it onto the z-axis of the
            // camera should result in a negative value. If it's positive, invert the axes.
            //if(_table_pose.r2.z > 0)
            //    _table_pose = _table_pose*dart::SE3FromRotationX(M_PI);
            table_points = cloud_cluster;
            return true;
            /*
            int oldsize = table_points->points.size();
            table_points->points.resize(table_points->points.size() + 300);
            for(int k = oldsize; k < table_points->points.size(); ++k)
            {
                Eigen::Vector3f pt2 = Eigen::Vector3f((1.0*rand()/RAND_MAX - 0.5)*_table_size.x, (1.0*rand()/RAND_MAX - 0.5)*_table_size.y, 0);
                pt2 = pt2.transpose()*evecs + centroid.head(3).transpose();
                table_points->points[k].x = pt2[0];
                table_points->points[k].y = pt2[1];
                table_points->points[k].z = pt2[2];
                table_points->points[k].data_c[2] = 1;
            }
            //*/
            
            
            
        }
        
        extract.setNegative(true);
        extract.filter(*cloud_plane);
        cloud_filtered.swap(cloud_plane);
    
    }
    if(!_found_table)
        printf("No suitable plane for table found.\n");
    return _found_table;
}

dart::SE3 PointcloudProcessor::projectOntoTable(const dart::SE3& pose)
{
    float3 pt_orig = SE3ToTranslation(pose);
    float3 pt_new = projectOntoTable(pt_orig);
    dart::SE3 ret = pose;
    ret.r0.w = pt_new.x;
    ret.r1.w = pt_new.y;
    ret.r2.w = pt_new.z;
    return ret;
}

float3 PointcloudProcessor::projectOntoTable(const float3& pt_orig)
{
    float3 pt_table = dart::SE3Invert(_table_pose)*pt_orig;
    pt_table.z = 0;
    return _table_pose*pt_table;
}

bool PointcloudProcessor::findObjectsOnTable(const dart::DepthSource<ushort,uchar3>* source)
{
    if(!_found_table)
    {
        printf("Cannot find objects on table until the table has been found!\n");
        return false;
    }
    _object_poses.clear();
    _object_sizes.clear();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pts = _constructPointcloud(source);
    vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters = _findObjectClusters(pts);
    dart::SE3 inv_t = dart::SE3Invert(_table_pose);
    for(int i = 0; i < clusters.size(); ++i)
    {
        float3 min_p = make_float3(numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
        float3 max_p = make_float3(-numeric_limits<double>::infinity(), -numeric_limits<double>::infinity(), -numeric_limits<double>::infinity());
        for(size_t j = 0; j < clusters[i]->points.size(); ++j)
        {
            float3 pt = make_float3(clusters[i]->points[j].x, clusters[i]->points[j].y, clusters[i]->points[j].z);
            pt = inv_t*pt;
                    
            min_p.x = min(pt.x, min_p.x);
            min_p.y = min(pt.y, min_p.y);
            min_p.z = min(pt.z, min_p.z);
            max_p.x = max(pt.x, max_p.x);
            max_p.y = max(pt.y, max_p.y);
            max_p.z = max(pt.z, max_p.z);            
        }
        _object_poses.push_back(_table_pose*dart::SE3FromTranslation((min_p + max_p)/2.0));
        _object_sizes.push_back(max_p - min_p);
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr PointcloudProcessor::_constructPointcloud(const dart::DepthSource<ushort,uchar3>* source, float filter_depth)
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
            if(depth > 0.0 && (filter_depth < 0.0 || depth <= filter_depth))
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




vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> PointcloudProcessor::_findObjectClusters(pcl::PointCloud<pcl::PointXYZINormal>::Ptr pts)
{
    // First throw out all points that are not above the table top.
    int invert_z = (_table_pose.r2.z > 0 ? -1 : 1);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    float3 table_centroid = make_float3(_table_pose.r0.w, _table_pose.r1.w, _table_pose.r2.w);
    dart::SE3 inv_t = dart::SE3Invert(_table_pose);
    for(size_t i = 0; i < pts->points.size(); ++i)
    {
        float3 pt = make_float3(pts->points[i].x, pts->points[i].y, pts->points[i].z);
        pt = inv_t*pt;
        pt.z *= invert_z;
        if(pt.z >= PLANE_DISTANCE_THRESHOLD && abs(pt.x) <= _table_size.x/2.0 && abs(pt.y) <= _table_size.y/2.0)
            inliers->indices.push_back(i);            
    }
    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr table_pts(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::ExtractIndices<pcl::PointXYZINormal> extract;
    extract.setInputCloud(pts);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*table_pts);
    
    printf("%lu table points.\n", table_pts->points.size());

    // Cluster the points above the table top.
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree->setInputCloud(table_pts);
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
    ec.setClusterTolerance(OBJECT_CLUSTER_TOLERANCE);
    ec.setMinClusterSize(OBJECT_MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize(2500000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(table_pts);
    ec.extract(cluster_indices);
    
    printf("%lu clusters.\n", cluster_indices.size());
    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr object_pts;
    vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> ret;
    for(unsigned int j = 0; j < cluster_indices.size(); ++j)
    {
        object_pts.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::PointIndices::Ptr idxs(&cluster_indices[j], NullDeleter());
        extract.setInputCloud(table_pts);
        extract.setIndices(idxs);
        extract.setNegative(false);
        extract.filter(*object_pts);
        
        ret.push_back(object_pts);
    }
    
    // Throw out clusters too far from the table.
    dart::SE3 inv_table = dart::SE3Invert(_table_pose);
    for(unsigned int i = 0; i < ret.size();)
    {
        double min_dis = numeric_limits<double>::infinity();
        for(unsigned int j = 0; j < ret[i]->points.size(); ++j)
            min_dis = min(min_dis, (double)(inv_table*make_float3(ret[i]->points[j].x, 
                 ret[i]->points[j].y, ret[i]->points[j].z)).z);
        if(min_dis > OBJECT_TABLE_THRESHOLD)
        {
            ret.erase(ret.begin() + i);
        }
        else
            ++i;
    }
    
      
    
    return ret;
}




















































