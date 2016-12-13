
#include <stdlib.h>



#include "util.h"

#include "marker_publisher.h"



using namespace std;

#ifdef __linux__
#include <arpa/inet.h>
#include <dirent.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
bool set_default_url(string* url, string* web_dir)
{
    // First get the local ip address.
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];
    if(getifaddrs(&ifaddr) == -1) 
    {
        printf("getifaddrs() failed.\n");
        return false;
    }
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) 
    {
        if (ifa->ifa_addr == NULL)
            continue;
         s=getnameinfo(ifa->ifa_addr,sizeof(struct sockaddr_in),host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

        if((strcmp(host,"127.0.0.1")!=0)&&(ifa->ifa_addr->sa_family==AF_INET))
        {
            if (s != 0)
            {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                return false;
            }
            //printf("\tInterface : <%s>\n",ifa->ifa_name );
            //printf("\t  Address : <%s>\n", host); 
            break;
        }
    }
    freeifaddrs(ifaddr);
    
    char username[64];
    getlogin_r(username, 64);
    const char* homedir;
    if((homedir = getenv("HOME")) == NULL)
    {
        printf("HOME environment variable not defined.\n");
        return false;
    }
    
    *web_dir = combine_paths(homedir, "public_html");
    struct stat st = {0};
    if(stat(web_dir->c_str(), &st) == -1)
    {
        printf("Unable to open the public_html folder in this user's home directory. Please ensure you are running a web server on this machine.\n");
        return false;
    }
    
    *web_dir = combine_paths(*web_dir, "meshes");
    if(stat(web_dir->c_str(), &st) == -1)
    {
        mkdir(web_dir->c_str(), 0777);
    }
    
    *url = string("http://") + host + "/~" + username + "/meshes";
    
    return true;
}
#else
bool set_default_url(string* url, string* web_dir)
{
    printf("Default url and web_dir not available. Please set these arguments by hand.\n");
    return false;
}
#endif


MarkerPublisher::MarkerPublisher(string url, string web_dir, string topic) :
    _url(url), _web_dir(web_dir), _pc_pub(NULL), _cloud_mask(NULL), _show_sdfs_instead(false)
{
    _ros_node = new ros::NodeHandle();
    _pub = new ros::Publisher(_ros_node->advertise<visualization_msgs::MarkerArray>(topic, 1));
}

// Helper function.
int is_big_endian(void)
{
    union {
        uint32_t i;
        char c[4];
    } bint = {0x01020304};

    return bint.c[0] == 1; 
}
MarkerPublisher::MarkerPublisher(string url, string web_dir, string topic, string pointcloud_topic) :
MarkerPublisher(url, web_dir, topic)
{
    _pc_pub = new ros::Publisher(_ros_node->advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1));
    _pts.reset(new sensor_msgs::PointCloud2);
    _pts->is_bigendian = is_big_endian();
    _pts->fields.resize(4);
    _pts->fields[0].name = "x";
    _pts->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    _pts->fields[0].offset = 0;
    _pts->fields[0].count = 1;
    _pts->fields[1].name = "y";
    _pts->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    _pts->fields[1].offset = 4;
    _pts->fields[1].count = 1;
    _pts->fields[2].name = "z";
    _pts->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    _pts->fields[2].offset = 8;
    _pts->fields[2].count = 1;
    _pts->fields[3].name = "rgb";
    _pts->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    _pts->fields[3].offset = 16;
    _pts->fields[3].count = 1;
    _pts->point_step = 32;
    _pts->is_dense = false;
}

MarkerPublisher::~MarkerPublisher()
{
    _pub->shutdown();
    if(_pc_pub != NULL)
    {
        _pc_pub->shutdown();
        delete _pc_pub;
    }
    delete _pub;
    delete _ros_node;
    
}

int MarkerPublisher::_indexFromID(int marker_id)
{
    for(int i = 0; i < _markers.markers.size(); ++i)
    {
        if(_markers.markers[i].id == marker_id)
            return i;
    }
    return -1;
}

int MarkerPublisher::addUntrackedObject(const dart::SE3& pose, const float3& size)
{
    float3 color;
    int ret = _getFreeMarkerID();
    ///*
    _markers.markers.resize(_markers.markers.size() + 1);
    visualization_msgs::Marker& marker = _markers.markers[_markers.markers.size()-1];
    marker.id = ret;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size.x;
    marker.scale.y = size.y;
    marker.scale.z = size.z;
    marker.color.a = 0.5;
    color = make_float3(1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX);
    marker.color.r = color.x;
    marker.color.g = color.y;
    marker.color.b = color.z;
    updateUntrackedObject(ret, pose);
    //*/
    
    /*int id = _getFreeMarkerID();
    _markers.markers.resize(_markers.markers.size() + 1);
    visualization_msgs::Marker& marker2 = _markers.markers[_markers.markers.size()-1];
    marker2.id = id;
    marker2.type = visualization_msgs::Marker::POINTS;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.scale.x = 0.02;
    marker2.scale.y = 0.02;
    marker2.scale.z = 0.02;
    marker2.color.a = 1.0;
    color = make_float3(1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX);
    marker2.color.r = color.x;
    marker2.color.g = color.y;
    marker2.color.b = color.z;
    marker2.points.resize(300);
    for(int k = 0; k < marker2.points.size(); ++k)
    {
        float3 pt = make_float3((1.0*rand()/RAND_MAX - 0.5)*size.x, (1.0*rand()/RAND_MAX - 0.5)*size.y, 0);
        pt = pose*pt;
        marker2.points[k].x = pt.x;
        marker2.points[k].y = pt.y;
        marker2.points[k].z = pt.z;
    }*/
    
    
    for(int i = 0; i < 3; ++i)
    {
        float k = 0.2; // optional to scale the length of the arrows
        // Unit vector on the axis.
        float3 v2 = make_float3(i==0?k:0,i==1?k:0,i==2?k:0);
        v2 = pose*v2;
        float3 v1 = make_float3(0,0,0);
        v1 = pose*v1;
        
        visualization_msgs::Marker marker3;
        marker3.id =_getFreeMarkerID();
        marker3.type = visualization_msgs::Marker::ARROW;
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.scale.x = 0.01;
        marker3.scale.y = 0.01;
        marker3.scale.z = 0.01;
        
        geometry_msgs::Point p;
        p.x = v1.x;
        p.y = v1.y;
        p.z = v1.z;
        marker3.points.push_back(p);
        p.x = v2.x;
        p.y = v2.y;
        p.z = v2.z;
        marker3.points.push_back(p);
        
        marker3.color.a = 1.0;
        marker3.color.r = (i == 0 ? 1.0 : 0.0);
        marker3.color.g = (i == 1 ? 1.0 : 0.0);
        marker3.color.b = (i == 2 ? 1.0 : 0.0);
        _markers.markers.push_back(marker3);
    }
    
    return ret;
}

int MarkerPublisher::addUntrackedObject(pcl::PointCloud<pcl::PointXYZINormal>::Ptr source, float filter)
{
    int ret = _getFreeMarkerID();
    _markers.markers.resize(_markers.markers.size() + 1);
    visualization_msgs::Marker& marker = _markers.markers[_markers.markers.size()-1];
    marker.id = ret;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    float3 color = make_float3(1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX);
    marker.color.r = color.x;
    marker.color.g = color.y;
    marker.color.b = color.z;
    //marker.points.resize(source->points.size());
    for(int k = 0; k < source->points.size(); ++k)
    {
        if(filter < 0 || source->points[k].data_c[2] == filter)
        {
            marker.points.resize(marker.points.size() + 1);
            int kk = marker.points.size() - 1;
            marker.points[kk].x = source->points[k].x;
            marker.points[kk].y = source->points[k].y;
            marker.points[kk].z = source->points[k].z;
        }
    }
    return ret;
}

void MarkerPublisher::updateUntrackedObject(int id, const dart::SE3& pose)
{
    visualization_msgs::Marker& marker = _markers.markers[_indexFromID(id)];
    float4 rot = SE3ToQuaternion(pose);
    float3 tran = SE3ToTranslation(pose);
    marker.pose.position.x = tran.x;
    marker.pose.position.y = tran.y;
    marker.pose.position.z = tran.z;
    marker.pose.orientation.x = rot.x;
    marker.pose.orientation.y = rot.y;
    marker.pose.orientation.z = rot.z;
    marker.pose.orientation.w = rot.w;
}

void MarkerPublisher::update(const ObjectInterface& tracker, std_msgs::Header header)
{
    for(int i = 0; i < _meshes.size(); ++i)
        _meshes[i].exists = false;
        
    for(int obj = 0; obj < tracker.numObjects(); ++obj)
    {
        string obj_name = tracker.objectName(obj);
        for(int frame = 0; frame < tracker.numFrames(obj); ++frame)
        {
            if(_show_sdfs_instead)
            {
                int ii = _getMeshMarker(obj_name, frame, 0);
                if(ii < 0)
                    ii = _addNewSdfMarker(tracker, obj_name, frame, 0);
                mesh_marker& mesh = _meshes[ii];
                visualization_msgs::Marker& marker = _markers.markers[_indexFromID(mesh.marker_id)];
                dart::SE3 t = tracker.objectTransform(obj)*tracker.frameTransform(obj, frame);
                float4 rot = SE3ToQuaternion(t);
                float3 tran = SE3ToTranslation(t);
                marker.pose.position.x = tran.x;
                marker.pose.position.y = tran.y;
                marker.pose.position.z = tran.z;
                marker.pose.orientation.x = rot.x;
                marker.pose.orientation.y = rot.y;
                marker.pose.orientation.z = rot.z;
                marker.pose.orientation.w = rot.w;
                mesh.exists = true;
            }
            else
            {
                for(int idx = 0; idx < tracker.numGeoms(obj, frame); ++idx)
                {
                    int geom = tracker.geomID(obj, frame, idx);
                    int ii = _getMeshMarker(obj_name, frame, geom);
                    if(ii < 0)
                        ii = _addNewMeshMarker(tracker, obj_name, frame, geom);
                    mesh_marker& mesh = _meshes[ii];
                    visualization_msgs::Marker& marker = _markers.markers[_indexFromID(mesh.marker_id)];
                    dart::SE3 t = tracker.objectTransform(obj)*tracker.frameTransform(obj, frame)*tracker.relativeGeomTransform(obj, geom);
                    float4 rot = SE3ToQuaternion(t);
                    float3 tran = SE3ToTranslation(t);
                    marker.pose.position.x = tran.x;
                    marker.pose.position.y = tran.y;
                    marker.pose.position.z = tran.z;
                    marker.pose.orientation.x = rot.x;
                    marker.pose.orientation.y = rot.y;
                    marker.pose.orientation.z = rot.z;
                    marker.pose.orientation.w = rot.w;
                    mesh.exists = true;
                }
            }
        }
    }
    
    for(int i = 0; i < _meshes.size(); )
    {
        if(_meshes[i].exists)
        {
            ++i;
        }
        else
        {
            _markers.markers.erase(_markers.markers.begin()+_indexFromID(_meshes[i].marker_id));
            _meshes.erase(_meshes.begin()+i);
        }
    }
    for(int i = 0; i < _markers.markers.size(); ++i)
        _markers.markers[i].header = header;
    _pub->publish(_markers);
}

void MarkerPublisher::showSdfsInstead(bool show_sdfs_instead)
{
    // Clear the markers.
    for(int i = 0; i < _meshes.size(); ++i)
    {
        int j = _indexFromID(_meshes[i].marker_id);
        _markers.markers.erase(_markers.markers.begin()+j);
    }
    _meshes.clear();
    _show_sdfs_instead = show_sdfs_instead;
}

void set_point(void* ptr, float x, float y, float z, const uchar3& rgb)
{
    float* p = (float*)ptr;
    p[0] = x;
    p[1] = y;
    p[2] = z;
    unsigned char* c = (unsigned char*)ptr;
    c[16] = rgb.x;
    c[17] = rgb.y;
    c[18] = rgb.z;
}
void MarkerPublisher::publishPointcloud(const dart::DepthSource<ushort,uchar3>* source, std_msgs::Header header)
{
    _pts->header = header;
    _pts->height = source->getDepthHeight();
    _pts->width = source->getDepthWidth();
    _pts->row_step = _pts->point_step*_pts->width;
    _pts->data.resize(_pts->height*_pts->width*_pts->point_step);
    const ushort* ds = source->getDepth();
    const uchar3* cs = source->getColor();
    float2 fl = source->getFocalLength();
    float2 pp = source->getPrincipalPoint();
    for(int k = 0; k < _pts->height*_pts->width; ++k)
    {
        double depth = 1.0*source->getScaleToMeters()*ds[k];
        void* vp = (void*)&(_pts->data[k*_pts->point_step]);
        if(depth > 0.0)
        {
            int i = k % _pts->width;
            int j = k/_pts->width;
            float3 pt = uvd_to_xyz(make_float3(i, j, depth), fl, pp, _pts->width, _pts->height);
            uchar3 color = cs[k];
            if(_cloud_mask != NULL && _cloud_mask[k] == 0)
                color.x = color.y = color.z = 0.0; //1.0*(0.0f + color.x + color.y + color.z)/3;
            set_point(vp, pt.x, pt.y, pt.z, color);
        }
        else
        {
            set_point(vp, NAN, NAN, NAN, cs[k]);
        }
    }
    _pc_pub->publish(_pts);
    _cloud_mask = NULL;
}




int MarkerPublisher::_getMeshMarker(const string& object_name, int frame_id, int geom_id)
{
    for(int i = 0; i < _meshes.size(); ++i)
    {   
        if(_meshes[i].object_name.compare(object_name) == 0 && _meshes[i].frame_id == frame_id && _meshes[i].geom_id == geom_id)
            return i;
    }
    return -1;
}

int MarkerPublisher::_getFreeMarkerID()
{
    int marker_id;
    for(marker_id = 0; marker_id < _markers.markers.size(); ++marker_id)
    {
        bool found = false;
        for(int i = 0; i < _markers.markers.size() && !found; ++i)
            found |= _markers.markers[i].id == marker_id;
        if(!found)
            break;
    }
    return marker_id;
}

int MarkerPublisher::_addNewMeshMarker(const ObjectInterface& tracker, const string& object_name, int frame_id, int geom_id)
{
    string fp = combine_paths(_web_dir, object_name + strprintf("_%d.stl", geom_id));
    string url = combine_paths(_url, object_name + strprintf("_%d.stl", geom_id));
    //printf("%s\n", url.c_str());
    int id = _idFromName(tracker, object_name);
    // Write model file.
    _writeBinarySTL(fp, tracker.meshFaces(id, geom_id), tracker.meshVerts(id, geom_id), tracker.meshNumFaces(id, geom_id), tracker.meshNumVerts(id, geom_id));
    int ii = _meshes.size();
    _meshes.resize(_meshes.size() + 1);
    _meshes[ii].fp = fp;
    _meshes[ii].object_name = object_name;
    _meshes[ii].frame_id = frame_id;
    _meshes[ii].geom_id = geom_id;
    
    // Find the first available marker id.
    int marker_id = _getFreeMarkerID();
    _meshes[ii].marker_id = marker_id;
    
    // increase size of _markers vector
    int jj = _markers.markers.size();
    _markers.markers.resize(_markers.markers.size() + 1);
    visualization_msgs::Marker& m = _markers.markers[jj];
    float3 scale = tracker.geomScale(id, geom_id);
    m.scale.x = scale.x; m.scale.y = scale.y; m.scale.z = scale.z;
    m.mesh_resource = url;
    m.id = marker_id;
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 1.0;
    if(_obj_colors.find(object_name) == _obj_colors.end())
        _obj_colors[object_name] = make_float3(1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX);
    float3 color = _obj_colors[object_name];
    m.color.r = color.x;
    m.color.g = color.y;
    m.color.b = color.z;
    
    return ii;
}

int MarkerPublisher::_addNewSdfMarker(const ObjectInterface& tracker, const string& object_name, int frame_id, int geom_id)
{
    int id = _idFromName(tracker, object_name);
    int ii = _meshes.size();
    _meshes.resize(_meshes.size() + 1);
    _meshes[ii].object_name = object_name;
    _meshes[ii].frame_id = frame_id;
    _meshes[ii].geom_id = geom_id;
    
    // Find the first available marker id.
    int marker_id = _getFreeMarkerID();
    _meshes[ii].marker_id = marker_id;
    
    // increase size of _markers vector
    int jj = _markers.markers.size();
    _markers.markers.resize(_markers.markers.size() + 1);
    visualization_msgs::Marker& m = _markers.markers[jj];
    m.id = marker_id;
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;
    const dart::Grid3D<float>& sdf = tracker.getSdf(id, frame_id);
    m.scale.x = m.scale.y = m.scale.z = sdf.resolution;
    //m.color.a = 1.0;
    if(_obj_colors.find(object_name) == _obj_colors.end())
        _obj_colors[object_name] = make_float3(1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX);
    //float3 color = _obj_colors[object_name];
    //m.color.r = color.x;
    //m.color.g = color.y;
    //m.color.b = color.z;
    
    float min_dis = 0;
    float max_dis = 0;
    for(int x = 0; x < sdf.dim.x; ++x)
    {
        for(int y = 0; y < sdf.dim.y; ++y)
        {
            for(int z = 0; z < sdf.dim.z; ++z)
            {
                int3 grid_pti = make_int3(x, y, z);
                min_dis = min(min_dis, sdf.getValue(grid_pti));
                max_dis = max(max_dis, sdf.getValue(grid_pti));
            }
        }
    }
    
    m.points.clear();
    m.colors.clear();
    min_dis = -sdf.resolution*10;
    max_dis = sdf.resolution*10;
    for(int x = 0; x < sdf.dim.x; ++x)
    {
        for(int y = 0; y < sdf.dim.y; ++y)
        {
            for(int z = 0; z < sdf.dim.z; ++z)
            {
                int3 grid_pti = make_int3(x, y, z);
                if(sdf.getValue(grid_pti) <= max_dis && sdf.getValue(grid_pti) >= min_dis)
                {
                    float3 grid_ptf = make_float3(x + 0.5, y + 0.5, z + 0.5);
                    float3 pt = sdf.getWorldCoords(grid_ptf);
                    m.points.resize(m.points.size() + 1);
                    int i = m.points.size() - 1;
                    m.points[i].x = pt.x;
                    m.points[i].y = pt.y;
                    m.points[i].z = pt.z;
                    m.colors.resize(m.colors.size() + 1);
                    float d = max(0.0, min(1.0, (max_dis - sdf.getValue(grid_pti))/(max_dis - min_dis)));
                    float3 c = jetmapColor(d);
                    m.colors[i].r = c.x;
                    m.colors[i].g = c.y;
                    m.colors[i].b = c.z;
                    m.colors[i].a = 1.0;
                }
            }
        }
    }
    
    return ii;
}

int MarkerPublisher::_idFromName(const ObjectInterface& tracker, const string& object_name)
{
    for(int i = 0; i < tracker.numObjects(); ++i)
    {
        if(tracker.objectName(i).compare(object_name) == 0)
            return i;
    }
    return -1;
}


void swap_bytes(void* ptr, unsigned nbytes)
{
    unsigned char* p = (unsigned char*)ptr;
    for(unsigned int i = 0; i < nbytes/2; ++i)
    {
        unsigned char c = p[i];
        p[i] = p[nbytes-i-1];
        p[nbytes-i-1] = c;
    }
}


void sfwrite(void* ptr, unsigned int s, unsigned int n, FILE* f)
{
    if(SWAP_BYTES)
    {
        unsigned char* pp = new unsigned char[s];
        for(unsigned int i = 0; i < n; ++i)
        {
            unsigned char* p = ((unsigned char*)ptr) + i*s;
            memcpy(pp, p, s);
            swap_bytes((void*)pp, s);
            fwrite((void*)pp, s, 1, f);
        }
    }
    else
    {
        fwrite(ptr, s, n, f);
    }
}

void MarkerPublisher::_writeBinarySTL(const string& fp, const int3* faces, const float3* verts, unsigned int nFaces, unsigned int nVerts)
{
    FILE* f = fopen(fp.c_str(), "wb");
    char title[80];
    sprintf(title, "Mesh");
    fwrite(title, sizeof(char), 80, f);
    sfwrite((void*)&nFaces, sizeof(unsigned int), 1, f);
    unsigned short spacer = 0;
    for(int i = 0; i < nFaces; ++i)
    {
        const float3& v1 = verts[faces[i].x];
        const float3& v2 = verts[faces[i].y];
        const float3& v3 = verts[faces[i].z];
        float3 n = norm(cross(v2 - v1, v3 - v1));
        sfwrite((void*)&n.x, sizeof(float), 1, f); sfwrite((void*)&n.y, sizeof(float), 1, f); sfwrite((void*)&n.z, sizeof(float), 1, f);
        sfwrite((void*)&v1.x, sizeof(float), 1, f); sfwrite((void*)&v1.y, sizeof(float), 1, f); sfwrite((void*)&v1.z, sizeof(float), 1, f);
        sfwrite((void*)&v2.x, sizeof(float), 1, f); sfwrite((void*)&v2.y, sizeof(float), 1, f); sfwrite((void*)&v2.z, sizeof(float), 1, f);
        sfwrite((void*)&v3.x, sizeof(float), 1, f); sfwrite((void*)&v3.y, sizeof(float), 1, f); sfwrite((void*)&v3.z, sizeof(float), 1, f);
        sfwrite((void*)&spacer, sizeof(unsigned short), 1, f);
    }
    fclose(f);
}
































