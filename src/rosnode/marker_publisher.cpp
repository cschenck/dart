
#include <stdlib.h>

#include "util.h"

#include "marker_publisher.h"



using namespace std;

MarkerPublisher::MarkerPublisher(string url, string web_dir, string topic) :
    _url(url), _web_dir(web_dir)
{
    _ros_node = new ros::NodeHandle();
    _pub = new ros::Publisher(_ros_node->advertise<visualization_msgs::MarkerArray>(topic, 1));
}

MarkerPublisher::~MarkerPublisher()
{
    _pub->shutdown();
    delete _pub;
    delete _ros_node;
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
            for(int idx = 0; idx < tracker.numGeoms(obj, frame); ++idx)
            {
                int geom = tracker.geomID(obj, frame, idx);
                int ii = _getMeshMarker(obj_name, frame, geom);
                if(ii < 0)
                    ii = _addNewMeshMarker(tracker, obj_name, frame, geom);
                mesh_marker& mesh = _meshes[ii];
                visualization_msgs::Marker& marker = _markers.markers[ii];
                dart::SE3 t = tracker.frameTransform(obj, frame)*tracker.relativeGeomTransform(obj, geom);
                float4 rot = SE3ToQuaternion(t);
                float3 tran = SE3ToTranslation(t);
                marker.pose.position.x = tran.x;
                marker.pose.position.y = tran.y;
                marker.pose.position.z = tran.z;
                marker.pose.orientation.x = rot.x;
                marker.pose.orientation.y = rot.y;
                marker.pose.orientation.z = rot.z;
                marker.pose.orientation.w = rot.w;
                
                marker.header = header;
                mesh.exists = true;
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
            _meshes.erase(_meshes.begin()+i);
            _markers.markers.erase(_markers.markers.begin()+i);
        }
    }
    _pub->publish(_markers);
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

int MarkerPublisher::_addNewMeshMarker(const ObjectInterface& tracker, const string& object_name, int frame_id, int geom_id)
{
    string fp = combine_paths(_web_dir, object_name + strprintf("_%d.stl", geom_id));
    string url = combine_paths(_url, object_name + strprintf("_%d.stl", geom_id));
    printf("%s\n", url.c_str());
    int id = _idFromName(tracker, object_name);
    // Write model file.
    _writeBinarySTL(fp, tracker.meshFaces(id, geom_id), tracker.meshVerts(id, geom_id), tracker.meshNumFaces(id, geom_id), tracker.meshNumVerts(id, geom_id));
    int ret = _meshes.size();
    _meshes.resize(_meshes.size() + 1);
    _meshes[ret].fp = fp;
    _meshes[ret].object_name = object_name;
    _meshes[ret].frame_id = frame_id;
    _meshes[ret].geom_id = geom_id;
    
    // increase size of _markers vector
    _markers.markers.resize(_markers.markers.size() + 1);
    visualization_msgs::Marker& m = _markers.markers[ret];
    float3 scale = tracker.geomScale(id, geom_id);
    m.scale.x = scale.x; m.scale.y = scale.y; m.scale.z = scale.z;
    m.mesh_resource = url;
    m.id = ret;
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 1.0;
    if(_obj_colors.find(object_name) == _obj_colors.end())
        _obj_colors[object_name] = make_float3(1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX, 1.0*rand()/RAND_MAX);
    float3 color = _obj_colors[object_name];
    m.color.r = color.x;
    m.color.g = color.y;
    m.color.b = color.z;
    
    return ret;
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































