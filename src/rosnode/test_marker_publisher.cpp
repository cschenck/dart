
#include <map>
#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <std_msgs/Header.h>

#include "marker_publisher.h"

using namespace std;

void make_cube(float size, vector<int3>* fcs, vector<float3>* vts)
{
    vector<float3>& verts = *vts;
    verts.resize(8);
    verts[0] = make_float3(0,0,0);
    verts[1] = make_float3(size,0,0);
    verts[2] = make_float3(size,size,0);
    verts[3] = make_float3(0,size,0);
    verts[4] = make_float3(0,size,size);
    verts[5] = make_float3(size,size,size);
    verts[6] = make_float3(size,0,size);
    verts[7] = make_float3(0,0,size);
    
    for(int i = 0; i < verts.size(); ++i)
        verts[i] -= size/2.0;
    
    vector<int3>& faces = *fcs;
    faces.resize(12);
    faces[0] = make_int3(0, 1, 3); faces[1] = make_int3(1, 2, 3);
    faces[2] = make_int3(3, 2, 4); faces[3] = make_int3(2, 5, 4);
    faces[4] = make_int3(1, 6, 2); faces[5] = make_int3(6, 5, 2);
    faces[6] = make_int3(7, 3, 4); faces[7] = make_int3(7, 0, 3);
    faces[8] = make_int3(7, 4, 5); faces[9] = make_int3(7, 5, 6);
    faces[10] = make_int3(0, 7, 1); faces[11] = make_int3(1, 7, 6);
}

void make_pyramid(float size, vector<int3>* fcs, vector<float3>* vts)
{
    vector<float3>& verts = *vts;
    verts.resize(5);
    verts[0] = make_float3(0,0,0);
    verts[1] = make_float3(size,0,0);
    verts[2] = make_float3(size,0,size);
    verts[3] = make_float3(0,0,size);
    verts[4] = make_float3(0.5*size,size,0.5*size);
    for(int i = 0; i < verts.size(); ++i)
        verts[i] -= size/2.0;
    
    vector<int3>& faces = *fcs;
    faces.resize(6);
    faces[0] = make_int3(0, 3, 1); 
    faces[1] = make_int3(1, 3, 2);
    faces[2] = make_int3(0, 1, 4);
    faces[3] = make_int3(1, 2, 4);
    faces[4] = make_int3(2, 3, 4);
    faces[5] = make_int3(3, 0, 4);
}

void reverse_faces(vector<int3>* fcs)
{
    vector<int3>& faces = *fcs;
    for(int i = 0; i < faces.size(); ++i)
        faces[i] = make_int3(faces[i].z, faces[i].y, faces[i].x);
}

struct mesh
{
    vector<float3> verts;
    vector<int3> faces;
    int geom_id;
    float3 size;
    dart::SE3 rel_tran;
};

struct object
{
    vector<dart::SE3> frame_trans;
    vector<vector<mesh>> meshes;
    map<int, mesh*> geom_id_map;
    string name;
    int num_geoms;
};

class TestInterface : public ObjectInterface
{
public:

    TestInterface() 
    {
        _iter = 0;
        dart::SE3 eye = dart::SE3FromTranslation(0.0f, 0.0f, 0.0f);
        // Construct the omni-omega-tron!
        _objects.resize(2);
        object& obj = _objects[0];
        obj.num_geoms = 2;
        obj.frame_trans.resize(2);
        obj.meshes.resize(2);
        obj.name = "cube_with_pyramid";
        // Add cube.
        obj.frame_trans[0] = eye;
        obj.meshes[0].resize(1);
        make_cube(1.0, &(obj.meshes[0][0].faces), &(obj.meshes[0][0].verts));
        reverse_faces(&(obj.meshes[0][0].faces));
        obj.meshes[0][0].geom_id = 0;
        obj.meshes[0][0].size = make_float3(1,1,1);
        obj.meshes[0][0].rel_tran = eye;
        obj.geom_id_map[obj.meshes[0][0].geom_id] = &(obj.meshes[0][0]);
        // Add pyramid.
        obj.frame_trans[1] = dart::SE3FromTranslation(0.0f, 1.0f, 0.0f);
        obj.meshes[1].resize(1);
        make_pyramid(1.0, &(obj.meshes[1][0].faces), &(obj.meshes[1][0].verts));
        reverse_faces(&(obj.meshes[1][0].faces));
        obj.meshes[1][0].geom_id = 1;
        obj.meshes[1][0].size = make_float3(1,1,1);
        obj.meshes[1][0].rel_tran = eye;
        obj.geom_id_map[obj.meshes[1][0].geom_id] = &(obj.meshes[1][0]);
        
        object& obj2 = _objects[1];
        obj2.num_geoms = 2;
        obj2.frame_trans.resize(1);
        obj2.meshes.resize(1);
        obj2.name = "cube_with_pyramid2";
        // Add cube.
        obj2.frame_trans[0] = dart::SE3FromTranslation(0.0, -1.0, 0.0);
        obj2.meshes[0].resize(2);
        make_cube(1.0, &(obj2.meshes[0][0].faces), &(obj2.meshes[0][0].verts));
        reverse_faces(&(obj2.meshes[0][0].faces));
        obj2.meshes[0][0].geom_id = 0;
        obj2.meshes[0][0].size = make_float3(0.5,0.5,0.5);
        obj2.meshes[0][0].rel_tran = eye;
        obj2.geom_id_map[obj2.meshes[0][0].geom_id] = &(obj2.meshes[0][0]);
        // Add pyramid.
        make_pyramid(1.0, &(obj2.meshes[0][1].faces), &(obj2.meshes[0][1].verts));
        reverse_faces(&(obj2.meshes[0][1].faces));
        obj2.meshes[0][1].geom_id = 1;
        obj2.meshes[0][1].size = make_float3(0.5,1.5,1);
        obj2.meshes[0][1].rel_tran = dart::SE3FromTranslation(0.0f, -1.0f, 0.0f)*dart::SE3FromRotationX(M_PI);
        obj2.geom_id_map[obj2.meshes[0][1].geom_id] = &(obj2.meshes[0][1]);
    }
    
    void update()
    {
        _objects[0].frame_trans[1] = dart::SE3FromTranslation(0.0f, 1.0f, 0.0f)*dart::SE3FromRotationY(1.0*_iter/(5.0*30.0)*M_PI);
        _objects[1].frame_trans[0] = dart::SE3FromTranslation(0.0f, -1.0f, 0.0f)*dart::SE3FromRotationY(-1.0*_iter/(5.0*30.0)*M_PI);
        ++_iter;
    }
    
    inline int numObjects() const { return _objects.size(); }
    inline string objectName(int object_id) const { return _objects[object_id].name; }
    inline int numFrames(int object_id) const { return _objects[object_id].frame_trans.size(); }
    inline int numGeoms(int object_id, int frame_id) const { return _objects[object_id].meshes[frame_id].size(); }
    inline int geomID(int object_id, int frame_id, int idx) const { return _objects[object_id].meshes[frame_id][idx].geom_id; }
    inline dart::SE3 frameTransform(int object_id, int frame_id) const { return _objects[object_id].frame_trans[frame_id]; }
    inline dart::SE3 relativeGeomTransform(int object_id, int geom_id) const { return _objects[object_id].geom_id_map.at(geom_id)->rel_tran; }
    inline float3 geomScale(int object_id, int geom_id) const { return _objects[object_id].geom_id_map.at(geom_id)->size; }
    inline const int3* meshFaces(int object_id, int geom_id) const { return &(_objects[object_id].geom_id_map.at(geom_id)->faces[0]); }
    inline const float3* meshVerts(int object_id, int geom_id) const { return &(_objects[object_id].geom_id_map.at(geom_id)->verts[0]); }
    inline const float3* meshNorms(int object_id, int geom_id) const { return NULL; }
    inline int meshNumFaces(int object_id, int geom_id) const { return _objects[object_id].geom_id_map.at(geom_id)->faces.size(); }
    inline int meshNumVerts(int object_id, int geom_id) const { return _objects[object_id].geom_id_map.at(geom_id)->verts.size(); }

private:
    vector<object> _objects;
    int _iter;
};


int main(int argc, char** argv)
{    
    ROS_INFO("Starting dart node.");
    ros::init(argc, argv, "dart");
    
    MarkerPublisher mp(argv[1], argv[2], argv[3]);
    TestInterface inter;
    std_msgs::Header header;
    header.frame_id = "base";
    ros::Rate r(30);
    
    while(ros::ok())
    {
        header.stamp = ros::Time::now();
        mp.update(inter, header);
        inter.update();
        r.sleep();
    }
    
    ros::shutdown();
    
    return 0;
}


















