#ifndef __MARKER_PUBLISHER_H__
#define __MARKER_PUBLISHER_H__

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include "geometry/SE3.h"
#include "tracker.h"

#define SWAP_BYTES false

using namespace std;

struct mesh_marker
{
    string fp;
    string object_name;
    int frame_id;
    int geom_id;
    bool exists;
};

class ObjectInterface
{
public:
    virtual int numObjects() const = 0;
    virtual string objectName(int object_id) const = 0;
    virtual int numFrames(int object_id) const = 0;
    virtual int numGeoms(int object_id, int frame_id) const = 0;
    virtual int geomID(int object_id, int frame_id, int idx) const = 0;
    virtual dart::SE3 frameTransform(int object_id, int frame_id) const = 0;
    virtual dart::SE3 relativeGeomTransform(int object_id, int geom_id) const = 0;
    virtual float3 geomScale(int object_id, int geom_id) const = 0;
    virtual const int3* meshFaces(int object_id, int geom_id) const = 0;
    virtual const float3* meshVerts(int object_id, int geom_id) const = 0;
    virtual const float3* meshNorms(int object_id, int geom_id) const = 0;
    virtual int meshNumFaces(int object_id, int geom_id) const = 0;
    virtual int meshNumVerts(int object_id, int geom_id) const = 0;
};

class TrackerWrapper : public ObjectInterface
{
public:

    TrackerWrapper(const dart::Tracker& tracker) : _tracker(tracker) {}
    inline int numObjects() const { return _tracker.getNumModels(); }
    inline string objectName(int object_id) const { return _tracker.getModel(object_id).getName(); }
    inline int numFrames(int object_id) const { return _tracker.getModel(object_id).getNumFrames(); }
    inline int numGeoms(int object_id, int frame_id) const { return _tracker.getModel(object_id).getFrameNumGeoms(frame_id); }
    inline int geomID(int object_id, int frame_id, int idx) const { return _tracker.getModel(object_id).getFrameGeoms(frame_id)[idx]; }
    inline dart::SE3 frameTransform(int object_id, int frame_id) const { return _tracker.getModel(object_id).getTransformModelToFrame(frame_id); }
    inline dart::SE3 relativeGeomTransform(int object_id, int geom_id) const { return _tracker.getModel(object_id).getGeometryTransform(geom_id); }
    inline float3 geomScale(int object_id, int geom_id) const { return _tracker.getModel(object_id).getGeometryScale(geom_id); }
    inline const int3* meshFaces(int object_id, int geom_id) const { return _tracker.getModel(object_id).getMesh(_tracker.getModel(object_id).getMeshNumber(geom_id)).faces; }
    inline const float3* meshVerts(int object_id, int geom_id) const { return _tracker.getModel(object_id).getMesh(_tracker.getModel(object_id).getMeshNumber(geom_id)).vertices; }
    inline const float3* meshNorms(int object_id, int geom_id) const { return _tracker.getModel(object_id).getMesh(_tracker.getModel(object_id).getMeshNumber(geom_id)).normals; }
    inline int meshNumFaces(int object_id, int geom_id) const { return _tracker.getModel(object_id).getMesh(_tracker.getModel(object_id).getMeshNumber(geom_id)).nFaces; }
    inline int meshNumVerts(int object_id, int geom_id) const { return _tracker.getModel(object_id).getMesh(_tracker.getModel(object_id).getMeshNumber(geom_id)).nVertices; }

private:
    const dart::Tracker& _tracker;
};


class MarkerPublisher
{
public:
    MarkerPublisher(string url, string web_dir, string topic);
    ~MarkerPublisher();
    void update(const ObjectInterface& tracker, std_msgs::Header header);

private:
    int _getMeshMarker(const string& object_name, int frame_id, int geom_id);
    int _addNewMeshMarker(const ObjectInterface& tracker, const string& object_name, int frame_id, int geom_id);
    void _writeBinarySTL(const string& fp, const int3* faces, const float3* verts, unsigned int nFaces, unsigned int nVerts);
    int _idFromName(const ObjectInterface& tracker, const string& object_name);
    

    map<string, float3> _obj_colors;
    ros::NodeHandle* _ros_node;
    ros::Publisher* _pub;
    string _url;
    string _web_dir;
    vector<mesh_marker> _meshes;
    visualization_msgs::MarkerArray _markers;
};









#endif

