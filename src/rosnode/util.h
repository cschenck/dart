
#include <string>

#include <vector_types.h>
#include <vector_functions.h>

#include "geometry/SE3.h"

using namespace std;

string combine_paths(const string& left, const string& right);

string strprintf(const char* format, ...);

float4 SE3ToQuaternion(const dart::SE3& source);

float3 SE3ToTranslation(const dart::SE3& source);

/*
float3 operator-(const float3& a, const float3& b);

float3 operator+(const float3& a, const float3& b);

float3 operator-(const float3& a, const float& b);

float3 operator+(const float3& a, const float& b);

float3 operator*(const float3& a, const float& b);

float3 operator/(const float3& a, const float& b);

float3 cross(const float3& left, const float3& right);
*/

//float dot(const float3& left, const float3& right);

float3 norm(const float3& v);

__device__ __host__
float3 uvd_to_xyz(float3 uvd, float2 focal_length, float2 pp, int width, int height);

