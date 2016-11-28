
#include <math.h>
#include <stdarg.h>
#include <vector>

#include "util.h"

using namespace std;

string vstrprintf(const char* format, va_list args)
{
	vector<char> buff(1);
	va_list args_b;
	va_copy(args_b, args);
	int n = vsnprintf(&buff[0], 1, format, args_b);
	buff.resize(n+1);
	vsnprintf(&buff[0], n + 1, format, args);
	va_end(args_b);
	return string(buff.begin(), buff.end());
}

string strprintf(const char* format, ...)
{
	va_list args;
	va_start(args, format);
	string ret = vstrprintf(format, args);
	va_end(args);
	return ret;
}

string combine_paths(const string& left, const string& right)
{
    string ret = left;
    if(left[left.length()-1] != '/')
        ret += '/';
    ret += right;
    return ret;
}

float4 SE3ToQuaternion(const dart::SE3& source){
    
    float4 r;
    
    float tr = source.r0.x + source.r1.y + source.r2.z;
    if(tr > 0.){
        float s = sqrt(tr+1.) * 2.;
        r.w = 0.25 * s;
        r.x = ( source.r2.y - source.r1.z ) / s;
        r.y = ( source.r0.z - source.r2.x ) / s;
        r.z = ( source.r1.x - source.r0.y ) / s;
    }
    else{
        if( source.r0.x > source.r1.y && source.r0.x > source.r2.z ){
            float s = 2.f * sqrtf(1.f+source.r0.x-source.r1.y-source.r2.z);
            r.w = (source.r2.y - source.r1.z) / s;
            r.x = 0.25f * s;
            r.y = (source.r0.y + source.r1.x) / s;
            r.z = (source.r0.z + source.r2.x) / s;
        }
        else if(source.r1.y > source.r2.z){
            float s = 2.f * sqrtf(1.f+source.r1.y-source.r0.x-source.r2.z);
            r.w = (source.r0.z - source.r2.x) / s;
            r.x = (source.r0.y + source.r1.x) / s;
            r.y = 0.25 * s;
            r.z = (source.r1.z + source.r2.y) / s;
        }
        else{
            float s = 2.f * sqrtf(1.f+source.r2.z-source.r0.x-source.r1.y);
            r.w = (source.r1.x - source.r0.y) / s;
            r.x = (source.r0.z + source.r2.x) / s;
            r.y = (source.r1.z + source.r2.y) / s;
            r.z = 0.25 * s;
        }
    }
    
    return r;
}

float3 SE3ToTranslation(const dart::SE3& source)
{
    return make_float3(source.r0.w, source.r1.w, source.r2.w);
}

/*
float3 operator-(const float3& a, const float3& b)
{
	float3 ret;
	ret.x = a.x - b.x;
	ret.y = a.y - b.y;
	ret.z = a.z - b.z;
	return ret;
}

float3 operator+(const float3& a, const float3& b)
{
	float3 ret;
	ret.x = a.x + b.x;
	ret.y = a.y + b.y;
	ret.z = a.z + b.z;
	return ret;
}

float3 operator-(const float3& a, const float& b)
{
	float3 ret;
	ret.x = a.x-b;
	ret.y = a.y-b;
	ret.z = a.z-b;
	return ret;
}

float3 operator+(const float3& a, const float& b)
{
	float3 ret;
	ret.x = a.x+b;
	ret.y = a.y+b;
	ret.z = a.z+b;
	return ret;
}

float3 operator*(const float3& a, const float& b)
{
	float3 ret;
	ret.x = a.x*b;
	ret.y = a.y*b;
	ret.z = a.z*b;
	return ret;
}

float3 operator/(const float3& a, const float& b)
{
	float3 ret;
	ret.x = a.x/b;
	ret.y = a.y/b;
	ret.z = a.z/b;
	return ret;
}

float3 cross(const float3& left, const float3& right)
{
	return make_float3(left.y*right.z - left.z*right.y, left.z*right.x - left.x*right.z, left.x*right.y - left.y*right.x);
}
*/

float _dot(const float3& left, const float3& right)
{
	return left.x*right.x + left.y*right.y + left.z*right.z;
}

float3 norm(const float3& v)
{
	float m = sqrt(_dot(v, v));
	return make_float3(v.x/m, v.y/m, v.z/m);
}




