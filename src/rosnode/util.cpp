
#include <algorithm>
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
    float m00 = source.r0.x;
    float m01 = source.r0.y;
    float m02 = source.r0.z;
    float m10 = source.r1.x;
    float m11 = source.r1.y;
    float m12 = source.r1.z;
    float m20 = source.r2.x;
    float m21 = source.r2.y;
    float m22 = source.r2.z;
    
    //swap(m01, m10);
    //swap(m02, m20);
    //swap(m12, m21);
    
    float tr = m00 + m11 + m22;
    if(tr > 0.){
        float s = sqrt(tr+1.) * 2.;
        r.w = 0.25 * s;
        r.x = ( m21 - m12 ) / s;
        r.y = ( m02 - m20 ) / s;
        r.z = ( m10 - m01 ) / s;
    }
    else{
        if( m00 > m11 && m00 > m22 ){
            float s = 2.f * sqrtf(1.f+m00-m11-m22);
            r.w = (m21 - m12) / s;
            r.x = 0.25f * s;
            r.y = (m01 + m10) / s;
            r.z = (m02 + m20) / s;
        }
        else if(m11 > m22){
            float s = 2.f * sqrtf(1.f+m11-m00-m22);
            r.w = (m02 - m20) / s;
            r.x = (m01 + m10) / s;
            r.y = 0.25 * s;
            r.z = (m12 + m21) / s;
        }
        else{
            float s = 2.f * sqrtf(1.f+m22-m00-m11);
            r.w = (m10 - m01) / s;
            r.x = (m02 + m20) / s;
            r.y = (m12 + m21) / s;
            r.z = 0.25 * s;
        }
    }
    
    float m = sqrt(r.x*r.x + r.y*r.y + r.z*r.z + r.w*r.w);
    return make_float4(r.x/m, r.y/m, r.z/m, r.w/m);
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


















