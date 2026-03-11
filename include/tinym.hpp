// Tiny ym
// This is only for the simple vec3, vec4, mat3, mat4 operations

#ifndef TINYM_HPP
#define TINYM_HPP

#include <cmath>
#include <cstring>

namespace tinym {

using T = float;

struct vec3 {
    union {
        struct { T x, y, z; };
        T v[3];
    };
    vec3() : x(0), y(0), z(0) {}
    vec3(T x, T y, T z) : x(x), y(y), z(z) {}

    T& operator[](int i) { return v[i]; }
    const T& operator[](int i) const { return v[i]; }

    vec3 operator-(const vec3& v) const { return vec3(x - v.x, y - v.y, z - v.z); }
    vec3 operator+(const vec3& v) const { return vec3(x + v.x, y + v.y, z + v.z); }
    vec3 operator*(float s) const { return vec3(x * s, y * s, z * s); }
    float norm() const { return std::sqrt(x*x + y*y + z*z); }
    vec3 normalize() const {
        float n = norm();
        return n > 0.0f ? vec3(x/n, y/n, z/n) : vec3();
    }
    
    float dot(const vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    vec3 cross(const vec3& v) const {
        return vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }
};

struct vec4 {
    union {
        struct { T x, y, z, w; };
        T v[4];
    };
    vec4() : x(0), y(0), z(0), w(0) {}
    vec4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
    vec4(const vec3& v, T w) : x(v.x), y(v.y), z(v.z), w(w) {}

    operator vec3() const { return vec3(x, y, z); }
    T& operator[](int i) { return v[i]; }
    const T& operator[](int i) const { return v[i]; }
    vec4& operator+=(const vec4& o) {
        for(int row = 0; row < 4; row++)
            this->v[row] += o[row];
        return *this;
    }
    vec4 operator*(float s) const { return vec4(x*s, y*s, z*s, w*s); }
};

struct mat3 {
    union {
        vec3 c[3];
        T v[9];
    };
    mat3(vec3& c0, vec3& c1, vec3& c2) {
        c[0] = c0; c[1] = c1; c[2] = c2;
    }
    mat3(vec4& c0, vec4& c1, vec4& c2) {
        c[0] = c0; c[1] = c1; c[2] = c2;
    }
};

struct mat4 {
    union {
        vec4 c[4];
        T v[16];
    };
    mat4() {
        memset(v, 0, sizeof(v));
    }
    mat4(T d) {
        c[0][0] = d; c[1][0] = 0; c[2][0] = 0; c[3][0] = 0;
        c[0][1] = 0; c[1][1] = d; c[2][1] = 0; c[3][1] = 0;
        c[0][2] = 0; c[1][2] = 0; c[2][2] = d; c[3][2] = 0;
        c[0][3] = 0; c[1][3] = 0; c[2][3] = 0; c[3][3] = d;
    }
    vec4& operator[](int i) { return c[i]; }
    mat4 operator*(const mat4& rhs) const {
        mat4 res;
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                for (int i = 0; i < 4; ++i) {
                    res.v[c*4 + r] += v[i*4 + r] * rhs.v[c*4 + i];
                }
            }
        }
        return res;
    }
    vec4 operator*(const vec4& v) const {
        return vec4(
            c[0].x*v.x + c[1].x*v.y + c[2].x*v.z + c[3].x*v.w,
            c[0].y*v.x + c[1].y*v.y + c[2].y*v.z + c[3].y*v.w,
            c[0].z*v.x + c[1].z*v.y + c[2].z*v.z + c[3].z*v.w,
            c[0].w*v.x + c[1].w*v.y + c[2].w*v.z + c[3].w*v.w
        );
    }
};


// 3. 카메라 View 행렬 생성 (LookAt)
inline mat4 lookAt(const vec3& eye, const vec3& center, const vec3& up) {
    vec3 f = (center - eye).normalize();
    vec3 s = f.cross(up).normalize();
    vec3 u = s.cross(f);

    mat4 res;
    res.v[0] = s.x;  res.v[4] = s.y;  res.v[8]  = s.z;  res.v[12] = -s.dot(eye);
    res.v[1] = u.x;  res.v[5] = u.y;  res.v[9]  = u.z;  res.v[13] = -u.dot(eye);
    res.v[2] = -f.x; res.v[6] = -f.y; res.v[10] = -f.z; res.v[14] = f.dot(eye);
    res.v[3] = 0.0f; res.v[7] = 0.0f; res.v[11] = 0.0f; res.v[15] = 1.0f;
    return res;
}

// 4. 카메라 Projection 행렬 생성 (Perspective)
inline mat4 perspective(float fovy_rad, float aspect, float zNear, float zFar) {
    float tanHalfFovy = std::tan(fovy_rad / 2.0f);
    
    mat4 res;
    res.v[0] = 1.0f / (aspect * tanHalfFovy);
    res.v[5] = 1.0f / (tanHalfFovy);
    res.v[10] = -(zFar + zNear) / (zFar - zNear);
    res.v[11] = -1.0f;
    res.v[14] = -(2.0f * zFar * zNear) / (zFar - zNear);
    return res;
}

// 5. 임의의 축을 기준으로 회전하는 행렬 생성 (Rodrigues' rotation formula)
inline tinym::mat4 rotate(float angle, const tinym::vec3& axis) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    float C = 1.0f - c;

    // 회전축은 반드시 정규화(Normalize)되어야 합니다.
    tinym::vec3 ax = axis.normalize();
    float x = ax.x;
    float y = ax.y;
    float z = ax.z;

    // tinym::mat4(T d) 생성자를 이용해 대각선이 1인 단위 행렬(Identity)로 뼈대를 잡습니다.
    tinym::mat4 res(1.0f);

    // Column-major 배열 (c[열].행) 에 맞춰 회전 공식 적용
    res.c[0].x = x * x * C + c;
    res.c[0].y = y * x * C + z * s;
    res.c[0].z = z * x * C - y * s;

    res.c[1].x = x * y * C - z * s;
    res.c[1].y = y * y * C + c;
    res.c[1].z = z * y * C + x * s;

    res.c[2].x = x * z * C + y * s;
    res.c[2].y = y * z * C - x * s;
    res.c[2].z = z * z * C + c;

    // 4번째 열과 4번째 행은 단위 행렬 값(0, 0, 0, 1) 그대로 유지됩니다.
    return res;
}

inline const T* value_ptr(const vec3& v3) { return v3.v; }
inline const T* value_ptr(const vec4& v4) { return v4.v; }
inline const T* value_ptr(const mat3& m3) { return m3.v; }
inline const T* value_ptr(const mat4& m4) { return m4.v; }

}; // end namespace


#endif
