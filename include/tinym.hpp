// Tiny ym
// This is only for the simple vec3, vec4, mat3, mat4 operations

#ifndef TINYM_HPP
#define TINYM_HPP

#include <cmath>
#include <cstring>
#include <ostream>
#include <type_traits>

namespace tinym {

//using T = float;

template <typename T>
struct vec3_view_base {
    T* ptr;
    vec3_view_base(T* ptr) : ptr(ptr) {}
    T& operator[](int i) { return ptr[i]; }
    const T& operator[](int i) const { return ptr[i]; }
};
template <typename T>
struct vec3_base {
    union {
        struct { T x, y, z; };
        struct { T r, g, b; };
        T v[3];
    };
    vec3_base() : x(0), y(0), z(0) {}
    vec3_base(T a) : x(a), y(a), z(a) {}
    vec3_base(T x, T y, T z) : x(x), y(y), z(z) {}

    T& operator[](int i) { return v[i]; }
    const T& operator[](int i) const { return v[i]; }
    
    operator vec3_view_base<T>() { return vec3_view_base<T>(v); }
    //operator const vec3_view_base<T>() const { return vec3_view_base<T>(const_cast<T*>(v)); }

    vec3_base operator-(const vec3_base& v) const { return vec3_base(x - v.x, y - v.y, z - v.z); }
    vec3_base operator+(const vec3_base& v) const { return vec3_base(x + v.x, y + v.y, z + v.z); }
    vec3_base operator/(const vec3_base& v) const { return vec3_base(x/v.x, y/v.y, z/v.z); }
    vec3_base operator/(const float& v) const { return vec3_base(x/v, y/v, z/v); }
    vec3_base operator*(float s) const { return vec3_base(x * s, y * s, z * s); }
    vec3_base& operator+=(const vec3_view_base<T>& v) { 
        x += v[0]; y += v[1]; z += v[2];
        return *this;
    }
    vec3_base& operator/=(const float& s) {
        x /= s; y /= s; z /= s;
        return *this;
    }
    vec3_base& operator-=(const vec3_view_base<T>& v) {
        x -= v[0]; y -= v[1]; z -= v[2];
        return *this;
    }
    float norm() const { return std::sqrt(x*x + y*y + z*z); }
    vec3_base normalize() const {
        float n = norm();
        return n > 0.0f ? vec3_base(x/n, y/n, z/n) : vec3_base();
    }
    
    float dot(const vec3_base& v) const { return x*v.x + y*v.y + z*v.z; }
    vec3_base cross(const vec3_base& v) const {
        return vec3_base(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }

    friend std::ostream& operator<<(std::ostream& os, const vec3_base& v) {
        os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
        return os;
    }
};

using vec3 = vec3_base<float>;
using vec3ui = vec3_base<uint>;
using vec3_view = vec3_view_base<float>;

inline vec3 operator-(const vec3_view& a, const vec3_view& b) {
    return vec3(a[0]-b[0], a[1]-b[1], a[2]-b[2]);
}
inline vec3 normalize(const vec3& a) {
    return a.normalize();
}
inline vec3 cross(const vec3& a, const vec3& b) {
    return a.cross(b);
}

template <typename T>
struct vec4_base {
    union {
        struct { T x, y, z, w; };
        T v[4];
    };
    vec4_base() : x(0), y(0), z(0), w(0) {}
    vec4_base(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
    vec4_base(const vec3_base<T>& v, T w) : x(v.x), y(v.y), z(v.z), w(w) {}

    operator vec3() const { return vec3_base<T>(x, y, z); }
    T& operator[](int i) { return v[i]; }
    const T& operator[](int i) const { return v[i]; }
    vec4_base& operator+=(const vec4_base& o) {
        for(int row = 0; row < 4; row++)
            this->v[row] += o[row];
        return *this;
    }
    vec4_base operator*(float s) const { return vec4_base(x*s, y*s, z*s, w*s); }
};

using vec4 = vec4_base<float>;
using vec4ui = vec4_base<uint>;

template <typename T>
struct mat3_base {
    union {
        vec3_base<T> c[3];
        T v[9];
    };
    mat3_base(vec3_base<T>& c0, vec3_base<T>& c1, vec3_base<T>& c2) {
        c[0] = c0; c[1] = c1; c[2] = c2;
    }
    mat3_base(vec4_base<T>& c0, vec4_base<T>& c1, vec4_base<T>& c2) {
        c[0] = c0; c[1] = c1; c[2] = c2;
    }
};

using mat3 = mat3_base<float>;

template <typename T>
struct mat4_base {
    union {
        vec4_base<T> c[4];
        T v[16];
    };
    mat4_base() {
        memset(v, 0, sizeof(v));
    }
    mat4_base(T d) {
        c[0][0] = d; c[1][0] = 0; c[2][0] = 0; c[3][0] = 0;
        c[0][1] = 0; c[1][1] = d; c[2][1] = 0; c[3][1] = 0;
        c[0][2] = 0; c[1][2] = 0; c[2][2] = d; c[3][2] = 0;
        c[0][3] = 0; c[1][3] = 0; c[2][3] = 0; c[3][3] = d;
    }
    mat4_base(vec4_base<T> c0, vec4_base<T> c1, vec4_base<T> c2, vec4_base<T> c3) {
        c[0] = c0; c[1] = c1; c[2] = c2; c[3] = c3;
    }
    vec4_base<T>& operator[](int i) { return c[i]; }
    mat4_base operator*(const mat4_base& rhs) const {
        mat4_base res;
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                for (int i = 0; i < 4; ++i) {
                    res.v[c*4 + r] += v[i*4 + r] * rhs.v[c*4 + i];
                }
            }
        }
        return res;
    }
    vec4_base<T> operator*(const vec4_base<T>& v) const {
        return vec4_base<T>(
            c[0].x*v.x + c[1].x*v.y + c[2].x*v.z + c[3].x*v.w,
            c[0].y*v.x + c[1].y*v.y + c[2].y*v.z + c[3].y*v.w,
            c[0].z*v.x + c[1].z*v.y + c[2].z*v.z + c[3].z*v.w,
            c[0].w*v.x + c[1].w*v.y + c[2].w*v.z + c[3].w*v.w
        );
    }
};

using mat4 = mat4_base<float>;


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

template <typename T>
inline const T* value_ptr(const vec3_base<T>& v3) { return v3.v; }
template <typename T>
inline const T* value_ptr(const vec4_base<T>& v4) { return v4.v; }
template <typename T>
inline const T* value_ptr(const mat3_base<T>& m3) { return m3.v; }
template <typename T>
inline const T* value_ptr(const mat4_base<T>& m4) { return m4.v; }

template <typename T>
inline T min(const vec3_base<T>& v) {
    T m = v.x;
    if(m > v.y) m = v.y;
    if(m > v.z) m = v.z;
    return m;
}
template <typename T>
inline T max(const vec3_base<T>& v) {
    T m = v.x;
    if(m < v.y) m = v.y;
    if(m < v.z) m = v.z;
    return m;
}

template <typename>
struct is_vec3_like : std::false_type {};

template <typename T>
struct is_vec3_like<vec3_base<T>> : std::true_type {};

template <typename T>
struct is_vec3_like<vec3_view_base<T>> : std::true_type {};

template <typename T>
inline constexpr bool is_vec3_like_v =
    is_vec3_like<std::remove_cv_t<std::remove_reference_t<T>>>::value;

template <typename>
struct vec3_scalar;

template <typename T>
struct vec3_scalar<vec3_base<T>> { using type = T; };

template <typename T>
struct vec3_scalar<vec3_view_base<T>> { using type = T; };

template <typename T>
using vec3_scalar_t =
    typename vec3_scalar<std::remove_cv_t<std::remove_reference_t<T>>>::type;

template <
    typename A, typename B,
    typename = std::enable_if_t<
        is_vec3_like_v<A> &&
        is_vec3_like_v<B> &&
        std::is_same_v<vec3_scalar_t<A>, vec3_scalar_t<B>>
    >
>
inline vec3_base<vec3_scalar_t<A>> min(const A& a, const B& b) {
    using T = vec3_scalar_t<A>;
    vec3_base<T> ret;
    ret[0] = a[0] > b[0] ? b[0] : a[0];
    ret[1] = a[1] > b[1] ? b[1] : a[1];
    ret[2] = a[2] > b[2] ? b[2] : a[2];
    return ret;
}

template <
    typename A, typename B,
    typename = std::enable_if_t<
        is_vec3_like_v<A> &&
        is_vec3_like_v<B> &&
        std::is_same_v<vec3_scalar_t<A>, vec3_scalar_t<B>>
    >
>
inline vec3_base<vec3_scalar_t<A>> max(const A& a, const B& b) {
    using T = vec3_scalar_t<A>;
    vec3_base<T> ret;
    ret[0] = a[0] < b[0] ? b[0] : a[0];
    ret[1] = a[1] < b[1] ? b[1] : a[1];
    ret[2] = a[2] < b[2] ? b[2] : a[2];
    return ret;
}

template <
    typename A, typename B, typename C,
    typename = std::enable_if_t<
        is_vec3_like_v<A> &&
        is_vec3_like_v<B> &&
        is_vec3_like_v<C> &&
        std::is_same_v<vec3_scalar_t<A>, vec3_scalar_t<B>> &&
        std::is_same_v<vec3_scalar_t<A>, vec3_scalar_t<C>>
    >
>
inline vec3_base<vec3_scalar_t<A>> min(const A& a, const B& b, const C& c) {
    return min(min(a, b), c);
}

template <
    typename A, typename B, typename C,
    typename = std::enable_if_t<
        is_vec3_like_v<A> &&
        is_vec3_like_v<B> &&
        is_vec3_like_v<C> &&
        std::is_same_v<vec3_scalar_t<A>, vec3_scalar_t<B>> &&
        std::is_same_v<vec3_scalar_t<A>, vec3_scalar_t<C>>
    >
>
inline vec3_base<vec3_scalar_t<A>> max(const A& a, const B& b, const C& c) {
    return max(max(a, b), c);
}

}; // end namespace


#endif
