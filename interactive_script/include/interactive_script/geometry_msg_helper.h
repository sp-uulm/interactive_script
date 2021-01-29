#ifndef GEOMETRY_MSG_HELPER_H
#define GEOMETRY_MSG_HELPER_H

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <tuple>

namespace geometry_msgs {

using namespace geometry_msgs::msg;

// helper functions

inline double roll(const Quaternion& q, double reference = 0) {
    double result = std::fmod(3*M_PI - std::atan2(2*q.y*q.z + 2*q.w*q.x, -q.w*q.w + q.x*q.x + q.y*q.y - q.z*q.z), 2*M_PI);

    return result + 2*M_PI * round((reference - result) / (2*M_PI));
}

inline double pitch(const Quaternion& q) {
    return std::asin(2*q.w*q.y - 2*q.x*q.z);
}

/*
 * calculate yaw from quaternion using the formula from
 *
 *    http://www.tinkerforge.com/en/doc/Software/Bricks/IMU_Brick_CSharp.html
 *
 * the result is then normalized to the range [reference-M_PI; reference+M_PI]
 */
inline double yaw(const Quaternion& q, double reference = M_PI) {
    double result = std::fmod(2*M_PI + std::atan2(2*q.x*q.y + 2*q.w*q.z, q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z), 2*M_PI);

    return result + 2*M_PI * round((reference - result) / (2*M_PI));
}

inline std::tuple<double, double, double> rpy(const Quaternion& q) {
    return std::make_tuple(roll(q),
                           pitch(q),
                           yaw(q));
}

// constructors

inline Quaternion quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion result;
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    result.w = q.w();
    return result;
}

inline Quaternion quaternion() {
    Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
    return q;
}

inline Vector3Stamped vec3(double x, double y, double z,
                           const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    Vector3Stamped vs;
    vs.vector.x = x;
    vs.vector.y = y;
    vs.vector.z = z;
    vs.header.frame_id = frame_id;
    vs.header.stamp = time;
    return vs;
}

inline Vector3Stamped vec3(const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    return vec3(0.0, 0.0, 0.0, frame_id, time);
}

inline Vector3 vec3(double x, double y, double z) {
    Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

inline Vector3 vec3() {
    return vec3(0.0, 0.0, 0.0);
}

inline PointStamped point(double x, double y, double z,
                          const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    PointStamped ps;
    ps.point.x = x;
    ps.point.y = y;
    ps.point.z = z;
    ps.header.frame_id = frame_id;
    ps.header.stamp = time;
    return ps;
}

inline PointStamped point(const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    return point(0.0, 0.0, 0.0, frame_id, time);
}

inline Point point(double x, double y, double z) {
    Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

inline Point point() {
    return point(0.0, 0.0, 0.0);
}

inline PoseStamped pose(double x, double y, double z, double yaw,
                        const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    PoseStamped ps;
    ps.pose.position = point(x, y, z);
    ps.pose.orientation = quaternion(0.0, 0.0, yaw);
    ps.header.frame_id = frame_id;
    ps.header.stamp = time;
    return ps;
}

inline PoseStamped pose(double x, double y, double z,
                        const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    PoseStamped ps;
    ps.pose.position = point(x, y, z);
    ps.pose.orientation = quaternion();
    ps.header.frame_id = frame_id;
    ps.header.stamp = time;
    return ps;
}

inline PoseStamped pose(const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    return pose(0.0, 0.0, 0.0, frame_id, time);
}

inline Pose pose(double x, double y, double z, double yaw = 0.0) {
    Pose p;
    p.position = point(x, y, z);
    p.orientation = quaternion(0.0, 0.0, yaw);
    return p;
}

inline Pose pose() {
    return pose(0.0, 0.0, 0.0);
}

inline PoseStamped origin(const std::string& frame_id, rclcpp::Time time = rclcpp::Time{0}) {
    return pose(frame_id, time);
}

inline Pose origin() {
    return pose();
}

template <typename T>
inline Vector3 to_vec3(T&& o) {
    return vec3(o.x, o.y, o.z);
}

template <typename T>
inline Point to_point(T&& o) {
    return point(o.x, o.y, o.z);
}

template <typename T, typename... Args>
inline typename std::enable_if<std::is_same<T, Vector3>::value, Vector3>::type
construct(Args&&... args) {
    return vec3(std::forward<Args>(args)...);
}

template <typename T, typename... Args>
inline typename std::enable_if<std::is_same<T, Point>::value, Point>::type
construct(Args&&... args) {
    return point(std::forward<Args>(args)...);
}

// operators

template <typename T>
inline bool operator!=( const T& t1, const T& t2 )
{
    return !(t1==t2);
}

template <typename T>
inline T operator-(const T& p1, const T& p2) {
    return construct<T>(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);
}

template <typename T>
inline T operator+(const T& p1, const T& p2) {
    return construct<T>(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z);
}

template <typename T>
inline T operator*(double d, const T& p) {
    return construct<T>(d*p.x, d*p.y, d*p.z);
}

template <typename T>
inline T operator*(const T& p, double d) {
    return construct<T>(d*p.x, d*p.y, d*p.z);
}

template <typename T>
inline T& operator+=(T& lhs, const T& rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;

    return lhs;
}

template <typename T>
inline constexpr double norm3(const T& t) {
    return sqrt(t.x*t.x + t.y*t.y + t.z*t.z);
}

template <typename T>
inline constexpr double norm2(const T& t) {
    return sqrt(t.x*t.x + t.y*t.y);
}

template <typename T>
inline double dist3(const T& a, const T& b) {
    return norm3(b-a);
}

template <typename T>
inline double dist2(const T& a, const T& b) {
    return norm2(b-a);
}

}

#endif // GEOMETRY_MSG_HELPER_H
