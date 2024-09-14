#ifndef HEURISTIC_MANAGER__H
#define HEURISTIC_MANAGER__H

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>




class RotationQuaternion {
private:
    double w, x, y, z;

public:
    // Constructor
    RotationQuaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
    RotationQuaternion(std::array<double,4> rotQuat) : w(rotQuat[0]), x(rotQuat[1]),y(rotQuat[2]),z(rotQuat[3]){}

    // Normalize the quaternion
    void normalize() {
        double norm = std::sqrt(w*w + x*x + y*y + z*z);
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    // Quaternion multiplication
    RotationQuaternion operator*(const RotationQuaternion& q) const {
        return RotationQuaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
    RotationQuaternion operator*(double scalar) const {
        return RotationQuaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    RotationQuaternion operator+(const RotationQuaternion& q) const {
        return RotationQuaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    RotationQuaternion operator-(const RotationQuaternion& q) const {
        return RotationQuaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }

    // Inverse of the quaternion
    RotationQuaternion inverse() const {
        return RotationQuaternion(w, -x, -y, -z);
    }

    // Distance between two quaternions
    static double distance(const RotationQuaternion& q1, const RotationQuaternion& q2) {
        double dot_product = std::fabs(q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);
        dot_product = std::clamp(dot_product, -1.0, 1.0);
        return 2 * std::acos(dot_product);
    }

    static RotationQuaternion slerp(const RotationQuaternion& q1, const RotationQuaternion& q2, double t) {
        double dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
        const double THRESHOLD = 0.9995;

        if (dot > THRESHOLD) {
            // Linear interpolation for very close quaternions
            RotationQuaternion result = RotationQuaternion(
                q1.w + t * (q2.w - q1.w),
                q1.x + t * (q2.x - q1.x),
                q1.y + t * (q2.y - q1.y),
                q1.z + t * (q2.z - q1.z)
            );
            result.normalize();
            return result;
        }

        dot = std::clamp(dot, -1.0, 1.0);
        double theta_0 = std::acos(dot);        // theta_0 is the angle between input quaternions
        double theta = theta_0 * t;             // theta is the angle between q1 and the result

        RotationQuaternion q3 = q2 - q1 * dot;
        q3.normalize();

        return q1 * std::cos(theta) + q3 * std::sin(theta);
    }

    bool operator==(const RotationQuaternion& q) const {
        return (std::fabs(w - q.w) < 1e-9 && std::fabs(x - q.x) < 1e-9 && std::fabs(y - q.y) < 1e-9 && std::fabs(z - q.z) < 1e-9) ||
               (std::fabs(w + q.w) < 1e-9 && std::fabs(x + q.x) < 1e-9 && std::fabs(y + q.y) < 1e-9 && std::fabs(z + q.z) < 1e-9);
    }

    // Print the quaternion
    void print() const {
        std::cout << "Quaternion: (" << w << ", " << x << ", " << y << ", " << z << ")" << std::endl;
    }

    // Getters
    double getW() const { return w; }
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
};




class Heuristic {
public:
    virtual ~Heuristic() {}
    virtual double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const = 0;
};


class EuclideanDistance : public Heuristic {
public:
    double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const override {
        int sum = 0;
        for (int i = 0; i < 3; ++i) {
            int diff = a[i] - b[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }
};


class ManhattanDistance : public Heuristic {
public:
    double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const override {
        int sum = 0;
        for (int i = 0; i < 3; ++i) {
            int diff = std::abs(a[i] - b[i]);
            sum += diff;
        }
        return sum;
    }
};

class InadManhattanDistance : public Heuristic {
public:
    double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const override {
        int sum = 0;
        for (int i = 0; i < 3; ++i) {
            int diff = std::abs(a[i] - b[i]);
            sum += diff;
        }
        return 10*sum;
    }
};


class HeuristicManager {
private:
    std::map<std::string, std::unique_ptr<Heuristic>> heuristics;
    std::vector<std::string> heuristicIndex;

public:
    
    void registerHeuristic(const std::string& name, std::unique_ptr<Heuristic> heuristic) {
        heuristics[name] = std::move(heuristic);
        heuristicIndex.push_back(name);
    }

    
    const Heuristic* getHeuristic(const std::string& name) const {
        auto it = heuristics.find(name);
        if (it != heuristics.end()) {
            return it->second.get();
        }
        return nullptr;
    }

    const Heuristic* getHeuristicByIndex(int i) const {
        std::string name = heuristicIndex.at(i);
        auto it = heuristics.find(name);
        if (it != heuristics.end()) {
            return it->second.get();
        }
        return nullptr;
    }

    
    size_t countHeuristics() const {
        return heuristics.size();
    }

    void printAllHeuristic()
    {
        //for (auto const & heurist: heuristicIndex)
        //{
        //    std::cout << heurist << std::endl;
        //}
        for (size_t i = 0; i < countHeuristics();++i)
        {
            std::cout << heuristicIndex[i] << std::endl;
        }
    }
};

#endif //HEURISTIC_MANAGER__H