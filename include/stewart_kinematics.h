#ifndef STEWART_KINEMATICS_H
#define STEWART_KINEMATICS_H

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#ifndef DTCM_ATTR
#define DTCM_ATTR
#endif

#define __attribute__((section(".dtcm"))) DTCM_ATTR

struct Pose6D {
    double x;
    double y;
    double z;
    double phi;
    double theta;
    double psi;
};

struct Quaternion {
    double w;
    double x;
    double y;
    double z;
};

struct Point3D {
    double x;
    double y;
    double z;
};

struct ActuatorState {
    double current_length;
    double target_length;
    double velocity;
    double acceleration;
    bool limit_switch_min;
    bool limit_switch_max;
    bool error_state;
};

struct PlatformGeometry {
    double base_radius;
    double base_angle_offset;
    
    double top_radius;
    double top_angle_offset;
    
    double min_leg_length;
    double max_leg_length;
    double actuator_offset;
    
    double max_translation;
    double max_rotation;
    
    Point3D base_points[6];
    Point3D top_points[6];
};

struct IKResult {
    ActuatorState actuators[6];
    bool solution_valid;
    uint8_t error_code;
    double computation_time_us;
};

class StewartKinematics {
private:
    PlatformGeometry geometry;
    Pose6D current_pose;
    Pose6D target_pose;
    
    double rotation_matrix[3][3];
    
    double temp_vectors[6][3];
    double leg_lengths_squared[6];
    
    void computeRotationMatrix(double phi, double theta, double psi);
    void quaternionToRotationMatrix(const Quaternion& q);
    void quaternionToEuler(const Quaternion& q, double& phi, double& theta, double& psi);
    Point3D transformPoint(const Point3D& local_point, const Point3D& global_position);
    bool checkWorkspaceLimits(const double leg_lengths[6]);
    bool checkSingularity(const Pose6D& pose);
    void computeJacobian(const Pose6D& pose, double jacobian[6][6]);
    void initializeGeometry();

public:
    StewartKinematics();
    StewartKinematics(const PlatformGeometry& geo);
    
    IKResult solveIK(const Pose6D& target_pose);
    IKResult solveIKQuaternion(const Point3D& position, const Quaternion& orientation);
    
    void setCurrentPose(const Pose6D& pose);
    Pose6D getCurrentPose() const;
    
    void updateGeometry(const PlatformGeometry& geo);
    PlatformGeometry getGeometry() const;
    
    double legLengthToServoAngle(double leg_length, uint8_t leg_index);
    Pose6D solveFK(const double leg_lengths[6]);
    bool validatePose(const Pose6D& pose);
};

class TrajectoryGenerator {
private:
    Pose6D start_pose;
    Pose6D end_pose;
    Pose6D current_pose;
    
    double trajectory_duration;
    double current_time;
    
    double max_velocity;
    double max_acceleration;
    
    bool trajectory_active;
    
    double computeSCurve(double start, double end, double t, double duration);

public:
    TrajectoryGenerator();
    
    void initTrajectory(const Pose6D& start, const Pose6D& end, 
                        double duration, bool use_scurve = true);
    Pose6D getNextPose(double dt);
    bool isTrajectoryComplete();
    void reset();
};

#endif
