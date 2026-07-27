#include "stewart_kinematics.h"
#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "hardware_config.h"

using std::cos;
using std::sin;
using std::sqrt;
using std::fabs;
using std::atan2;
using std::copysign;

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double SQRT3 = 1.73205080756887729353;

StewartKinematics::StewartKinematics() {
    geometry.base_radius = DEFAULT_BASE_RADIUS;
    geometry.top_radius = DEFAULT_TOP_RADIUS;
    geometry.base_angle_offset = DEFAULT_BASE_ANGLE_OFFSET;
    geometry.top_angle_offset = DEFAULT_TOP_ANGLE_OFFSET;
    geometry.min_leg_length = DEFAULT_MIN_LEG_LENGTH;
    geometry.max_leg_length = DEFAULT_MAX_LEG_LENGTH;
    geometry.actuator_offset = DEFAULT_ACTUATOR_OFFSET;
    geometry.max_translation = DEFAULT_MAX_TRANSLATION;
    geometry.max_rotation = DEFAULT_MAX_ROTATION;
    
    initializeGeometry();
    
    current_pose = {HOME_POSITION.x, HOME_POSITION.y, HOME_POSITION.z, 
                    HOME_POSITION.phi, HOME_POSITION.theta, HOME_POSITION.psi};
    target_pose = current_pose;
}

StewartKinematics::StewartKinematics(const PlatformGeometry& geo) {
    geometry = geo;
    initializeGeometry();
    current_pose = {HOME_POSITION.x, HOME_POSITION.y, HOME_POSITION.z, 
                    HOME_POSITION.phi, HOME_POSITION.theta, HOME_POSITION.psi};
    target_pose = current_pose;
}

void StewartKinematics::initializeGeometry() {
    for (int i = 0; i < 6; i++) {
        double angle = geometry.base_angle_offset + i * PI / 3.0;
        geometry.base_points[i].x = geometry.base_radius * cos(angle);
        geometry.base_points[i].y = geometry.base_radius * sin(angle);
        geometry.base_points[i].z = 0.0;
    }
    
    for (int i = 0; i < 6; i++) {
        double angle = geometry.top_angle_offset + i * PI / 3.0;
        geometry.top_points[i].x = geometry.top_radius * cos(angle);
        geometry.top_points[i].y = geometry.top_radius * sin(angle);
        geometry.top_points[i].z = 0.0;
    }
}

void StewartKinematics::computeRotationMatrix(double phi, double theta, double psi) {
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_psi = cos(psi);
    double sin_psi = sin(psi);
    
    rotation_matrix[0][0] = cos_theta * cos_psi;
    rotation_matrix[0][1] = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
    rotation_matrix[0][2] = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
    
    rotation_matrix[1][0] = cos_theta * sin_psi;
    rotation_matrix[1][1] = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
    rotation_matrix[1][2] = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
    
    rotation_matrix[2][0] = -sin_theta;
    rotation_matrix[2][1] = sin_phi * cos_theta;
    rotation_matrix[2][2] = cos_phi * cos_theta;
}

void StewartKinematics::quaternionToRotationMatrix(const Quaternion& q) {
    double w = q.w;
    double x = q.x;
    double y = q.y;
    double z = q.z;
    
    double norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm > 1e-6) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
    
    double xx = x * x;
    double yy = y * y;
    double zz = z * z;
    double xy = x * y;
    double xz = x * z;
    double yz = y * z;
    double wx = w * x;
    double wy = w * y;
    double wz = w * z;
    
    rotation_matrix[0][0] = 1.0 - 2.0 * (yy + zz);
    rotation_matrix[0][1] = 2.0 * (xy - wz);
    rotation_matrix[0][2] = 2.0 * (xz + wy);
    
    rotation_matrix[1][0] = 2.0 * (xy + wz);
    rotation_matrix[1][1] = 1.0 - 2.0 * (xx + zz);
    rotation_matrix[1][2] = 2.0 * (yz - wx);
    
    rotation_matrix[2][0] = 2.0 * (xz - wy);
    rotation_matrix[2][1] = 2.0 * (yz + wx);
    rotation_matrix[2][2] = 1.0 - 2.0 * (xx + yy);
}

Point3D StewartKinematics::transformPoint(const Point3D& local_point, const Point3D& global_position) {
    Point3D transformed;
    
    transformed.x = rotation_matrix[0][0] * local_point.x + 
                    rotation_matrix[0][1] * local_point.y + 
                    rotation_matrix[0][2] * local_point.z;
    
    transformed.y = rotation_matrix[1][0] * local_point.x + 
                    rotation_matrix[1][1] * local_point.y + 
                    rotation_matrix[1][2] * local_point.z;
    
    transformed.z = rotation_matrix[2][0] * local_point.x + 
                    rotation_matrix[2][1] * local_point.y + 
                    rotation_matrix[2][2] * local_point.z;
    
    transformed.x += global_position.x;
    transformed.y += global_position.y;
    transformed.z += global_position.z;
    
    return transformed;
}

bool StewartKinematics::checkWorkspaceLimits(const double leg_lengths[6]) {
    for (int i = 0; i < 6; i++) {
        if (leg_lengths[i] < geometry.min_leg_length || 
            leg_lengths[i] > geometry.max_leg_length) {
            return false;
        }
    }
    return true;
}

bool StewartKinematics::checkSingularity(const Pose6D& pose) {
    double jacobian[6][6];
    computeJacobian(pose, jacobian);
    
    double min_singular = 1e9;
    double max_singular = 0.0;
    
    for (int i = 0; i < 6; i++) {
        double abs_val = fabs(jacobian[i][i]);
        if (abs_val < min_singular) min_singular = abs_val;
        if (abs_val > max_singular) max_singular = abs_val;
    }
    
    if (max_singular > 0 && min_singular > 0) {
        double condition_number = max_singular / min_singular;
        return condition_number > 100.0;
    }
    
    return false;
}

void StewartKinematics::computeJacobian(const Pose6D& pose, double jacobian[6][6]) {
    computeRotationMatrix(pose.phi, pose.theta, pose.psi);
    
    Point3D global_position = {pose.x, pose.y, pose.z};
    
    for (int i = 0; i < 6; i++) {
        Point3D top_global = transformPoint(geometry.top_points[i], global_position);
        
        Point3D leg_vector;
        leg_vector.x = top_global.x - geometry.base_points[i].x;
        leg_vector.y = top_global.y - geometry.base_points[i].y;
        leg_vector.z = top_global.z - geometry.base_points[i].z;
        
        double leg_length = sqrt(leg_vector.x * leg_vector.x + 
                                 leg_vector.y * leg_vector.y + 
                                 leg_vector.z * leg_vector.z);
        
        if (leg_length > 1e-6) {
            leg_vector.x /= leg_length;
            leg_vector.y /= leg_length;
            leg_vector.z /= leg_length;
        }
        
        jacobian[i][0] = leg_vector.x;
        jacobian[i][1] = leg_vector.y;
        jacobian[i][2] = leg_vector.z;
        
        jacobian[i][3] = leg_vector.y * top_global.z - leg_vector.z * top_global.y;
        jacobian[i][4] = leg_vector.z * top_global.x - leg_vector.x * top_global.z;
        jacobian[i][5] = leg_vector.x * top_global.y - leg_vector.y * top_global.x;
    }
}

IKResult StewartKinematics::solveIK(const Pose6D& target_pose_input) {
    IKResult result;
    result.solution_valid = false;
    result.error_code = 0;
    
    uint32_t start_time = micros();
    
    computeRotationMatrix(target_pose_input.phi, target_pose_input.theta, target_pose_input.psi);
    
    Point3D global_position = {target_pose_input.x, target_pose_input.y, target_pose_input.z};
    
    for (int i = 0; i < 6; i++) {
        Point3D top_global = transformPoint(geometry.top_points[i], global_position);
        
        temp_vectors[i][0] = top_global.x - geometry.base_points[i].x;
        temp_vectors[i][1] = top_global.y - geometry.base_points[i].y;
        temp_vectors[i][2] = top_global.z - geometry.base_points[i].z;
        
        leg_lengths_squared[i] = temp_vectors[i][0] * temp_vectors[i][0] + 
                                temp_vectors[i][1] * temp_vectors[i][1] + 
                                temp_vectors[i][2] * temp_vectors[i][2];
        
        result.actuators[i].target_length = sqrt(leg_lengths_squared[i]);
    }
    
    if (!checkWorkspaceLimits(&result.actuators[0].target_length)) {
        result.error_code = 1;
        result.computation_time_us = micros() - start_time;
        return result;
    }
    
    if (checkSingularity(target_pose_input)) {
        result.error_code = 2;
        result.computation_time_us = micros() - start_time;
        return result;
    }
    
    result.solution_valid = true;
    target_pose = target_pose_input;
    
    for (int i = 0; i < 6; i++) {
        result.actuators[i].current_length = result.actuators[i].target_length;
        result.actuators[i].velocity = 0.0;
        result.actuators[i].acceleration = 0.0;
        result.actuators[i].error_state = false;
    }
    
    result.computation_time_us = micros() - start_time;
    return result;
}

IKResult StewartKinematics::solveIKQuaternion(const Point3D& position, const Quaternion& orientation) {
    Pose6D pose;
    pose.x = position.x;
    pose.y = position.y;
    pose.z = position.z;
    quaternionToEuler(orientation, pose.phi, pose.theta, pose.psi);
    
    return solveIK(pose);
}

void StewartKinematics::quaternionToEuler(const Quaternion& q, double& phi, double& theta, double& psi) {
    double w = q.w;
    double x = q.x;
    double y = q.y;
    double z = q.z;
    
    double norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm > 1e-6) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
    
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    phi = atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1.0) {
        theta = copysign(PI / 2.0, sinp);
    } else {
        theta = asin(sinp);
    }
    
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    psi = atan2(siny_cosp, cosy_cosp);
}

void StewartKinematics::setCurrentPose(const Pose6D& pose) {
    current_pose = pose;
}

Pose6D StewartKinematics::getCurrentPose() const {
    return current_pose;
}

void StewartKinematics::updateGeometry(const PlatformGeometry& geo) {
    geometry = geo;
    initializeGeometry();
}

PlatformGeometry StewartKinematics::getGeometry() const {
    return geometry;
}

double StewartKinematics::legLengthToServoAngle(double leg_length, uint8_t leg_index) {
    const double servo_horn_length = 30.0;
    const double connection_length = 80.0;
    const double base_offset = 20.0;
    
    double effective_length = leg_length - base_offset;
    
    double numerator = servo_horn_length * servo_horn_length + 
                       effective_length * effective_length - 
                       connection_length * connection_length;
    double denominator = 2.0 * servo_horn_length * effective_length;
    
    double cos_angle = numerator / denominator;
    
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;
    
    double angle_rad = acos(cos_angle);
    
    double angle_deg = angle_rad * RAD_TO_DEG;
    
    angle_deg += leg_index * 60.0;
    
    return angle_deg;
}

Pose6D StewartKinematics::solveFK(const double leg_lengths[6]) {
    Pose6D pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.z = 150.0;
    pose.phi = 0.0;
    pose.theta = 0.0;
    pose.psi = 0.0;
    
    const int max_iterations = 10;
    const double tolerance = 1e-6;
    
    for (int iter = 0; iter < max_iterations; iter++) {
        IKResult ik_result = solveIK(pose);
        
        double error[6];
        double max_error = 0.0;
        for (int i = 0; i < 6; i++) {
            error[i] = leg_lengths[i] - ik_result.actuators[i].target_length;
            if (fabs(error[i]) > max_error) {
                max_error = fabs(error[i]);
            }
        }
        
        if (max_error < tolerance) {
            break;
        }
        
        double jacobian[6][6];
        computeJacobian(pose, jacobian);
        
        double step_size = 0.1;
        pose.x += error[0] * step_size;
        pose.y += error[1] * step_size;
        pose.z += error[2] * step_size;
        pose.phi += error[3] * step_size * 0.01;
        pose.theta += error[4] * step_size * 0.01;
        pose.psi += error[5] * step_size * 0.01;
    }
    
    return pose;
}

bool StewartKinematics::validatePose(const Pose6D& pose) {
    double translation = sqrt(pose.x * pose.x + pose.y * pose.y + pose.z * pose.z);
    if (fabs(pose.x) > geometry.max_translation ||
        fabs(pose.y) > geometry.max_translation ||
        fabs(pose.z - 150.0) > geometry.max_translation) {
        return false;
    }
    
    if (fabs(pose.phi) > geometry.max_rotation ||
        fabs(pose.theta) > geometry.max_rotation ||
        fabs(pose.psi) > geometry.max_rotation) {
        return false;
    }
    
    IKResult result = solveIK(pose);
    return result.solution_valid;
}

TrajectoryGenerator::TrajectoryGenerator() {
    trajectory_duration = 1.0;
    current_time = 0.0;
    max_velocity = 100.0;
    max_acceleration = 500.0;
    trajectory_active = false;
    
    start_pose = {0.0, 0.0, 150.0, 0.0, 0.0, 0.0};
    end_pose = start_pose;
    current_pose = start_pose;
}

void TrajectoryGenerator::initTrajectory(const Pose6D& start, const Pose6D& end, 
                                       double duration, bool use_scurve) {
    start_pose = start;
    end_pose = end;
    trajectory_duration = duration;
    current_time = 0.0;
    trajectory_active = true;
    current_pose = start;
}

double TrajectoryGenerator::computeSCurve(double start, double end, double t, double duration) {
    if (duration <= 0.0) return end;
    if (t <= 0.0) return start;
    if (t >= duration) return end;
    
    double tau = t / duration;
    
    double smooth_tau = tau * tau * (3.0 - 2.0 * tau);
    
    return start + (end - start) * smooth_tau;
}

Pose6D TrajectoryGenerator::getNextPose(double dt) {
    if (!trajectory_active) {
        return current_pose;
    }
    
    current_time += dt;
    
    if (current_time >= trajectory_duration) {
        current_pose = end_pose;
        trajectory_active = false;
        return current_pose;
    }
    
    current_pose.x = computeSCurve(start_pose.x, end_pose.x, current_time, trajectory_duration);
    current_pose.y = computeSCurve(start_pose.y, end_pose.y, current_time, trajectory_duration);
    current_pose.z = computeSCurve(start_pose.z, end_pose.z, current_time, trajectory_duration);
    current_pose.phi = computeSCurve(start_pose.phi, end_pose.phi, current_time, trajectory_duration);
    current_pose.theta = computeSCurve(start_pose.theta, end_pose.theta, current_time, trajectory_duration);
    current_pose.psi = computeSCurve(start_pose.psi, end_pose.psi, current_time, trajectory_duration);
    
    return current_pose;
}

bool TrajectoryGenerator::isTrajectoryComplete() {
    return !trajectory_active;
}

void TrajectoryGenerator::reset() {
    trajectory_active = false;
    current_time = 0.0;
    current_pose = start_pose;
}
