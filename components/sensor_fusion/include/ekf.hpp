#pragma once
#include "sensor_fusion.hpp"
#include "linalg.hpp"
#include <array>

class ExtendedKalmanFusion final: public IFusion { 
    uint64_t latest_timestamp_us_;
    uint64_t min_timestep_us_; 
    
    linalg::Vector<float, 4> q_; 
    linalg::Vector<float, 3> b_; 
    linalg::Matrix<float, 7, 7> cov_; 
    linalg::Matrix<float, 7, 7> proc_noise_; 
    linalg::Matrix<float, 3, 3> meas_noise_;

    void normalize_quaternion_();
    linalg::Matrix<float, 4, 4> omega_matrix_(const linalg::Vector<float, 3> &omega) const noexcept;  
    linalg::Matrix<float, 7, 7> compute_f_jacobian_(const linalg::Vector<float, 3> &gyro, float dt) const noexcept; 
    linalg::Matrix<float, 3, 3> quaternion_to_rotation_matrix_() const noexcept; 
    linalg::Matrix<float, 3, 7> compute_h_jacobian_() const noexcept;

    void predict(const linalg::Vector<float, 3> &gyro, float dt) noexcept; 
    void update(const linalg::Vector<float, 3> &accel) noexcept; 
public:
    ExtendedKalmanFusion(const ImuSample *init = nullptr, const uint64_t min_timestep_us_ = 100);
    Attitude update(const ImuSample &sample) override;
}; 