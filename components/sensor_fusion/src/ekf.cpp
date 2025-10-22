#include <cmath>
#include "ekf.hpp"
#include "linalg.hpp"

const float GRAVITY = 9.81;  

ExtendedKalmanFusion::ExtendedKalmanFusion(const ImuSample *init = nullptr, const uint64_t min_timestep_us = 100): 
    q_ {1.0f, 0.0f, 0.0f, 0.0f}, b_ {0.0f}, cov_ { linalg::matrix_identity<float, 7, 7>() }, proc_noise_ {0.0f}, meas_noise_ {0.0f}, min_timestep_us_ { min_timestep_us }
{
    if (init != nullptr) {
        float norm = sqrtf(powf(init->acc[0], 2) + powf(init->acc[1], 2) + powf(init->acc[2], 2));
        q_[0] = sqrtf(1.0f - init->acc[2] / norm) / 2.0f; 
        q_[1] = -init->acc[1] / (2.0f * q_[0]); 
        q_[2] = init->acc[0] / (2.0f * q_[0]); 
        q_[3] = 0.0f; 

        normalize_quaternion_();
    }

    proc_noise_[0 + 7*0] = 0.05f; // q0
    proc_noise_[1 + 7*1] = 0.05f; // q1
    proc_noise_[2 + 7*2] = 0.05f; // q2
    proc_noise_[3 + 7*3] = 0.05f; // q3
    proc_noise_[4 + 7*4] = 0.01f; // bx
    proc_noise_[5 + 7*5] = 0.01f; // by
    proc_noise_[6 + 7*6] = 0.01f; // bz

    meas_noise_[0 + 3*0] = 0.02f; // accel x
    meas_noise_[1 + 3*1] = 0.02f; // accel y
    meas_noise_[2 + 3*2] = 0.02f; // accel z
}

inline void ExtendedKalmanFusion::normalize_quaternion_() {
    float norm = sqrtf(powf(q_[0], 2) + powf(q_[1], 2) + powf(q_[2], 2) + powf(q_[3], 2));
    q_[0] /= norm; 
    q_[1] /= norm; 
    q_[2] /= norm; 
    q_[3] /= norm;
}; 

inline std::array<float, 4 * 4> ExtendedKalmanFusion::omega_matrix_(const std::array<float, 3> &omega) const noexcept {
    return std::array<float, 16> {
        0.0f,       -omega[0],  -omega[2],  -omega[3], 
        omega[0],   0.0f,       omega[2],   -omega[1], 
        omega[1],   -omega[2],  0.0f,       omega[0],
        omega[2],   omega[1],   -omega[0],  0.0f,
    }; 
}

inline std::array<float, 7 * 7> ExtendedKalmanFusion::compute_f_jacobian_(const std::array<float, 3> &gyro, float dt) const noexcept {
    float p = gyro[0] - b_[0]; 
    float q = gyro[1] - b_[1]; 
    float r = gyro[2] - b_[2];
    return {
        1.0f,       -p * dt,    -q * dt,    -r * dt,    0.5f * q_[1] * dt,      0.5f * q_[2] * dt,      0.5f * q_[3] * dt, 
        p * dt,     1.0f,       r * dt,     -q * dt,    -0.5f * q_[0] * dt,     -0.5f * q_[3] * dt,     0.5f * q_[2] * dt, 
        q * dt,     -r * dt,    1.0f,       p * dt,     0.5f * q_[3] * dt,      -0.5f * q_[0] * dt,     -0.5f * q_[1] * dt, 
        r * dt,     q * dt,     -p * dt,    1.0f,       -0.5f * q_[2] * dt,     0.5f * q_[1] * dt,      -0.5f * q_[0] * dt,
        0.0f,       0.0f,       0.0f,       0.0f,       1.0f,                   0.0f,                   0.0f, 
        0.0f,       0.0f,       0.0f,       0.0f,       0.0f,                   1.0f,                   0.0f, 
        0.0f,       0.0f,       0.0f,       0.0f,       0.0f,                   0.0f,                   1.0f, 
    };
}

// Predict Step to propogate state and covariance forward using gyro data
void ExtendedKalmanFusion::predict(const std::array<float, 3> &gyro, float dt) noexcept {
    // 1. remove bias from gyro measurement
    linalg::Vector<float, 3> omega { gyro[0] - b_[0], gyro[1] - b_[1], gyro[2] - b_[2] };

    // 2. Integrate angular rate (dq/dt = 0.5 * \Omega(omega) * q)
    linalg::Vector<float, 4> q_dot;
    linalg::Matrix<float, 4, 4> omega_matrix = omega_matrix_(omega); 

    linalg::matrix_vector_multiply(omega_matrix, q_, q_dot);
    q_dot[0] *= 0.5f; 
    q_dot[1] *= 0.5f;
    q_dot[2] *= 0.5f;
    q_dot[3] *= 0.5f;

    // 3. Update state with new quaternion
    q_[0] = q_[0] + q_dot[0] * dt; 
    q_[1] = q_[1] + q_dot[1] * dt; 
    q_[2] = q_[2] + q_dot[2] * dt; 
    q_[3] = q_[3] + q_dot[3] * dt; 
    normalize_quaternion_(); 

    // 4. Compute Jacobian of motion model 
    linalg::Matrix<float, 7, 7> f_jacobian { compute_f_jacobian_(gyro, dt) }; 

    // 5. Propagate uncertainty using the covariance update: P' = FPFᵀ + Q
    linalg::Matrix<float, 7, 7> tmp {}; 
    linalg::Matrix<float, 7, 7> f_t {}; 

    linalg::matrix_transpose<float, 7, 7>(f_jacobian, f_t); 
    linalg::matrix_multiply<float, 7, 7, 7>(cov_, f_t, tmp); 
    linalg::matrix_multiply<float, 7, 7, 7>(f_jacobian, tmp, cov_); 
    linalg::matrix_add<float, 7, 7>(cov_, proc_noise_, cov_);  
}

inline std::array<float, 3 * 3> ExtendedKalmanFusion::quaternion_to_rotation_matrix_() const noexcept {
    return {
        1.0f - 2.0f * (q_[2] * q_[2] + q_[3] * q_[3]), 
        2.0f * (q_[1] * q_[2] - q_[0] * q_[3]), 
        2.0f * (q_[1] * q_[3] + q_[0] * q_[2]),
        2.0f * (q_[1] * q_[2] + q_[0] * q_[3]),
        1.0f - 2.0f * (q_[1] * q_[1] + q_[3] * q_[3]),
        2.0f * (q_[2] * q_[3] - q_[0] * q_[1]),
        2.0f * (q_[1] * q_[3] - q_[0] * q_[2]),
        2.0f * (q_[2] * q_[3] + q_[0] * q_[1]),
        1.0f - 2.0f * (q_[1] * q_[1] + q_[2] * q_[2])
    }; 
}

inline std::array<float, 3 * 7> ExtendedKalmanFusion::compute_h_jacobian_() const noexcept {
    return {
        2.0f * (-GRAVITY * q_[2]), 2.0f * (GRAVITY * q_[3]),  2.0f * (-GRAVITY * q_[0]), 2.0f * (GRAVITY * q_[1]),  0.0f, 0.0f, 0.0f, 
        2.0f * (GRAVITY * q_[1]),  2.0f * (GRAVITY * q_[0]),  2.0f * (GRAVITY * q_[3]),  2.0f * (GRAVITY * q_[2]),  0.0f, 0.0f, 0.0f, 
        2.0f * (GRAVITY * q_[0]),  2.0f * (-GRAVITY * q_[1]), 2.0f * (-GRAVITY * q_[2]), 2.0f * (-GRAVITY * q_[3]), 0.0f, 0.0f, 0.0f
    }; 
}

void ExtendedKalmanFusion::update(const std::array<float, 3> &accel) noexcept {
    // 1. Compute expected gravity vector in sensor frame
    linalg::Vector<float, 3> accel_expected;
    {
        linalg::Vector<float, 3> gravity { 0.0f, 0.0f, -GRAVITY }; 
        linalg::Matrix<float, 3, 3> r { quaternion_to_rotation_matrix_() }; 

        linalg::Matrix<float, 3, 3> r_tr;
        linalg::matrix_transpose<float, 3, 3>(r, r_tr); 
    
        linalg::matrix_vector_multiply(r_tr, gravity, accel_expected); 
    }

    // 2. Compute innovation (measurement residual): z - h(x)
    linalg::Vector<float, 3> innovation { 
        accel[0] - accel_expected[0], 
        accel[1] - accel_expected[1], 
        accel[2] - accel_expected[2] 
    };

    // 3. Compute Jacobian of measurement model
    linalg::Matrix<float, 3, 7> h_jacobian = compute_h_jacobian_(); 

    // 4. Compute innovation covariance: S = HPHᵀ + R
    linalg::Matrix<float, 3, 3> s; 

    linalg::Matrix<float, 7, 3> h_tr; 
    linalg::matrix_transpose<float, 3, 7>(h_jacobian, h_tr);
    {
        linalg::Matrix<float, 7, 3> tmp; 
        linalg::matrix_multiply<float, 7, 7, 3>(cov_, h_tr, tmp); 
        linalg::matrix_multiply<float, 3, 7, 3>(h_jacobian, tmp, s); 
        linalg::matrix_add<float, 3, 3>(s, meas_noise_, s);
    }

    // 5. Compute Kalman Gain K = PHᵀS⁻¹
    linalg::Matrix<float, 3, 3> s_inv; 
    if (linalg::matrix_inverse_3x3(s, s_inv) == 0) {
        linalg::Matrix<float, 7, 3> K;
        {
            linalg::Matrix<float, 7, 3> tmp; 
            linalg::matrix_multiply<float, 7, 3, 3>(h_tr, s_inv, tmp); 
            linalg::matrix_multiply<float, 7, 7, 3>(cov_, tmp, K); 
        }
        
        // 6. Update state: x = x + K(z - h(x))
        {
            linalg::Vector<float, 7> delta; 
            linalg::matrix_vector_multiply(K, innovation, delta); 
            q_[0] += delta[0]; 
            q_[1] += delta[1]; 
            q_[2] += delta[2]; 
            q_[3] += delta[3];
            b_[0] += delta[4]; 
            b_[1] += delta[5]; 
            b_[2] += delta[6];
        } 

        // 7. Update covariance: P = (I - KH)P 
        {
            std::array<float, 7*7> tmp; 
            std::array<float, 7*7> tmp2;
            linalg::matrix_multiply<float, 7, 3, 7>(K, h_jacobian, tmp); 
            linalg::matrix_subtract<float, 7, 7>(
                linalg::matrix_identity<float, 7, 7>(), tmp, tmp2
            ); 
            linalg::matrix_multiply<float, 7, 7, 7>(tmp2, cov_, tmp); 
            cov_ = tmp; 
        }

        // 8. Normalize quaternion after update
        normalize_quaternion_(); 
    } else {
        // TODO: do something if s not invertible
        normalize_quaternion_(); 
    }
}

Attitude ExtendedKalmanFusion::update(const ImuSample &sample) {
    const uint64_t dt_us = (sample.timestamp_us > latest_timestamp_us_)
        ? (sample.timestamp_us - latest_timestamp_us_)
        : 0;

    if (dt_us < min_timestep_us_) {
        return Attitude { sample.timestamp_us, q_, b_ };
    }

    const float dt = static_cast<float>(dt_us) * 1e-6f;
    predict(sample.gyro, dt);
    update(sample.acc);

    latest_timestamp_us_ = sample.timestamp_us;

    return Attitude { sample.timestamp_us, q_, b_ };
}