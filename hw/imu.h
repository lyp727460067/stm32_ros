#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HW_IMU_RUN_MODE_NORMAL = 0,
    HW_IMU_RUN_MODE_STANDBY,
    HW_IMU_RUN_MODE_SLEEP,
    HW_IMU_RUN_MODE_END,
} HwImuRunMode;

typedef struct
{
    /// @privatesection
    uint8_t tx_buf_[15];
    volatile uint8_t rx_buf_[15];
    volatile uint8_t* rx_it_;
    volatile bool is_good_;

    volatile uint16_t angle_;
    volatile int16_t x_, y_, z_;
    volatile int16_t gz_;
    HwImuRunMode mode_;
    HwImuRunMode remote_mode_;
    bool is_enable_cal_;
    bool data_valible;
    uint32_t set_mode_at_;
} HwImu;

/**
 * Constructor
 *
 * @param this_
 * @memberof HwImu
 */
void hw_imu_construct(HwImu* const this_);

/**
 * Destructor
 *
 * @param this_
 * @memberof HwImu
 */
void hw_imu_destruct(HwImu* const this_);

/**
 * @memberof HwImu
 */
void hw_imu_set_run_mode(HwImu* const this_, const HwImuRunMode mode);

/**
 * Return the current active run mode
 *
 * @memberof HwImu
 * @see hw_imu_get_set_run_mode()
 */
static inline HwImuRunMode hw_imu_get_run_mode(HwImu* const this_)
{
    assert(this_);
    return this_->remote_mode_;
}

/**
 * Return the current set run mode. This could be different with the active one
 * if the IMU hasn't yet responded
 *
 * @memberof HwImu
 * @see hw_imu_get_run_mode()
 */
static inline HwImuRunMode hw_imu_get_set_run_mode(HwImu* const this_)
{
    assert(this_);
    return this_->mode_;
}

/**
 * @memberof HwImu
 */
void hw_imu_set_enable_calib(HwImu* const this_, const bool flag);

/**
 * @memberof HwImu
 */
static inline bool hw_imu_is_good(HwImu* const this_)
{
    assert(this_);
    return this_->is_good_;
}

/**
 * Return the most current angle
 *
 * @param this_
 * @return Angle in 100x degree
 * @memberof HwImu
 */
static inline uint16_t hw_imu_get_angle(HwImu* const this_)
{
    assert(this_);
    return this_->angle_;
}

/**
 * @memberof HwImu
 */
static inline int16_t hw_imu_get_x(HwImu* const this_)
{
    assert(this_);
    return this_->x_;
}

/**
 * @memberof HwImu
 */
static inline int16_t hw_imu_get_y(HwImu* const this_)
{
    assert(this_);
    return this_->y_;
}

/**
 * @memberof HwImu
 */
static inline int16_t hw_imu_get_z(HwImu* const this_)
{
    assert(this_);
    return this_->z_;
}

/**
 * Important logic routine, must be called frequently and regularly
 *
 * @param this_
 * @memberof HwImu
 */
void hw_imu_update(HwImu* const this_);

#ifdef __cplusplus
}
#endif
