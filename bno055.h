#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>
#include <stdint.h>

#include "nrf_drv_twi.h"

/**
 * @brief BNO055 register addresses for page 0.
 */
typedef enum {
    BNO055_REG_ADDR_CHIP_ID	         = 0x00,
    BNO055_REG_ADDR_ACC_ID	         = 0x01,
    BNO055_REG_ADDR_MAG_ID	         = 0x02,
    BNO055_REG_ADDR_GYR_REV_ID	     = 0x03,
    BNO055_REG_ADDR_SW_REV_ID_LSB	 = 0x04,
    BNO055_REG_ADDR_SW_REV_ID_MSB	 = 0x05,
    BNO055_REG_ADDR_BL_REV_ID	     = 0X06,
    BNO055_REG_ADDR_PAGE_ID	         = 0X07,
    BNO055_REG_ADDR_ACC_DATA_X_LSB   = 0X08,
    BNO055_REG_ADDR_ACC_DATA_X_MSB   = 0X09,
    BNO055_REG_ADDR_ACC_DATA_Y_LSB   = 0X0A,
    BNO055_REG_ADDR_ACC_DATA_Y_MSB   = 0X0B,
    BNO055_REG_ADDR_ACC_DATA_Z_LSB   = 0X0C,
    BNO055_REG_ADDR_ACC_DATA_Z_MSB   = 0X0D,
    BNO055_REG_ADDR_MAG_DATA_X_LSB   = 0X0E,
    BNO055_REG_ADDR_MAG_DATA_X_MSB   = 0X0F,
    BNO055_REG_ADDR_MAG_DATA_Y_LSB   = 0X10,
    BNO055_REG_ADDR_MAG_DATA_Y_MSB   = 0X11,
    BNO055_REG_ADDR_MAG_DATA_Z_LSB   = 0X12,
    BNO055_REG_ADDR_MAG_DATA_Z_MSB   = 0X13,
    BNO055_REG_ADDR_GYR_DATA_X_LSB   = 0X14,
    BNO055_REG_ADDR_GYR_DATA_X_MSB   = 0X15,
    BNO055_REG_ADDR_GYR_DATA_Y_LSB   = 0X16,
    BNO055_REG_ADDR_GYR_DATA_Y_MSB   = 0X17,
    BNO055_REG_ADDR_GYR_DATA_Z_LSB   = 0X18,
    BNO055_REG_ADDR_GYR_DATA_Z_MSB   = 0X19,
    BNO055_REG_ADDR_EUL_HEADING_LSB  = 0X1A,
    BNO055_REG_ADDR_EUL_HEADING_MSB  = 0X1B,
    BNO055_REG_ADDR_EUL_ROLL_LSB	 = 0X1C,
    BNO055_REG_ADDR_EUL_ROLL_MSB	 = 0X1D,
    BNO055_REG_ADDR_EUL_PITCH_LSB	 = 0X1E,
    BNO055_REG_ADDR_EUL_PITCH_MSB	 = 0X1F,
    BNO055_REG_ADDR_QUA_DATA_W_LSB   = 0X20,
    BNO055_REG_ADDR_QUA_DATA_W_MSB   = 0X21,
    BNO055_REG_ADDR_QUA_DATA_X_LSB   = 0X22,
    BNO055_REG_ADDR_QUA_DATA_X_MSB   = 0X23,
    BNO055_REG_ADDR_QUA_DATA_Y_LSB   = 0X24,
    BNO055_REG_ADDR_QUA_DATA_Y_MSB   = 0X25,
    BNO055_REG_ADDR_QUA_DATA_Z_LSB   = 0X26,
    BNO055_REG_ADDR_QUA_DATA_Z_MSB   = 0X27,
    BNO055_REG_ADDR_LIA_DATA_X_LSB   = 0X28,
    BNO055_REG_ADDR_LIA_DATA_X_MSB   = 0X29,
    BNO055_REG_ADDR_LIA_DATA_Y_LSB   = 0X2A,
    BNO055_REG_ADDR_LIA_DATA_Y_MSB   = 0X2B,
    BNO055_REG_ADDR_LIA_DATA_Z_LSB   = 0X2C,
    BNO055_REG_ADDR_LIA_DATA_Z_MSB   = 0X2D,
    BNO055_REG_ADDR_GRV_DATA_X_LSB   = 0X2E,
    BNO055_REG_ADDR_GRV_DATA_X_MSB   = 0X2F,
    BNO055_REG_ADDR_GRV_DATA_Y_LSB   = 0X30,
    BNO055_REG_ADDR_GRV_DATA_Y_MSB   = 0X31,
    BNO055_REG_ADDR_GRV_DATA_Z_LSB   = 0X32,
    BNO055_REG_ADDR_GRV_DATA_Z_MSB   = 0X33,
    BNO055_REG_ADDR_TEMP             = 0X34,
    BNO055_REG_ADDR_CALIB_STAT	     = 0X35,
    BNO055_REG_ADDR_ST_RESULT	     = 0X36,
    BNO055_REG_ADDR_INT_STA	         = 0X37,
    BNO055_REG_ADDR_SYS_CLK_STATUS   = 0X38,
    BNO055_REG_ADDR_SYS_STATUS	     = 0X39,
    BNO055_REG_ADDR_SYS_ERR	         = 0X3A,
    BNO055_REG_ADDR_UNIT_SEL	     = 0X3B,
    /** 0x3C is reserved */
    BNO055_REG_ADDR_OPR_MODE	     = 0X3D,
    BNO055_REG_ADDR_PWR_MODE	     = 0X3E,
    BNO055_REG_ADDR_SYS_TRIGGER	     = 0X3F,
    BNO055_REG_ADDR_TEMP_SOURCE	     = 0X40,
    BNO055_REG_ADDR_AXIS_MAP_CONFIG  = 0X41,
    BNO055_REG_ADDR_AXIS_MAP_SIGN	 = 0X42,
    /** 0x43 - 0x54 are reserved */
    BNO055_REG_ADDR_ACC_OFFSET_X_LSB = 0X55,
    BNO055_REG_ADDR_ACC_OFFSET_X_MSB = 0X56,
    BNO055_REG_ADDR_ACC_OFFSET_Y_LSB = 0X57,
    BNO055_REG_ADDR_ACC_OFFSET_Y_MSB = 0X58,
    BNO055_REG_ADDR_ACC_OFFSET_Z_LSB = 0X59,
    BNO055_REG_ADDR_ACC_OFFSET_Z_MSB = 0X5A,
    BNO055_REG_ADDR_MAG_OFFSET_X_LSB = 0X5B,
    BNO055_REG_ADDR_MAG_OFFSET_X_MSB = 0X5C,
    BNO055_REG_ADDR_MAG_OFFSET_Y_LSB = 0X5D,
    BNO055_REG_ADDR_MAG_OFFSET_Y_MSB = 0X5E,
    BNO055_REG_ADDR_MAG_OFFSET_Z_LSB = 0X5F,
    BNO055_REG_ADDR_MAG_OFFSET_Z_MSB = 0X60,
    BNO055_REG_ADDR_GYR_OFFSET_X_LSB = 0X61,
    BNO055_REG_ADDR_GYR_OFFSET_X_MSB = 0X62,
    BNO055_REG_ADDR_GYR_OFFSET_Y_LSB = 0X63,
    BNO055_REG_ADDR_GYR_OFFSET_Y_MSB = 0X64,
    BNO055_REG_ADDR_GYR_OFFSET_Z_LSB = 0X65,
    BNO055_REG_ADDR_GYR_OFFSET_Z_MSB = 0X66,
    BNO055_REG_ADDR_ACC_RADIUS_LSB   = 0X67,
    BNO055_REG_ADDR_ACC_RADIUS_MSB   = 0X68,
    BNO055_REG_ADDR_MAG_RADIUS_LSB   = 0X69,
    BNO055_REG_ADDR_MAG_RADIUS_MSB   = 0X6A
} bno055_reg_addr_t;

/**
 * @brief BNO055 power modes.
 */
typedef enum {
      BNO055_POWER_MODE_NORMAL   = 0x00,
      BNO055_POWER_MODE_LOWPOWER = 0x01,
      BNO055_POWER_MODE_SUSPEND  = 0x02
} bno055_power_mode_t;

/**
 * @brief BNO055 operating modes.
 */
typedef enum {
    BNO055_OPERATING_MODE_CONFIGMODE   = 0x00,
    BNO055_OPERATING_MODE_ACCONLY      = 0x01,
    BNO055_OPERATING_MODE_MAGONLY      = 0x02,
    BNO055_OPERATING_MODE_GYROONLY     = 0x03,
    BNO055_OPERATING_MODE_ACCMAG       = 0x04,
    BNO055_OPERATING_MODE_ACCGYRO      = 0x05,
    BNO055_OPERATING_MODE_MAGGYRO      = 0x06,
    BNO055_OPERATING_MODE_AMG          = 0x07,
    BNO055_OPERATING_MODE_IMU          = 0x08,
    BNO055_OPERATING_MODE_COMPASS      = 0x09,
    BNO055_OPERATING_MODE_M4G          = 0x0A,
    BNO055_OPERATING_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPERATING_MODE_NDOF         = 0x0C
} bno055_operating_mode_t;

/**
 * @brief Quaternion data as reported by the BNO055.
 * 
 * @details Values are reported in fixed-point 2's complement format where the sign and integer part
 *          are the 2 MSBs, and the fractional part is the 14 LSBs.
 *          This is stated in table 3-31 of revision 1.4 of the BNO055 datasheet.
 * 
 * @note    As expected, I have observed these values to be unitary, although I believe this is not
 *          stated in the datasheet. The only exception is possibly when reading before the mode has
 *          been configured or finished starting up: I have obtained zeros when reading right after
 *          configuring the mode.
 */
typedef struct {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} bno055_quat_t;

typedef enum {
    BNO055_READY_EVT_TYPE_CANCELLED,
    BNO055_READY_EVT_TYPE_STATE_REACHED
} bno055_ready_evt_type_t;

typedef enum {
    /* The BNO055 did not enter sleep status because of a failure */
    BNO055_FAILURE_REASON_NONE,
    /* The BNO055 failed to send a TWI ACK (possibly disconnected) */
    BNO055_FAILURE_REASON_TWI_ACK,
    /* The BNO055 stretched the clock for too long */
    BNO055_FAILURE_REASON_TWI_TIMEOUT,
    /* A read value was inconsistent with the tracked state */
    BNO055_FAILURE_REASON_INCONSISTENCY
} bno055_failure_reason_t;

typedef enum {
    BNO055_DRIVER_STATUS_AWAKE,
    BNO055_DRIVER_STATUS_ASLEEP
} bno055_driver_status_t;

typedef enum {
    BNO055_EVT_TYPE_FAILURE,
    BNO055_EVT_TYPE_READY,
    BNO055_EVT_TYPE_QUAT_READ_DONE,
    BNO055_EVT_TYPE_REGISTER_READ_DONE
} bno055_evt_type_t;

typedef struct {
    bno055_evt_type_t type;
    union {
        bno055_ready_evt_type_t ready;
        bno055_quat_t           quat_read_done;
        struct {
            uint8_t *    p_data;
            uint8_t data_size;
        } register_read_done;
    } data;
} bno055_evt_t;

/** @brief Time it takes for BNO055 to be responsive after power on */
#define BNO055_POWER_ON_RESET_TIME_MS (650 + 50)

/**
 * @brief Transition times for operating modes.
 *
 * @details From table 3-6 of revision 1.4 of the BNO055 datasheet.
 */
#define BNO055_CONFIGMODE_TO_ANY_TIME_MS 7
#define BNO055_ANY_TO_CONFIGMODE_TIME_MS 19

/**
 * @brief I2C possible addresses for the BNO055.
 * 
 * @details I2C address is selected through the BNO055's COM3 pin.
 *          See table 4-7 of revision 1.4 of the BNO055 datasheet for more information.
 */
#define BNO055_ADDRESS_1 0x28
#define BNO055_ADDRESS_2 0x29

/** @brief Get the bno055 module status */
bno055_driver_status_t bno055_get_driver_status(void);

/** @brief Get the cause of the last failure that put the driver into sleep */
bno055_failure_reason_t bno055_get_failure_reason(void);

/**
 * @brief Determine whether state transitions are ready.
 * 
 * @retval true  The BNO055's power mode and operating mode have matched those configured with
 *               bno055_set_power_mode and bno055_set_operating_mode. 
 *         false The BNO055's state is still transitioning to the configured state.
 */
bool bno055_is_ready(void);

/**
 * @brief Read arbitrary registers.
 * 
 * @details Upon successful completion, a REGISTER_READ_DONE event is reported.
 * 
 * @param[in] addr   Start address of registers to read
 * @param[in] length Number of registers to read
 * 
 * @retval NRF_SUCCESS             Successfully started operation.
 * @retval NRF_ERROR_INVALID_STATE The BNO055 is not ready yet, or it is not in valid state for
 *                                 register reads (e.g. its power mode is set to suspend).
 * @retval NRF_ERROR_BUSY          There is another operation ongoing.
 */
uint32_t bno055_read_register(bno055_reg_addr_t addr, uint8_t length);

/**
 * @brief Read quaternion orientation data.
 * 
 * @details Upon successful completion, a QUAT_READ_DONE event is reported.
 * 
 * @retval NRF_SUCCESS             Successfully started operation.
 * @retval NRF_ERROR_INVALID_STATE The BNO055 is not ready yet, or it is not in a fusion mode.
 * @retval NRF_ERROR_BUSY          There is another operation ongoing.
 */
uint32_t bno055_read_quat(void);

/**
 * @brief Cancel requested read operation.
 * 
 * @details If successful, the driver will stop being in a ready state. The driver can become ready
 *          again once ongoing I2C operations finish. No <type>_READ_DONE events will be reported.
 * 
 * @retval NRF_SUCCESS             Successfully cancelled operation.
 * @retval NRF_ERROR_INVALID_STATE No ongoing operation.
 */ 
uint32_t bno055_cancel_read(void);

/** @brief Get configured power mode. */
bno055_power_mode_t bno055_get_power_mode(void);

/**
 * @brief Set power mode.
 * 
 * @details Once the remote BNO055 has finished being configured, a READY event is reported.
 * 
 * @retval NRF_SUCCESS             Successfully started transition.
 * @retval NRF_ERROR_INVALID_STATE One of the following:
 *                                 - The current operating mode is not CONFIGMODE, and therefore
 *                                   does not allow changing power mode.
 *                                 - The power mode is already set to power_mode.
 * @retval NRF_ERROR_BUSY          There is an ongoing read operation.
 */
uint32_t bno055_set_power_mode(bno055_power_mode_t power_mode);

/** @brief Get the configured operating mode */
bno055_operating_mode_t bno055_get_operating_mode(void);

/**
 * @brief Set operating mode.
 * 
 * @details Once the remote BNO055 has finished being configured, a READY event is reported.
 * 
 * @retval NRF_SUCCESS             Successfully started transition.
 * @retval NRF_ERROR_INVALID_STATE One of the following:
 *                                 - The current power mode is SUSPEND, which does not allow
 *                                   changing operating mode.
 *                                 - The power mode is already set to power_mode.
 * @retval NRF_ERROR_BUSY          There is an ongoing readoperation.
 */
uint32_t bno055_set_operating_mode(bno055_operating_mode_t operating_mode);

/**
 * @brief Set operating and power mode.
 * 
 * @details Once the remote BNO055 has finished being configured, a READY event is reported.
 * 
 * @retval NRF_SUCCESS             Successfully started transition.
 * @retval NRF_ERROR_INVALID_STATE The requested modes are already set.
 * @retval NRF_ERROR_INVALID_PARAM The requested mode combination is not valid.
 * @retval NRF_ERROR_BUSY          There is an ongoing read operation.
 */
uint32_t bno055_set_operating_and_power_modes(bno055_operating_mode_t operating_mode, bno055_power_mode_t power_mode);

/**
 * @brief Wake up the driver
 * 
 * @retval NRF_SUCCESS             Success, driver now awake.
 * @retval NRF_ERROR_INVALID_STATE Driver already awake. 
 */
uint32_t bno055_wakeup(void);

/**
 * @brief Initialize BNO055 module.
 * 
 * @param[in] config       Configuration for TWI peripheral, passed to nrf_drv_twi module.
 * @param[in] evt_callback Callback for all types of events.
 */
void bno055_init(nrf_drv_twi_config_t * p_config, void (* evt_callback)(bno055_evt_t *));

#endif // BNO055_H