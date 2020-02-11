
#include <stdatomic.h>
#include <stdint.h>

#include "bno055.h"

#include "nrf_assert.h"
#include "nrf_error.h"

#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_evt_queue.h"

/* The maximum time the BNO055 is allowed to stretch the clock */
#define BNO055_TWI_TIMEOUT_TIME_MS 500

/* Minimum time the I2C bus must remain idle between writes when on suspend mode low or power mode. 
 * Taken from table 4-8 of the datasheet */
#define BNO055_LONG_WRITE_IDLE_TIME_US 500

#define FULL_READ_SIZE (BNO055_REG_ADDR_QUA_DATA_Z_MSB - BNO055_REG_ADDR_ACC_DATA_X_LSB + 1)

typedef enum {
    BNO055_PAGE_0 = 0x00,
    BNO055_PAGE_1 = 0x01,
    BNO055_PAGE_INVALID
} bno055_page_t;

typedef enum {
    REGISTER_OPERATION_TYPE_READ,
    REGISTER_OPERATION_TYPE_WRITE
} register_operation_type_t;

typedef struct {
    register_operation_type_t type;
    uint8_t address;
    union {
        struct {
            uint8_t   rx_len;
            uint8_t * p_rx_buf;
        } read;
        uint8_t write;
    } data;
} register_operation_t;

static void (*bno055_evt_callback)(bno055_evt_t *);

static bno055_failure_reason_t failure_reason = BNO055_FAILURE_REASON_NONE;

/************************/
/* Forward declarations */
/************************/

static void on_register_operation_success(const register_operation_t *);
static void on_failure(bno055_failure_reason_t);
static void remote_state_update(void);

/********************/
/* Helper functions */
/********************/

/** @brief Convert uint8_t to int8_t in a portable way. */
static inline int8_t uint8_to_int8(uint8_t val)
{
    return val < 128 ? val : val - 256;
}

/**
 * @brief Returns whether a given mode is a fusion mode.
 * 
 * @details Refer to the section 3.3.3 of revision 1.4 of the BNO055 datasheet for a list of fusion
 *          modes.
 */
static bool is_fusion_mode(bno055_operating_mode_t operating_mode)
{
    bool b_is_fusion_mode;
    switch (operating_mode)
    {
        case BNO055_OPERATING_MODE_IMU:
        case BNO055_OPERATING_MODE_COMPASS:
        case BNO055_OPERATING_MODE_M4G:
        case BNO055_OPERATING_MODE_NDOF_FMC_OFF:
        case BNO055_OPERATING_MODE_NDOF:
            b_is_fusion_mode = true;
        break;
        default:
            b_is_fusion_mode = false;
        break;
    }
    return b_is_fusion_mode;
}

static uint32_t get_operating_mode_transition_time_ms(bno055_operating_mode_t start_mode,
    bno055_operating_mode_t end_mode)
{
    uint32_t transition_time_ms;

    if (start_mode == end_mode)
    {
        transition_time_ms = 0;
    }
    else if (BNO055_OPERATING_MODE_CONFIGMODE == start_mode
          && BNO055_OPERATING_MODE_CONFIGMODE != end_mode)
    {     
        transition_time_ms = BNO055_CONFIGMODE_TO_ANY_TIME_MS;
    }
    else if (BNO055_OPERATING_MODE_CONFIGMODE != start_mode
          && BNO055_OPERATING_MODE_CONFIGMODE == end_mode)
    {
        transition_time_ms = BNO055_ANY_TO_CONFIGMODE_TIME_MS;
    }
    else /* From not CONFIGMODE to not CONFIGMODE */
    {
        // TODO
        ASSERT(false);
    }

    return transition_time_ms;
}

static uint32_t get_power_mode_transition_time_ms(bno055_power_mode_t start_mode,
    bno055_power_mode_t end_mode)
{
    uint32_t transition_time_ms = 0;
    if (BNO055_POWER_MODE_SUSPEND == start_mode && BNO055_POWER_MODE_NORMAL == end_mode)
    {
        transition_time_ms = 750;
    }

    return transition_time_ms;
}

/**
 * @brief Whether an operation will alter the state of the IMU
 * 
 * Operating mode, power mode and register page are considered part of the IMU state.
 */
static bool alters_state(const register_operation_t * p_op)
{
    return REGISTER_OPERATION_TYPE_WRITE == p_op->type
        && (BNO055_REG_ADDR_OPR_MODE == p_op->address
        ||  BNO055_REG_ADDR_PWR_MODE == p_op->address
        ||  BNO055_REG_ADDR_PAGE_ID  == p_op->address);
}

/********************************************/
/** I2C to register write/reads abstraction */
/********************************************/

nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(0);

APP_TIMER_DEF(write_idle_time_done_timer);
static nrf_evt_queue_evt_t write_idle_time_done_evt;
static bool b_on_write_idle_time = false;

APP_TIMER_DEF(register_timeout_timer);
static nrf_evt_queue_evt_t register_timeout_evt; 

/* Buffers for TX and RX I2C data.
 * TODO: I am not sure whether the nrf_drv_twi requires them to be statically allocated or not, so
 *       I'm leaving it statically allocated for now. */
#define REGISTER_TX_BUFFER_SIZE 2 
#define REGISTER_RX_BUFFER_SIZE 32
static uint8_t register_tx_buffer[REGISTER_TX_BUFFER_SIZE];
static uint8_t register_rx_buffer[REGISTER_RX_BUFFER_SIZE];

/* Information on the last operation performed */
static register_operation_t register_last_op;

typedef enum {
    /* Available to perform a new operation */
    REGISTER_OPERATION_STATE_AVAILABLE,
    /* Waiting for the required idle time to pass (specified on table 4-8 of the datasheet) */
    REGISTER_OPERATION_STATE_IDLE_TIME,
    /* Ongoing TWI operation */
    REGISTER_OPERATION_STATE_ONGOING
} register_operation_state_t;

/* Whether there is currently an ongoing read or write operation. */
static bool register_operation_ongoing = false;

static nrf_evt_queue_evt_t register_operation_done_evt;
/* For passing TWI event information from the interrupt handler to main context */
static nrf_drv_twi_evt_t   register_operation_twi_evt_result;

static void write_idle_time_done_evt_callback(void)
{
    b_on_write_idle_time = false;
    remote_state_update();
}

static void write_idle_time_done_handler(void * p_context)
{
    uint32_t err_code = nrf_evt_queue_put(&write_idle_time_done_evt, write_idle_time_done_evt_callback);
    APP_ERROR_CHECK(err_code);
}

static void register_timeout_evt_handler(void)
{
    bool b_timed_out;
    
    CRITICAL_REGION_ENTER();
    /* Because the TWI interrupt cancels the timeout timer and event, if the TWI interrupt actually
     * happened, it must have queued its event *after* the timeout event.
     * This is a situation where it matters that event queues are ordered */
    b_timed_out = !nrf_evt_queue_is_queued(&register_operation_done_evt);
    if (b_timed_out) nrf_drv_twi_disable(&twi_instance);
    CRITICAL_REGION_EXIT();

    if (b_timed_out)
    {
        on_failure(BNO055_FAILURE_REASON_TWI_TIMEOUT);
    }
}

static void register_timeout_handler(void * p_context)
{
    uint32_t err_code = nrf_evt_queue_put(&register_timeout_evt, register_timeout_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void register_twi_evt_callback(void)
{
    register_operation_ongoing = false;

    switch (register_operation_twi_evt_result.type)
    {
        /* On success, update the remote state */
        case NRF_DRV_TWI_EVT_DONE:
        {
            register_operation_t operation;
            const nrf_drv_twi_xfer_desc_t * const p_xfer_desc = &register_operation_twi_evt_result.xfer_desc;
            switch (register_operation_twi_evt_result.xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX: /* Register write */
                    ASSERT(NULL != p_xfer_desc->p_primary_buf && 2 == p_xfer_desc->primary_length);
                    operation.type = REGISTER_OPERATION_TYPE_WRITE;
                    operation.address = p_xfer_desc->p_primary_buf[0];
                    operation.data.write = p_xfer_desc->p_primary_buf[1];

                    /* Indiscriminately wait the 500 us before allowing the next write.
                     * We do this because, even though the datasheet says we only need to wait 2 us
                     * when on normal mode, it doesn't specify which time applies when moving from
                     * suspend mode to normal mode or vice-versa.
                     * It's just easier to do a blanket 500 us and not worry about figuring out a
                     * low quality datasheet.
                     */
                    b_on_write_idle_time = true;
                    uint32_t write_idle_time_done_ticks = (uint32_t) CEIL_DIV(BNO055_LONG_WRITE_IDLE_TIME_US
                        * (uint64_t) APP_TIMER_CLOCK_FREQ, (APP_TIMER_PRESCALER + 1) * (uint64_t) 1000000);
                    uint32_t err_code = app_timer_start(write_idle_time_done_timer, write_idle_time_done_ticks, NULL);
                    APP_ERROR_CHECK(err_code);
                break;
                case NRF_DRV_TWI_XFER_TXRX: /* Register read */
                    ASSERT(NULL != p_xfer_desc->p_primary_buf && 1 == p_xfer_desc->primary_length);
                    ASSERT(NULL != p_xfer_desc->p_secondary_buf);
                    operation.type = REGISTER_OPERATION_TYPE_READ;
                    operation.address = p_xfer_desc->p_primary_buf[0];
                    operation.data.read.p_rx_buf = p_xfer_desc->p_secondary_buf;
                    operation.data.read.rx_len = p_xfer_desc->secondary_length;
                break;
                default:
                    ASSERT(false);
                break;
            }
            on_register_operation_success(&operation);
        } break;
        /* On failure, disable TWI and report the failure */
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        case NRF_DRV_TWI_EVT_DATA_NACK:
            nrf_drv_twi_disable(&twi_instance);
            on_failure(BNO055_FAILURE_REASON_TWI_ACK);
        break;
        default:
            ASSERT(false);
        break;
    }
}

/**
 * @brief Handler for TWI events reported by the nrf_drv_twi module.
 * 
 * @warning Called from inside a TWI interrupt.
 */
static void register_twi_handler(const nrf_drv_twi_evt_t * p_evt, void * p_context)
{
    (void) app_timer_stop(register_timeout_timer);
    (void) nrf_evt_queue_remove(&register_timeout_evt);

    register_operation_twi_evt_result = *p_evt;
    uint32_t err_code = nrf_evt_queue_put(&register_operation_done_evt, register_twi_evt_callback);
    // TODO: APP_ERROR_CHECKs won't work with the current interrupt priority, the
    // app_error_fault_handler needs to be fixed. We'll leave it anyway.
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief   Read BNO055 register
 * 
 * @details After the read is complete, the callback is called in main context using nrf_evt_queue.
 * 
 * @retval  NRF_SUCCESS    Successfully started read
 * @retval  NRF_ERROR_BUSY There is a read/write operation already ongoing
 */
static uint32_t register_read(bno055_reg_addr_t reg_address, uint8_t length)
{
    uint32_t ret_code;
    if (register_operation_ongoing)
    {
        ret_code = NRF_ERROR_BUSY;
    }
    else
    {
        ASSERT(length <= REGISTER_RX_BUFFER_SIZE);

        register_last_op.type = REGISTER_OPERATION_TYPE_READ;
        register_last_op.address = reg_address;
        register_last_op.data.read.p_rx_buf = register_rx_buffer;
        register_last_op.data.read.rx_len = length;

        nrf_drv_twi_xfer_desc_t xfer_desc = {
            .type = NRF_DRV_TWI_XFER_TXRX,
            .address = BNO055_ADDRESS_1,
            .primary_length = 1,
            .secondary_length = length,
            .p_primary_buf = register_tx_buffer,
            .p_secondary_buf = register_rx_buffer
        };

        register_tx_buffer[0] = reg_address;

        uint32_t err_code = nrf_drv_twi_xfer(&twi_instance, &xfer_desc, 0);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(register_timeout_timer, APP_TIMER_TICKS(BNO055_TWI_TIMEOUT_TIME_MS,
            APP_TIMER_PRESCALER), NULL);
        APP_ERROR_CHECK(err_code);

        register_operation_ongoing = true;

        ret_code = NRF_SUCCESS;
    }
    return ret_code;
}

/**
 * @brief   Read from BNO055 register.
 * 
 * @details After the write is complete, the callback is called in main context using nrf_evt_queue.
 * 
 * @retval  NRF_SUCCESS    Successfully started write
 * @retval  NRF_ERROR_BUSY There is a read/write operation already ongoing
 */
static uint32_t register_write(bno055_reg_addr_t reg_address, uint8_t value)
{
    uint32_t ret_code;
    if (register_operation_ongoing || b_on_write_idle_time)
    {
        ret_code = NRF_ERROR_BUSY;
    }
    else
    {
        register_last_op.type = REGISTER_OPERATION_TYPE_WRITE;
        register_last_op.address = reg_address;
        register_last_op.data.write = value;

        /* Writing is a single operation (no repeated start) */
        /* Refer to the BNO055 datasheet */
        nrf_drv_twi_xfer_desc_t xfer_desc = {
            .type = NRF_DRV_TWI_XFER_TX,
            .address = BNO055_ADDRESS_1,
            .primary_length = 2,
            .secondary_length = 0,
            .p_primary_buf = register_tx_buffer,
            .p_secondary_buf = NULL
        };

        register_tx_buffer[0] = reg_address;
        register_tx_buffer[1] = value;
        
        uint32_t err_code;
        err_code = nrf_drv_twi_xfer(&twi_instance, &xfer_desc, 0);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(register_timeout_timer, APP_TIMER_TICKS(BNO055_TWI_TIMEOUT_TIME_MS,
            APP_TIMER_PRESCALER), NULL);
        APP_ERROR_CHECK(err_code);

        register_operation_ongoing = true;

        ret_code = NRF_SUCCESS;
    }
    return ret_code;
}

/*********************************/
/** BNO055 remote state tracking */
/*********************************/

static bno055_driver_status_t remote_state_status = BNO055_DRIVER_STATUS_ASLEEP;

static bno055_page_t remote_state_page = BNO055_PAGE_INVALID;
static bool b_chip_id_verified = false;

APP_TIMER_DEF(remote_state_transition_timer);
static nrf_evt_queue_evt_t remote_state_transition_evt;
static bool b_remote_state_awaiting_transition = false;

/** @brief The actual current power mode actually set on the BNO055 */
static bno055_power_mode_t current_power_mode;
/** @brief Whether the current power mode is known and the corresponding variable is valid */
static bool b_current_power_mode_valid = false;
/** @brief The actual operating mode currently set on the BNO055 */
static bno055_operating_mode_t current_operating_mode;
/** @brief Whether the current operating mode is known and the corresponding variable is valid */
static bool b_current_operating_mode_valid = false;

/** @brief The desired power mode */
static bno055_power_mode_t target_power_mode = BNO055_POWER_MODE_SUSPEND; 
/** @brief The desired operating mode */
static bno055_operating_mode_t target_operating_mode = BNO055_OPERATING_MODE_CONFIGMODE;

bno055_driver_status_t bno055_get_driver_status(void)
{
    return remote_state_status;
}

static bool remote_state_is_ready(void)
{
    return  BNO055_DRIVER_STATUS_AWAKE == remote_state_status
            && !(register_operation_ongoing && alters_state(&register_last_op))
            && !b_remote_state_awaiting_transition
            && b_chip_id_verified
            && BNO055_PAGE_0 == remote_state_page
            && b_current_operating_mode_valid
            && b_current_power_mode_valid
            && current_operating_mode == target_operating_mode
            &&     current_power_mode == target_power_mode;
}

/**
 * @brief Update the power and operating mode to try reaching configured target state.
 * 
 * Should only be called once the state of the IMU.
 */
static void remote_state_update_modes(void)
{
    if (BNO055_OPERATING_MODE_CONFIGMODE == current_operating_mode
        && target_power_mode != current_power_mode)
    {
        (void) register_write(BNO055_REG_ADDR_PWR_MODE, target_power_mode);
    }
    else if (BNO055_POWER_MODE_NORMAL == current_power_mode
        && target_operating_mode != current_operating_mode)
    {
        (void) register_write(BNO055_REG_ADDR_OPR_MODE, target_operating_mode);
    }
    else
    {
        ASSERT(target_power_mode == current_power_mode);
        ASSERT(target_operating_mode == current_operating_mode);
        /* Do nothing */
    }
}

/** @brief Perform next necessary action to reach the target state. */
static void remote_state_update(void)
{
    if (BNO055_DRIVER_STATUS_AWAKE == remote_state_status
        && !b_remote_state_awaiting_transition) switch (remote_state_page)
    {
        case BNO055_PAGE_INVALID:
        {
            (void) register_read(BNO055_REG_ADDR_PAGE_ID, 1);
        } break;
        case BNO055_PAGE_1:
        {
            (void) register_write(BNO055_REG_ADDR_PAGE_ID, BNO055_PAGE_0);
        } break;
        case BNO055_PAGE_0:
            if (!b_chip_id_verified)
            {
                (void) register_read(BNO055_REG_ADDR_CHIP_ID, 1);
            }
            else if (!b_current_operating_mode_valid)
            {
                (void) register_read(BNO055_REG_ADDR_OPR_MODE, 1);
            }
            else if (!b_current_power_mode_valid)
            {
                (void) register_read(BNO055_REG_ADDR_PWR_MODE, 1);
            }
            else /* We have all information on remote states */
            {
                remote_state_update_modes();
            }
        break;
        default:
            ASSERT(false);
        break; 
    }
}

static void remote_state_transition_done(void)
{
    ASSERT(b_remote_state_awaiting_transition);
    b_remote_state_awaiting_transition = false;
    if (remote_state_is_ready())
    {
        bno055_evt_t evt;
        evt.type = BNO055_EVT_TYPE_READY;
        evt.data.ready = BNO055_READY_EVT_TYPE_STATE_REACHED;
        bno055_evt_callback(&evt);
    }
    else
    {
        remote_state_update();
    }
}

static void remote_state_transition_timer_handler(void * p_context)
{
    uint32_t err_code = nrf_evt_queue_put(&remote_state_transition_evt, remote_state_transition_done);
    APP_ERROR_CHECK(err_code);
}

static void remote_state_on_register_read_success(uint8_t address, uint8_t value)
{
    /* Whether the read value is consistent with out tracked state */
    bool b_is_inconsistent = false;

    /* TODO: Check that page IDs and power modes are valid values */
    if (BNO055_REG_ADDR_PAGE_ID == address)
    {
        if (BNO055_PAGE_INVALID != remote_state_page && value != remote_state_page) b_is_inconsistent = true;
        else remote_state_page = value;
    }
    else if (0x00 == remote_state_page) switch (address)
    {
        case BNO055_REG_ADDR_OPR_MODE:
            if (b_current_operating_mode_valid && value != current_operating_mode) b_is_inconsistent = true;
            else
            {
                b_current_operating_mode_valid = true;
                current_operating_mode = value;
            }
        break;
        case BNO055_REG_ADDR_PWR_MODE:
            if (b_current_power_mode_valid && value != current_power_mode) b_is_inconsistent = true;
            else
            {
                b_current_power_mode_valid = true;
                current_power_mode = value;
            }
        break;
        case BNO055_REG_ADDR_CHIP_ID:
            if (value != BNO055_CHIP_ID) b_is_inconsistent = true;
            else b_chip_id_verified = true;
        break;
        default:
            /* Register is not relevant to remote state tracking */
        break;
    }

    /* If we are supposed to know a given value, and it turns out to be wrong, we panic into
     * an unrecoverable failure.
     * A possible cause for this might be a reset of the BNO055 */
   if (b_is_inconsistent)
   {
        nrf_drv_twi_disable(&twi_instance);
        on_failure(BNO055_FAILURE_REASON_INCONSISTENCY);
   }
}

static void remote_state_on_write_success(uint8_t address, uint8_t value)
{
    ASSERT(!b_remote_state_awaiting_transition);

    uint32_t transition_time_ms = 0;
    if (BNO055_REG_ADDR_PAGE_ID == address)
    {
        remote_state_page = value;
    }
    else if (0x00 == remote_state_page) switch (address)
    {
        /* TODO: Check that the value variable is a valid value for the enums */
        case BNO055_REG_ADDR_OPR_MODE:
        {
            bno055_operating_mode_t new_operating_mode = value;
            transition_time_ms = get_operating_mode_transition_time_ms(current_operating_mode,
                new_operating_mode);
            current_operating_mode = new_operating_mode;
        } break;
        case BNO055_REG_ADDR_PWR_MODE:
        {
            bno055_power_mode_t new_power_mode = value;
            transition_time_ms = get_power_mode_transition_time_ms(current_power_mode, new_power_mode);
            current_power_mode = value;
        } break;
        default:
            /* The message isn't relevant to remote state tracking */
        break;
    }

    if (0 < transition_time_ms)
    {
        b_remote_state_awaiting_transition = true;
        uint32_t err_code = app_timer_start(remote_state_transition_timer,
            APP_TIMER_TICKS(transition_time_ms, APP_TIMER_PRESCALER), NULL);
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @brief Callback for *any* TWI operation.
 * 
 * Called after any TWI operation successfully completes, even those that were not started by
 * remote_state.
 */
static void remote_state_on_operation_success(const register_operation_t * p_op)
{
    ASSERT(BNO055_DRIVER_STATUS_AWAKE == remote_state_status);
    switch (p_op->type)
    {
        case REGISTER_OPERATION_TYPE_WRITE: /* Register write */
            remote_state_on_write_success(p_op->address, p_op->data.write);
        break;
        case REGISTER_OPERATION_TYPE_READ: /* Register read */
            /* Remember that contiguous registers may be read in a single read operation */
            for (int reg_index = 0; reg_index < p_op->data.read.rx_len; ++reg_index)
            {
                remote_state_on_register_read_success(p_op->address + reg_index,
                    p_op->data.read.p_rx_buf[reg_index]);
            }
        break;
        default:
            ASSERT(false);
        break;
    }

    remote_state_update();
}

uint32_t bno055_wakeup(void)
{
    uint32_t ret;
    switch (remote_state_status)
    {
        case BNO055_DRIVER_STATUS_ASLEEP:
            remote_state_page = BNO055_PAGE_INVALID;
            b_chip_id_verified = false;
            b_current_operating_mode_valid = false;
            b_current_power_mode_valid = false;

            remote_state_status = BNO055_DRIVER_STATUS_AWAKE;

            nrf_drv_twi_enable(&twi_instance);
            remote_state_update();
            ret = NRF_SUCCESS;
        break;
        case BNO055_DRIVER_STATUS_AWAKE:
            ret = NRF_ERROR_INVALID_STATE;
        break;
        default:
            ASSERT(false);
        break;
    }
    return ret;
}

/*********************/
/* Direct operations */
/*********************/

typedef enum {
    SIMPLE_OPERATION_STATE_ONGOING,
    SIMPLE_OPERATION_STATE_CANCELLED,
    SIMPLE_OPERATION_STATE_INACTIVE
} simple_operation_state_t;

static simple_operation_state_t simple_operation_state = SIMPLE_OPERATION_STATE_INACTIVE;
static void (*simple_operation_complete_callback)(const register_operation_t *);

static void simple_full_read_complete(const register_operation_t * p_op)
{
    ASSERT(REGISTER_OPERATION_TYPE_READ == p_op->type);
    ASSERT(FULL_READ_SIZE == p_op->data.read.rx_len);
    ASSERT(NULL != p_op->data.read.p_rx_buf);

    bno055_evt_t evt;
    evt.type = BNO055_EVT_TYPE_FULL_READ_DONE;
    evt.data.full_read_done.acc.x = 256 * uint8_to_int8(register_rx_buffer[1]) + register_rx_buffer[0];
    evt.data.full_read_done.acc.y = 256 * uint8_to_int8(register_rx_buffer[3]) + register_rx_buffer[2];
    evt.data.full_read_done.acc.z = 256 * uint8_to_int8(register_rx_buffer[5]) + register_rx_buffer[4];
    evt.data.full_read_done.mag.x = 256 * uint8_to_int8(register_rx_buffer[7]) + register_rx_buffer[6];
    evt.data.full_read_done.mag.y = 256 * uint8_to_int8(register_rx_buffer[9]) + register_rx_buffer[8];
    evt.data.full_read_done.mag.z = 256 * uint8_to_int8(register_rx_buffer[11]) + register_rx_buffer[10];
    evt.data.full_read_done.gyr.x = 256 * uint8_to_int8(register_rx_buffer[13]) + register_rx_buffer[12];
    evt.data.full_read_done.gyr.y = 256 * uint8_to_int8(register_rx_buffer[15]) + register_rx_buffer[14];
    evt.data.full_read_done.gyr.z = 256 * uint8_to_int8(register_rx_buffer[17]) + register_rx_buffer[16];
    evt.data.full_read_done.qua.w = 256 * uint8_to_int8(register_rx_buffer[19]) + register_rx_buffer[18];
    evt.data.full_read_done.qua.x = 256 * uint8_to_int8(register_rx_buffer[21]) + register_rx_buffer[20];
    evt.data.full_read_done.qua.y = 256 * uint8_to_int8(register_rx_buffer[23]) + register_rx_buffer[22];
    evt.data.full_read_done.qua.z = 256 * uint8_to_int8(register_rx_buffer[25]) + register_rx_buffer[24];
    bno055_evt_callback(&evt);
}

/* @brief Callback for when quat_read operation has completed. */
static void simple_quat_read_complete(const register_operation_t * p_op)
{
    ASSERT(REGISTER_OPERATION_TYPE_READ == p_op->type);
    ASSERT(8 == p_op->data.read.rx_len);
    ASSERT(NULL != p_op->data.read.p_rx_buf);

    bno055_evt_t evt;
    evt.type = BNO055_EVT_TYPE_QUAT_READ_DONE;
    evt.data.quat_read_done.w = 256 * uint8_to_int8(register_rx_buffer[1]) + register_rx_buffer[0];
    evt.data.quat_read_done.x = 256 * uint8_to_int8(register_rx_buffer[3]) + register_rx_buffer[2];
    evt.data.quat_read_done.y = 256 * uint8_to_int8(register_rx_buffer[5]) + register_rx_buffer[4];
    evt.data.quat_read_done.z = 256 * uint8_to_int8(register_rx_buffer[7]) + register_rx_buffer[6];
    bno055_evt_callback(&evt);
}

uint32_t bno055_full_read(void)
{
    uint32_t ret_code;
    if (remote_state_is_ready() && (current_operating_mode == BNO055_OPERATING_MODE_NDOF
        || current_operating_mode == BNO055_OPERATING_MODE_NDOF_FMC_OFF))
    {
        ret_code = register_read(BNO055_REG_ADDR_ACC_DATA_X_LSB, FULL_READ_SIZE);
    }
    else
    {
        ret_code = NRF_ERROR_INVALID_STATE;
    }

    if (NRF_SUCCESS == ret_code)
    {
        simple_operation_complete_callback = simple_full_read_complete;
        simple_operation_state = SIMPLE_OPERATION_STATE_ONGOING;
    }

    return ret_code;
}

uint32_t bno055_read_quat(void)
{
    uint32_t ret_code;
    if (remote_state_is_ready() && is_fusion_mode(current_operating_mode))
    {
        ret_code = register_read(BNO055_REG_ADDR_QUA_DATA_W_LSB, 8);
    }
    else
    {
        ret_code = NRF_ERROR_INVALID_STATE;
    }

    if (NRF_SUCCESS == ret_code)
    {
        simple_operation_complete_callback = simple_quat_read_complete;
        simple_operation_state = SIMPLE_OPERATION_STATE_ONGOING;
    }

    return ret_code;
}

static void simple_register_read_complete(const register_operation_t * p_op)
{
    bno055_evt_t evt;
    evt.type = BNO055_EVT_TYPE_REGISTER_READ_DONE;
    evt.data.register_read_done.p_data = p_op->data.read.p_rx_buf;
    evt.data.register_read_done.data_size = p_op->data.read.rx_len;
    bno055_evt_callback(&evt);
}

uint32_t bno055_read_register(bno055_reg_addr_t addr, uint8_t length)
{
    uint32_t ret_code;

    if (remote_state_is_ready() && BNO055_POWER_MODE_SUSPEND != target_power_mode)
    {
        ret_code = register_read(addr, length);
    }
    else
    {
        ret_code = NRF_ERROR_INVALID_STATE;
    }

    if (NRF_SUCCESS == ret_code)
    {
        simple_operation_complete_callback = simple_register_read_complete;
        simple_operation_state = SIMPLE_OPERATION_STATE_ONGOING;
    }
    return ret_code;
}

static void simple_on_operation_success(const register_operation_t * p_op)
{
    switch (simple_operation_state)
    {
        case SIMPLE_OPERATION_STATE_ONGOING:
            simple_operation_state = SIMPLE_OPERATION_STATE_INACTIVE;
            ASSERT(NULL != simple_operation_complete_callback);
            simple_operation_complete_callback(p_op);
        break;
        case SIMPLE_OPERATION_STATE_CANCELLED:
            simple_operation_state = SIMPLE_OPERATION_STATE_INACTIVE;
        break;
        case SIMPLE_OPERATION_STATE_INACTIVE:
            /* Not the result of a simple operation */
        break;
        default:
            ASSERT(false);
        break;
    }
}

uint32_t bno055_cancel_read(void)
{
    uint32_t ret_code = NRF_ERROR_INVALID_STATE;
    if (SIMPLE_OPERATION_STATE_ONGOING == simple_operation_state)
    {
        simple_operation_state = SIMPLE_OPERATION_STATE_CANCELLED;
        ret_code = NRF_SUCCESS;
    }
    return ret_code;
}

/**********/
/* Common */
/**********/

static void on_register_operation_success(const register_operation_t * p_op)
{
    bool b_was_a_cancelled_op = SIMPLE_OPERATION_STATE_CANCELLED == simple_operation_state;
    bool b_was_ready = remote_state_is_ready() && !alters_state(p_op);

    /* Remote state needs to be updated *before* any callbacks are called through
     * simple_on_operation_success */
    remote_state_on_operation_success(p_op);

    /* Only report simple operation successes if remote state was "ready" *before* the operation in
     * question (i.e. we need to check for readiness before updating the state) */
    if (b_was_ready) simple_on_operation_success(p_op);
    /* If the simple operation itself broke the ready state, we want to make sure that we are
     * not left expecting the completion for that operation. */
    else simple_operation_state = SIMPLE_OPERATION_STATE_INACTIVE;

    /* If the operation was both manual and a valid remote state operation, the ready event type
     * STATE_REACHED takes precedence (even though manual operations might break the readiness of
     * remote state, they may not if they wrote the same mode that was configured or it was
     * followed by a call to remote_state). */

    /* Report ready event if we were not ready before, but are ready now */
    if (!b_was_ready && remote_state_is_ready())
    {
        bno055_evt_t evt;
        evt.type = BNO055_EVT_TYPE_READY;
        evt.data.ready = BNO055_READY_EVT_TYPE_STATE_REACHED;
        bno055_evt_callback(&evt);
    }
    else if (b_was_a_cancelled_op)
    {
        bno055_evt_t evt;
        evt.type = BNO055_EVT_TYPE_READY;
        evt.data.ready = BNO055_READY_EVT_TYPE_CANCELLED;
        bno055_evt_callback(&evt);
    }
}

static void on_failure(bno055_failure_reason_t reason)
{
    failure_reason = reason;

    simple_operation_state = SIMPLE_OPERATION_STATE_INACTIVE;

    remote_state_page = BNO055_PAGE_INVALID;
    b_chip_id_verified = false;
    b_current_operating_mode_valid = false;
    b_current_power_mode_valid = false;

    remote_state_status = BNO055_DRIVER_STATUS_ASLEEP;

    bno055_evt_t evt;
    evt.type = BNO055_EVT_TYPE_FAILURE;
    bno055_evt_callback(&evt);
}

bno055_failure_reason_t bno055_get_failure_reason(void)
{
    return failure_reason;
}

bool bno055_is_ready(void)
{
    return remote_state_is_ready() && SIMPLE_OPERATION_STATE_CANCELLED != simple_operation_state;
}

bno055_power_mode_t bno055_get_power_mode(void)
{
    return target_power_mode;
}

bno055_operating_mode_t bno055_get_operating_mode(void)
{
    return target_operating_mode;
}

uint32_t bno055_set_power_mode(bno055_power_mode_t power_mode)
{
    uint32_t ret_code;
    if (SIMPLE_OPERATION_STATE_ONGOING == simple_operation_state)
    {
        ret_code = NRF_ERROR_BUSY;
    }
    /* Power mode changes are only allowed in when there's no failure, in CONFIG operating mode. */
    else if (target_power_mode != power_mode
           && BNO055_OPERATING_MODE_CONFIGMODE == target_operating_mode)
    {
        target_power_mode = power_mode;
        remote_state_update();
        ret_code = NRF_SUCCESS;
    }
    else
    {
        ret_code = NRF_ERROR_INVALID_STATE;
    }
    return ret_code;
}

uint32_t bno055_set_operating_mode(bno055_operating_mode_t operating_mode)
{
    uint32_t ret_code;
    if (SIMPLE_OPERATION_STATE_ONGOING == simple_operation_state)
    {
        ret_code = NRF_ERROR_BUSY;
    }
    else if (target_operating_mode != operating_mode
           && BNO055_POWER_MODE_NORMAL == target_power_mode)
    {
        target_operating_mode = operating_mode;
        remote_state_update();
        ret_code = NRF_SUCCESS;
    }
    else
    {
        ret_code = NRF_ERROR_INVALID_STATE;
    }
    return ret_code;
}

uint32_t bno055_set_operating_and_power_modes(bno055_operating_mode_t operating_mode, bno055_power_mode_t power_mode)
{
    uint32_t ret_code;
    if (SIMPLE_OPERATION_STATE_ONGOING == simple_operation_state)
    {
        ret_code = NRF_ERROR_BUSY;
    }
    else if (target_operating_mode == operating_mode && target_power_mode == power_mode)
    {
        ret_code = NRF_ERROR_INVALID_STATE;
    }
    else if (BNO055_OPERATING_MODE_CONFIGMODE != operating_mode && BNO055_POWER_MODE_NORMAL != power_mode)
    {
        ret_code = NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        target_operating_mode = operating_mode;
        target_power_mode = power_mode;
        remote_state_update();
        ret_code = NRF_SUCCESS;
    }
    return ret_code;
}

void bno055_init(nrf_drv_twi_config_t * p_config, void (* evt_callback)(bno055_evt_t *))
{
    ASSERT(NULL != evt_callback);
    bno055_evt_callback = evt_callback;
    uint32_t err_code;
    err_code = nrf_drv_twi_init(&twi_instance, p_config, register_twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&register_timeout_timer, APP_TIMER_MODE_SINGLE_SHOT, register_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&remote_state_transition_timer, APP_TIMER_MODE_SINGLE_SHOT,
        remote_state_transition_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&write_idle_time_done_timer, APP_TIMER_MODE_SINGLE_SHOT, write_idle_time_done_handler);
    APP_ERROR_CHECK(err_code);
}