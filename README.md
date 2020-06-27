# BNO055 Driver

An asynchronous driver made for the BNO055 built using Nordic Semiconductor's nRF5 SDK v12.3.0.

It's in a functional state but it is missing some features such as low power mode and manually writing registers, because I have not needed them so far.

# Features

- **Asynchronous**: callbacks done through [nrf-evt-queue](https://github.com/tlongeri/nrf-evt-queue) events.
- **Automatic mode configuration**: you can just tell the driver what operating mode and what power mode you want and forget about it. You don't need to wait for the changes to complete before setting new modes. The driver will let you know when it's ready.
- **Failure handling**: can handle and recover from I2C failures and unexpected/invalid BNO055 responses - usually this would involve physically fixing loose wires before telling software to try again.
- **Initial state flexibility**: you don't need to start the driver with a freshly resetted BNO055 for it to work: it will start by reading the current state and move on from there.

# Example usage

```c
static void bno055_evt_handler(bno055_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BNO055_EVT_TYPE_FAILURE: {
            switch (bno055_get_failure_reason())
            {
                case BNO055_FAILURE_REASON_TWI_ACK:
                    /* No ACK received. Probably disconnected wires */
                break;
                case BNO055_FAILURE_REASON_TWI_TIMEOUT:
                    /* The clock was stretched for too long. */
                    /* Maybe it's shorted to ground? */
                break;
                case BNO055_FAILURE_REASON_INCONSISTENCY:
                    /* Unexpected value read */
                break;
            }
            /* Try starting again */
            /* Maybe give 5 seconds to let the user fix wires */
            uint32_t err_code = bno055_wakeup();
            APP_ERROR_CHECK(err_code);
        } break;
        case BNO055_EVT_TYPE_READY: {
            /* We've reached the desired operating and power mode and
             * are ready to do things like read quaternions */
            /* Let's read a single quat */
            uint32_t err_code = bno055_read_quat();
            APP_ERROR_CHECK(err_code);
        } break;
        case BNO055_EVT_TYPE_QUAT_READ_DONE: {
            /* Quat read done */
            /* p_evt->data.quat_read contains the read quaternion in
             * fixed point representation with 14 bit fractional
             * part */
        } break;
    }
}

int main(void)
{
    /* Pass configuration for Nordic's nrf_drv_twi */
    nrf_drv_twi_config_t config = {
        .scl = SCL_PIN_NUMBER,
        .sda = SDA_PIN_NUMBER,
        .frequency = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        /* clear_bus_init should be set to true to allow recovering
         * from a stuck bus */
        .clear_bus_init = true,
        .hold_bus_uninit = true
    };
    bno055_init(&config, bno055_evt_handler);

    uint32_t err_code = bno055_wakeup();
    APP_ERROR_CHECK(err_code);

    bno055_set_operating_and_power_modes(BNO055_OPERATING_MODE_NDOF, BNO055_POWER_MODE_NORMAL);
    
    /* Wait for the ready event */
    while (true)
    {
        nrf_evt_queue_execute();
        sd_app_evt_wait();
    }
}
```