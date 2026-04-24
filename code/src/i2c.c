#include "i2c.h"

static bool i2c_slave_request_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg)
{
    i2c_slave_context_t *context = (i2c_slave_context_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_REQUEST;
    BaseType_t xTaskWoken = 0;
    xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}

static bool i2c_slave_receive_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg)
{
    i2c_slave_context_t *context = (i2c_slave_context_t *)arg;

    if (evt_data->length > sizeof(context->buffer)) {
        return 0;
    }

    context->command = evt_data->buffer[0];
    context->length = evt_data->length;
    memcpy(context->buffer, evt_data->buffer, evt_data->length);

    BaseType_t xTaskWoken = 0;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_RECEIVE;
    xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}

i2c_slave_context_t init_i2c_slave_context()
{
    static i2c_slave_context_t context = {0};
 
    context.event_queue = xQueueCreate(16, sizeof(i2c_slave_event_t));
    if (!context.event_queue) {
        printf("Creating queue failed\n");
        return context;
    }

    i2c_slave_config_t i2c_slv_config = {
        .i2c_port = I2C_SLAVE_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .slave_addr = ESP_SLAVE_ADDR,
        .send_buf_depth = 100,
        .receive_buf_depth = 100,
    };
    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &context.handle));
    
    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_slave_receive_cb,
        .on_request = i2c_slave_request_cb,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(context.handle, &cbs, &context));

    return context;
}