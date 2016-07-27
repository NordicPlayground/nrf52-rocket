#include "strato_ignition.h"
#include "pca20027.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "strato_app_config.h"

#define SAMPLES_IN_BUFFER 5

static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;
static uint32_t                m_sample_period;
static float                   m_current_voltage;
static ignition_adc_evt_handler_t  m_adc_cb;


void timer_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        SEGGER_RTT_printf(0, "ADC event number: %d\r\n",(int)m_adc_evt_counter);
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            SEGGER_RTT_printf(0, "%d\r\n", p_event->data.done.p_buffer[i]);
        }
        m_adc_evt_counter++;
    }
    m_adc_cb(5.0f);
    // TODO: this is just a place holder
}

void ignition_init(ignition_init_t * p_params)
{
    m_sample_period = p_params->adc_sampling_period_ms;
    m_adc_cb = p_params->adc_evt_handler;
    //Pin configs
    nrf_gpio_cfg_output(BOOST_5V_ENABLE);
    nrf_gpio_cfg_output(IGNITION_CH1);
    nrf_gpio_cfg_output(IGNITION_CH2);
    nrf_gpio_cfg_output(SC_DUMP);
    nrf_gpio_cfg_input(POWER_GOOD, NRF_GPIO_PIN_NOPULL);

    //Enable 5V Boost Converter
    nrf_gpio_pin_set(BOOST_5V_ENABLE);

    SEGGER_RTT_printf(0, "Power Good");
    while(nrf_gpio_pin_read(POWER_GOOD) == 1)
    {
        SEGGER_RTT_printf(0, "...");
    }
    SEGGER_RTT_printf(0, "\r\n");


    //Set up ADC Sampling of Supercap voltage
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    if (err_code != MODULE_ALREADY_INITIALIZED)
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }


    /* setup m_timer for compare event every 500ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, m_sample_period);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(nrf_drv_saadc_gpio_to_ain(VSC_SENSE));
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

ret_code_t ignition_cap_adc_sample_begin(void)
{
    return nrf_drv_ppi_channel_enable(m_ppi_channel);
}

ret_code_t ignition_cap_adc_sample_end(void)
{
    return nrf_drv_ppi_channel_disable(m_ppi_channel);
}


void ignition_dump_cap(bool state)
{
    if (state)
    {
        nrf_gpio_pin_set(SC_DUMP);
    }
    else
    {
        nrf_gpio_pin_clear(SC_DUMP);
    }
}

ret_code_t ignition_trigger_on(uint8_t channel)
{
    if (m_current_voltage < MINIMUM_IGNITION_VOLTAGE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        if (channel == 1)
        {
            nrf_gpio_pin_set(IGNITION_CH1);
        }
        else if (channel == 2)
        {
            nrf_gpio_pin_set(IGNITION_CH2);
        }
        return NRF_SUCCESS;
    }
}

void ignition_trigger_off(uint8_t channel)
{
    if (channel == 1)
    {
        nrf_gpio_pin_clear(IGNITION_CH1);
    }
    else if (channel == 2)
    {
        nrf_gpio_pin_clear(IGNITION_CH2);
    }
}
