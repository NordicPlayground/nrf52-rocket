#include "strato_ignition.h"
#include "pca20027.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "strato_app_config.h"
#include "ble_srs.h"
#include "strato_led.h"

#define SAMPLES_IN_BUFFER 10

static const nrf_drv_timer_t       m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t           m_buffer_pool[1][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t           m_ppi_channel;
static uint32_t                    m_adc_evt_counter;
static uint32_t                    m_sample_period;
static float                       m_current_voltage;
static ignition_adc_evt_handler_t  m_adc_cb;


void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    //Not needed
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        //SEGGER_RTT_printf(0, "ADC event number: %d\r\n",(int)m_adc_evt_counter);

        uint16_t average_result = 0;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            average_result = average_result + p_event->data.done.p_buffer[i];
        }

        average_result = average_result/SAMPLES_IN_BUFFER;

        m_current_voltage = ((double)(2*average_result*3))*(0.825)/((1024));

        m_adc_cb(m_current_voltage);
        m_adc_evt_counter++;
    }
}

void ignition_init(ignition_init_t * p_params)
{
    m_sample_period = p_params->adc_sampling_period_ms;
    m_adc_cb = p_params->adc_evt_handler;
    //Pin configs
    nrf_gpio_cfg_output(BOOST_5V_ENABLE);
    nrf_gpio_cfg_output(SERVO_ENABLE);
    nrf_gpio_cfg_output(IGNITION_CH1);
    nrf_gpio_cfg_output(IGNITION_CH2);
    nrf_gpio_cfg_output(SC_DUMP);
    nrf_gpio_cfg_input(POWER_GOOD, NRF_GPIO_PIN_NOPULL);

    nrf_gpio_pin_set(SERVO_ENABLE);

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


    /* setup m_timer for compare event every m_sample_period ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, m_sample_period);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL1);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_config =
    {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_3,
        .reference  = NRF_SAADC_REFERENCE_VDD4,
        .acq_time   = NRF_SAADC_ACQTIME_10US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .pin_p      = nrf_drv_saadc_gpio_to_ain(VSC_SENSE),
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    //NULL = Default config of 10bit, no oversampling, and irq priorotiy low
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

void power_5v_enable(bool state)
{
    //Enable 5V Boost Converter
    if (state)
    {
        nrf_gpio_pin_set(BOOST_5V_ENABLE);

        while(nrf_gpio_pin_read(POWER_GOOD) == 1)
        {
            SEGGER_RTT_printf(0, "...");
        }
        SEGGER_RTT_printf(0, "Power Good");
        SEGGER_RTT_printf(0, "\r\n");
        leds_set_rgb(leds_current_value_get() | 0xFF0000);
        nrf_gpio_pin_set(SERVO_ENABLE);

    }
    else
    {
        nrf_gpio_pin_clear(BOOST_5V_ENABLE);
        leds_set_rgb(leds_current_value_get() ^ 0xFF0000);

        //Supercap voltage is leaking into the translator
        nrf_gpio_pin_clear(SERVO_ENABLE);
    }
}

ret_code_t ignition_cap_adc_sample_begin(void)
{
    return nrf_drv_ppi_channel_enable(m_ppi_channel);
}

ret_code_t ignition_cap_adc_sample_end(void)
{
    nrf_drv_timer_disable(&m_timer);
    return nrf_drv_ppi_channel_disable(m_ppi_channel);
}

float ignition_cap_current_voltage_get(void)
{
    return m_current_voltage;
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
