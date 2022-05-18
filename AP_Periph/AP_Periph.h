#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include "SRV_Channel/SRV_Channel.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include "../AP_Bootloader/app_comms.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#include <AP_RTC/JitterCorrection.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL_ChibiOS/EventSource.h>

#include <ch.h>

#if HAL_GCS_ENABLED
#include "GCS_MAVLink.h"
#endif

#if defined(HAL_PERIPH_ENABLE_BATTERY_MPPT_PACKETDIGITAL) && HAL_MAX_CAN_PROTOCOL_DRIVERS < 2
#error "Battery MPPT PacketDigital driver requires at least two CAN Ports"
#endif


#include "Parameters.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init();
void stm32_watchdog_pat();
#endif
/*
  app descriptor compatible with MissionPlanner
 */
extern const struct app_descriptor app_descriptor;

class AP_Periph_FW {
public:
    AP_Periph_FW();

    CLASS_NO_COPY(AP_Periph_FW);

    static AP_Periph_FW* get_singleton()
    {
        if (_singleton == nullptr) {
            AP_HAL::panic("AP_Periph_FW used before allocation.");
        }
        return _singleton;
    }

    void init();
    void update();

    Parameters g;

    void can_start();
    void can_update();
    void can_mag_update();
    void can_gps_update();
    void send_moving_baseline_msg();
    void send_relposheading_msg();
    void can_baro_update();
    void can_airspeed_update();
    void can_rangefinder_update();
    void can_battery_update();
    void can_imu_update();

    void load_parameters();
    void prepare_reboot();

    bool canfdout() const { return (g.can_fdmode == 1); }

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
    void check_for_serial_reboot_cmd(const int8_t serial_index);
#endif

    void gpio_passthrough_isr(uint8_t pin, bool pin_state, uint32_t timestamp);

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static ChibiOS::CANIface* can_iface_periph[HAL_NUM_CAN_IFACES];
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static HALSITL::CANIface* can_iface_periph[HAL_NUM_CAN_IFACES];
#endif

    AP_SerialManager serial_manager;

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_GPS gps;
#if HAL_NUM_CAN_IFACES >= 2
    int8_t gps_mb_can_port = -1;
#endif
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    Compass compass;
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Baro baro;
#endif

#ifdef HAL_PERIPH_ENABLE_BATTERY
    struct AP_Periph_Battery {
        void handle_battery_failsafe(const char* type_str, const int8_t action) { }
        AP_BattMonitor lib{0, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::AP_Periph_Battery::handle_battery_failsafe, void, const char*, const int8_t), nullptr};

        uint32_t last_read_ms;
        uint32_t last_can_send_ms;
    } battery;
#endif

#if HAL_NUM_CAN_IFACES >= 2
    // This allows you to change the protocol and it continues to use the one at boot.
    // Without this, changing away from UAVCAN causes loss of comms and you can't
    // change the rest of your params or verify it succeeded.
    AP_CANManager::Driver_Type can_protocol_cached[HAL_NUM_CAN_IFACES];
#endif
    
#ifdef HAL_PERIPH_ENABLE_ADSB
    void adsb_init();
    void adsb_update();
    void can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg);
    struct {
        mavlink_message_t msg;
        mavlink_status_t status;
    } adsb;
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    AP_Airspeed airspeed;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    RangeFinder rangefinder;
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    void pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);
    void pwm_hardpoint_init();
    void pwm_hardpoint_update();
    struct {
        uint8_t last_state;
        uint32_t last_ts_us;
        uint32_t last_send_ms;
        uint16_t pwm_value;
        uint16_t highest_pwm;
    } pwm_hardpoint;
#endif


    SRV_Channels servo_channels;
    bool rcout_has_new_data_to_update;
    void rcout_update();

    void update_rainbow();

    // notification object for LEDs, buzzers etc
    AP_Notify notify;
    uint64_t vehicle_state = 1; // default to initialisation
    float yaw_earth;
    uint32_t last_vehicle_state;
    bool led_cmd_override = false;
    // Handled under LUA script to control LEDs
    void led_command_override() { led_cmd_override = true; }

    void toshibaled_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte);

    void i2c_setup();

    void rm3100_recv_byte(uint8_t idx, uint8_t byte);
    bool rm3100_send_byte(uint8_t &send_byte);

    float get_yaw_earth() { return yaw_earth; }
    uint32_t get_vehicle_state() { return vehicle_state; }
    void handle_rm3100_response();

#if AP_SCRIPTING_ENABLED
    AP_Scripting scripting;
#endif

#if HAL_LOGGING_ENABLED
    static const struct LogStructure log_structure[];
    AP_Logger logger;
#endif

#if HAL_GCS_ENABLED
    GCS_Periph _gcs;
#endif

#if HAL_INS_ENABLED
    AP_InertialSensor imu;
#endif

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
    uint32_t last_airspeed_update_ms;
    uint64_t last_time_sync_usec;
    int64_t time_offset_usec;

    uint8_t i2c_led_color_red;
    uint8_t i2c_led_color_green;
    uint8_t i2c_led_color_blue;
    uint8_t i2c_led_reg;
    bool i2c_new_led_data;

    uint8_t i2c2_transfer_byte_idx;
    uint8_t i2c2_transfer_address;
    uint8_t i2c2_transfer_direction;

    uint8_t rm3100_reg;
    uint8_t rm3100_reg_val;
    uint8_t rm3100_mag_data[9];
    bool mag_data_requested;
    bool rm3100_response_requested;
    bool rm3100_write_requested;

    HAL_EventHandle i2c_event_handle;
    ChibiOS::EventSource i2c_event_source;

    static AP_Periph_FW *_singleton;

    // show stack as DEBUG msgs
    void show_stack_free();

    static bool no_iface_finished_dna;
    bool saw_lock_once;
    JitterCorrection jitter;
};

namespace AP
{
    AP_Periph_FW& periph();
}

extern AP_Periph_FW periph;

extern "C" {
void can_printf(const char *fmt, ...) FMT_PRINTF(1,2);
}

