/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/split/transport/types.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "app_esb.h"
#include "common.h"

#define MAX_EVENT_ENV_SIZE MAX(sizeof(struct esb_event_envelope), sizeof(struct esb_key_state_envelope))

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
#define TX_BUFFER_SIZE (MAX_EVENT_ENV_SIZE + sizeof(struct esb_msg_postfix) + sizeof(struct esb_msg_meta))
#define RX_BUFFER_SIZE (sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix))
#else
#define TX_BUFFER_SIZE (MAX_EVENT_ENV_SIZE + sizeof(struct esb_msg_meta))
#define RX_BUFFER_SIZE (sizeof(struct esb_command_envelope))
#endif

RING_BUF_DECLARE(tx_buf, TX_BUFFER_SIZE * CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS);

#define RX_RING_BUF_SIZE (RX_BUFFER_SIZE * CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS)
struct ring_buf rx_bufs[CONFIG_ESB_PIPE_COUNT];
uint8_t rx_bufs_data[CONFIG_ESB_PIPE_COUNT][RX_RING_BUF_SIZE];

static const uint8_t peripheral_id = CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID;

static uint8_t position_state[ESB_KEY_STATE_LEN];

static void process_rx_cb(uint8_t pipe);

static struct zmk_split_esb_state state = {
    .tx_pipe = CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID % CONFIG_ESB_PIPE_COUNT,
    .process_rx_callback = process_rx_cb,
    .tx_buf = &tx_buf,
    .rx_bufs = rx_bufs,
};

static void begin_tx(void) {
    zmk_split_esb_tx(&state);
}

void zmk_split_esb_on_ptx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &state);
}

static ssize_t get_payload_data_size(const struct zmk_split_transport_peripheral_event *evt) {
    switch (evt->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return sizeof(evt->data.input_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        return sizeof(evt->data.key_position_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        return sizeof(evt->data.sensor_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return sizeof(evt->data.battery_event);
    default:
        return -ENOTSUP;
    }
}

static uint8_t get_retry_count(const struct zmk_split_transport_peripheral_event *evt) {
    switch (evt->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return CONFIG_ZMK_SPLIT_ESB_RETRY_INPUT_EVENT;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        return CONFIG_ZMK_SPLIT_ESB_RETRY_KEY_POSITION;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        return CONFIG_ZMK_SPLIT_ESB_RETRY_SENSOR_EVENT;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return CONFIG_ZMK_SPLIT_ESB_RETRY_BATTERY_EVENT;
    default:
        return 0;
    }
}

static bool any_key_pressed(void) {
    for (int i = 0; i < ESB_KEY_STATE_LEN; i++) {
        if (position_state[i] != 0) {
            return true;
        }
    }
    return false;
}

static int send_key_state(uint8_t retry_count) {
    struct esb_key_state_envelope env = {
        .prefix = {
            .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
            .payload_size = sizeof(struct esb_key_state_payload),
        },
        .payload = {
            .source = peripheral_id | ESB_SOURCE_KEY_STATE_FLAG,
        },
    };
    memcpy(env.payload.state, position_state, ESB_KEY_STATE_LEN);

    size_t evt_env_len = sizeof(env);

    if (ring_buf_space_get(&tx_buf) < ESB_MSG_EXTRA_SIZE + sizeof(env.payload)) {
        LOG_WRN("No room to send key state event to central");
        return -ENOSPC;
    }

    ring_buf_put(&tx_buf, (uint8_t *)&env, evt_env_len);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, evt_env_len)};
    ring_buf_put(&tx_buf, (uint8_t *)&postfix, sizeof(postfix));
#endif

    static uint16_t evt_msg_id = 0;
    if (++evt_msg_id >= UINT16_MAX - 1000) {
        evt_msg_id = 1;
    }

    struct esb_msg_meta meta = {.msg_id = evt_msg_id, .max_retry = retry_count};
    ring_buf_put(&tx_buf, (uint8_t *)&meta, sizeof(meta));

    begin_tx();
    return 0;
}

static struct k_work_delayable key_state_work;
static bool release_confirmed = true;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
static struct k_work_delayable idle_reset_work;

static void idle_reset_work_cb(struct k_work *work) {
    if (!any_key_pressed()) {
        esb_rf_ch_reset();
    }
}
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */

static void key_state_work_cb(struct k_work *work) {
    if (any_key_pressed()) {
        // Slow keep-alive (3000ms) only when keys are held down long-term.
        // Never flood the channel during normal typing/chording.
        send_key_state(CONFIG_ZMK_SPLIT_ESB_RETRY_KEY_POSITION);
        k_work_schedule(&key_state_work, K_MSEC(3000));
        release_confirmed = false;
    } else {
        // All keys are released: send confirmation once more to guarantee delivery
        if (!release_confirmed) {
            send_key_state(CONFIG_ZMK_SPLIT_ESB_RETRY_KEY_POSITION + 4);
            release_confirmed = true;
        }
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
        k_work_schedule(&idle_reset_work, K_MSEC(3000));
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */
    }
}

static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    if (event->type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT) {
        uint32_t pos = event->data.key_position_event.position;
        if (pos >= ESB_KEY_STATE_LEN * 8) {
            LOG_ERR("Key position %d exceeds maximum %d", pos, ESB_KEY_STATE_LEN * 8);
            return -EINVAL;
        }

        if (event->data.key_position_event.pressed) {
            position_state[pos / 8] |= BIT(pos % 8);
        } else {
            position_state[pos / 8] &= ~BIT(pos % 8);
        }

        bool pressed = any_key_pressed();
        uint8_t retries = pressed ? CONFIG_ZMK_SPLIT_ESB_RETRY_KEY_POSITION
                                  : (CONFIG_ZMK_SPLIT_ESB_RETRY_KEY_POSITION + 4);

        send_key_state(retries);

        if (pressed) {
            release_confirmed = false;
            k_work_schedule(&key_state_work, K_MSEC(3000));
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
            k_work_cancel_delayable(&idle_reset_work);
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */
        } else {
            // Schedule release confirmation in 20ms
            k_work_schedule(&key_state_work, K_MSEC(20));
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
            k_work_schedule(&idle_reset_work, K_MSEC(3000));
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */
        }

        return 0;
    }

    ssize_t data_size = get_payload_data_size(event);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    size_t payload_size = data_size
                        + sizeof(peripheral_id)
                        + sizeof(enum zmk_split_transport_peripheral_event_type);

    if (ring_buf_space_get(&tx_buf) < ESB_MSG_EXTRA_SIZE + payload_size) {
        LOG_WRN("No room to send event to the central (have %d but only space for %d/%d)",
                ESB_MSG_EXTRA_SIZE + payload_size, ring_buf_space_get(&tx_buf),
                ring_buf_capacity_get(&tx_buf));
        ring_buf_reset(&tx_buf);
        return -ENOSPC;
    }

    struct esb_event_envelope env = {.prefix = {
                                        .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                                        .payload_size = payload_size,
                                    },
                                    .payload = {
                                        .source = peripheral_id,
                                        .event = *event,
                                    }};

    size_t evt_env_len = sizeof(env.prefix) + payload_size;

    size_t put = ring_buf_put(&tx_buf, (uint8_t *)&env, evt_env_len);
    if (put != evt_env_len) {
        LOG_WRN("Failed to put the whole message (%d vs %d)", put, evt_env_len);
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, evt_env_len)};

    put = ring_buf_put(&tx_buf, (uint8_t *)&postfix, sizeof(postfix));
    if (put != sizeof(postfix)) {
        LOG_WRN("Failed to put the postfix (%d vs %d)", put, sizeof(postfix));
    }
#endif

    static uint16_t evt_msg_id = 0;
    if (++evt_msg_id >= UINT16_MAX - 1000) {
        evt_msg_id = 1;
    }

    uint8_t max_retry = get_retry_count(event);
    struct esb_msg_meta meta = {.msg_id = evt_msg_id, .max_retry = max_retry};

    put = ring_buf_put(&tx_buf, (uint8_t *)&meta, sizeof(meta));
    if (put != sizeof(meta)) {
        LOG_WRN("Failed to put the meta (%d vs %d)", put, sizeof(meta));
    }

    begin_tx();

    return 0;
}

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

static int split_peripheral_esb_set_enabled(bool enabled) {
    is_enabled = enabled;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT) 
    return zmk_split_esb_set_enable(enabled);
#else
    return 0;
#endif
}

static int
split_peripheral_esb_set_status_callback(zmk_split_transport_peripheral_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_peripheral_esb_get_status(void) {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
    };
}

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = split_peripheral_esb_report_event,
    .set_enabled = split_peripheral_esb_set_enabled,
    .set_status_callback = split_peripheral_esb_set_status_callback,
    .get_status = split_peripheral_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(esb_peripheral, &peripheral_api,
                                        CONFIG_ZMK_SPLIT_ESB_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_peripheral, split_peripheral_esb_get_status());
    }
}

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

static int zmk_split_esb_peripheral_init(void) {
    for (int i = 0; i < CONFIG_ESB_PIPE_COUNT; i++) {
        ring_buf_init(&rx_bufs[i], RX_RING_BUF_SIZE, rx_bufs_data[i]);
    }
    int ret = zmk_split_esb_init(APP_ESB_MODE_PTX, zmk_split_esb_on_ptx_esb_callback);
    if (ret < 0) {
        LOG_ERR("zmk_split_esb_init failed (ret %d)", ret);
        return ret;
    }
    k_work_init_delayable(&key_state_work, key_state_work_cb);
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
    k_work_init_delayable(&idle_reset_work, idle_reset_work_cb);
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */
    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static void process_rx_work_cb(struct k_work *work) {
    for (int pipe = 0; pipe < CONFIG_ESB_PIPE_COUNT; pipe++) {
        struct ring_buf *rx_buf = &state.rx_bufs[pipe];
        while (ring_buf_size_get(rx_buf) > ESB_MSG_EXTRA_SIZE) {
            struct esb_command_envelope env;
            int item_err = zmk_split_esb_get_item(rx_buf, (uint8_t *)&env,
                                                  sizeof(struct esb_command_envelope));
            switch (item_err) {
            case 0:
                if (env.payload.cmd.type == ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS) {
                    break;
                }
                if (env.payload.source != peripheral_id) {
                    LOG_WRN("Ignoring command type %d for source %d (expect %d)", 
                            env.payload.cmd.type, env.payload.source, peripheral_id);
                    break;
                }
                zmk_split_transport_peripheral_command_handler(&esb_peripheral, env.payload.cmd);
                break;
            case -EAGAIN:
                break;
            default:
                break;
            }
        }
    }
}

K_WORK_DEFINE(process_rx_work, process_rx_work_cb);

static void process_rx_cb(uint8_t pipe) {
    k_work_submit(&process_rx_work);
}
