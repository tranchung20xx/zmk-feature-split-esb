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
#include <zmk/split/transport/central.h>
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
#define TX_BUFFER_SIZE (sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix))
#define RX_BUFFER_SIZE (MAX_EVENT_ENV_SIZE + sizeof(struct esb_msg_postfix))
#else
#define TX_BUFFER_SIZE (sizeof(struct esb_command_envelope))
#define RX_BUFFER_SIZE (MAX_EVENT_ENV_SIZE)
#endif

RING_BUF_DECLARE(tx_buf, TX_BUFFER_SIZE * CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS);

#define RX_RING_BUF_SIZE (RX_BUFFER_SIZE * CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS)
struct ring_buf rx_bufs[CONFIG_ESB_PIPE_COUNT];
uint8_t rx_bufs_data[CONFIG_ESB_PIPE_COUNT][RX_RING_BUF_SIZE];

static void process_rx_cb(uint8_t pipe);

static struct zmk_split_esb_state state = {
    .tx_pipe = 0,
    .process_rx_callback = process_rx_cb,
    .tx_buf = &tx_buf,
    .rx_bufs = rx_bufs,
};

static void begin_tx(void) {
    zmk_split_esb_tx(&state);
}

static ssize_t get_payload_data_size(const struct zmk_split_transport_central_command *cmd) {
    switch (cmd->type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        return sizeof(cmd->data.invoke_behavior);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        return sizeof(cmd->data.set_physical_layout);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        return sizeof(cmd->data.set_hid_indicators);
    default:
        return -ENOTSUP;
    }
}

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {

    ssize_t data_size = get_payload_data_size(&cmd);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    size_t payload_size = data_size
                        + sizeof(source)
                        + sizeof(enum zmk_split_transport_central_command_type);

    if (ring_buf_space_get(&tx_buf) < ESB_MSG_EXTRA_SIZE + payload_size) {
        LOG_WRN("No room to send command to the peripheral %d (have %d but only space for %d/%d)", 
                source, ESB_MSG_EXTRA_SIZE + payload_size, ring_buf_space_get(&tx_buf),
                ring_buf_capacity_get(&tx_buf));
        ring_buf_reset(&tx_buf);
        return -ENOSPC;
    }

    struct esb_command_envelope env = {.prefix = {
                                            .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                                            .payload_size = payload_size,
                                        },
                                        .payload = {
                                            .source = source,
                                            .cmd = cmd,
                                        }};

    size_t cmd_env_len = sizeof(env.prefix) + payload_size;
    // LOG_HEXDUMP_DBG(&env, cmd_env_len, "Payload");

    size_t put = ring_buf_put(&tx_buf, (uint8_t *)&env, cmd_env_len);
    if (put != cmd_env_len) {
        LOG_WRN("Failed to put the whole message (%d vs %d)", put, cmd_env_len);
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, cmd_env_len)};

    put = ring_buf_put(&tx_buf, (uint8_t *)&postfix, sizeof(postfix));
    if (put != sizeof(postfix)) {
        LOG_WRN("Failed to put the postfix (%d vs %d)", put, sizeof(postfix));
    }
#endif

    static uint16_t cmd_msg_id = 0;
    if (++cmd_msg_id >= UINT16_MAX - 1000) {
        cmd_msg_id = 1;
    }
    // LOG_INF("cmd_msg_id: %d", cmd_msg_id);

    uint8_t max_retry = CONFIG_ZMK_SPLIT_ESB_RETRY_CMD;
    struct esb_msg_meta meta = {.msg_id = cmd_msg_id, .max_retry = max_retry};

    put = ring_buf_put(&tx_buf, (uint8_t *)&meta, sizeof(meta));
    if (put != sizeof(meta)) {
        LOG_WRN("Failed to put the meta (%d vs %d)", put, sizeof(meta));
    }

    state.tx_pipe = source % CONFIG_ESB_PIPE_COUNT;

    begin_tx();

    return 0;
}

void zmk_split_esb_on_prx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &state);
}

static int split_central_esb_get_available_source_ids(uint8_t *sources) {
    sources[0] = 0;

    return 1;
}

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled;

static int split_central_esb_set_enabled(bool enabled) {
    is_enabled = enabled;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT)
    return zmk_split_esb_set_enable(enabled);
#else
    return 0;
#endif
}

static int
split_central_esb_set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_central_esb_get_status() {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
    };
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_esb_send_command,
    .get_available_source_ids = split_central_esb_get_available_source_ids,
    .set_enabled = split_central_esb_set_enabled,
    .set_status_callback = split_central_esb_set_status_callback,
    .get_status = split_central_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(esb_central, &central_api, CONFIG_ZMK_SPLIT_ESB_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_central, split_central_esb_get_status());
    }
}

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

static int zmk_split_esb_central_init(void) {
    for (int i = 0; i < CONFIG_ESB_PIPE_COUNT; i++) {
        ring_buf_init(&rx_bufs[i], RX_RING_BUF_SIZE, rx_bufs_data[i]);
    }
    int ret = zmk_split_esb_init(APP_ESB_MODE_PRX, zmk_split_esb_on_prx_esb_callback);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }
    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

extern const struct zmk_split_transport_central *active_transport;

#define MAX_PERIPHERALS 16

static uint8_t peripheral_position_state[MAX_PERIPHERALS][ESB_KEY_STATE_LEN];

static void process_key_state(uint8_t source, const uint8_t *new_state) {
    if (&esb_central != active_transport) {
        return;
    }
    if (source >= MAX_PERIPHERALS) {
        LOG_WRN("Source %d exceeds MAX_PERIPHERALS", source);
        return;
    }

    uint8_t changed[ESB_KEY_STATE_LEN];
    for (int i = 0; i < ESB_KEY_STATE_LEN; i++) {
        changed[i] = new_state[i] ^ peripheral_position_state[source][i];
        peripheral_position_state[source][i] = new_state[i];
    }

    for (int i = 0; i < ESB_KEY_STATE_LEN; i++) {
        if (!changed[i]) {
            continue;
        }
        for (int j = 0; j < 8; j++) {
            if (changed[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                bool pressed = (new_state[i] & BIT(j)) != 0;

                struct zmk_split_transport_peripheral_event ev = {
                    .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                    .data = {
                        .key_position_event = {
                            .position = position,
                            .pressed = pressed,
                        }
                    }
                };

                zmk_split_transport_central_peripheral_event_handler(
                    &esb_central, source, ev);
            }
        }
    }
}

static void process_rx_work_cb(struct k_work *work) {
    for (int pipe = 0; pipe < CONFIG_ESB_PIPE_COUNT; pipe++) {
        struct ring_buf *rx_buf = &state.rx_bufs[pipe];
        while (ring_buf_size_get(rx_buf) > ESB_MSG_EXTRA_SIZE) {
            union {
                struct esb_msg_prefix prefix;
                struct esb_event_envelope event_env;
                struct esb_key_state_envelope key_state_env;
            } env;
            int item_err = zmk_split_esb_get_item(rx_buf, (uint8_t *)&env, sizeof(env));
            switch (item_err) {
            case 0: {
                uint8_t raw_source = env.event_env.payload.source;
                if (raw_source & ESB_SOURCE_KEY_STATE_FLAG) {
                    uint8_t clean_source = raw_source & ~ESB_SOURCE_KEY_STATE_FLAG;
                    process_key_state(clean_source, env.key_state_env.payload.state);
                } else {
                    if (&esb_central == active_transport) {
                        static uint8_t key_pos_states[(CONFIG_ZMK_SPLIT_ESB_AUTO_HEAL_KEY_POS_MAX + 8) / 8];
                        struct zmk_split_transport_peripheral_event ev = env.event_env.payload.event;

                        if (ev.type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT) {
                            uint8_t pressed = ev.data.key_position_event.pressed;
                            uint8_t position = ev.data.key_position_event.position;

                            if (position < CONFIG_ZMK_SPLIT_ESB_AUTO_HEAL_KEY_POS_MAX) {
                                if (pressed) {
                                    if ((key_pos_states[position / 8] >> (position % 8)) & 1) {
                                        LOG_WRN("re-pressing detected, inject release event");
                                        struct zmk_position_state_changed state_ev = {
                                            .source = env.event_env.payload.source,
                                            .position = position,
                                            .state = !pressed,
                                            .timestamp = k_uptime_get()
                                        };
                                        raise_zmk_position_state_changed(state_ev);
                                        k_sleep(K_MSEC(1));
                                    }
                                    key_pos_states[position / 8] |= 1 << (position % 8);
                                } else {
                                    key_pos_states[position / 8] &= ~(1 << (position % 8));
                                }
                            }
                        }

                        zmk_split_transport_central_peripheral_event_handler(
                            &esb_central, env.event_env.payload.source, env.event_env.payload.event);
                    }
                }
                break;
            }
            case -EAGAIN:
                break;
            case -EINVAL:
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
                esb_rf_ch_hop();
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */
                break;
            default:
                // LOG_WRN("Issue fetching an item from the RX buffer: %d", item_err);
                break;
            }
        }
    }
}

K_WORK_DEFINE(process_rx_work, process_rx_work_cb);

static void process_rx_cb(uint8_t pipe) {
    k_work_submit(&process_rx_work);
}
