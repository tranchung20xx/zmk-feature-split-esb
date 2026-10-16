/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_esb.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>

#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT)
#include "timeslot.h"
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type);
#endif

#define DT_DRV_COMPAT zmk_esb_split
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define HAS_BASE_ADDR_0 (DT_INST_NODE_HAS_PROP(0, base_addr_0))
#define HAS_BASE_ADDR_1 (DT_INST_NODE_HAS_PROP(0, base_addr_1))
#define HAS_ADDR_PREFIX (DT_INST_NODE_HAS_PROP(0, addr_prefix))

#define BASE_ADDR_0_LEN (DT_INST_PROP_LEN(0, base_addr_0))
#define BASE_ADDR_1_LEN (DT_INST_PROP_LEN(0, base_addr_1))
#define ADDR_PREFIX_LEN (DT_INST_PROP_LEN(0, addr_prefix))

#if (!HAS_BASE_ADDR_0 || BASE_ADDR_0_LEN != 4)
#error "zmk,esb-split :: base-addr-0 must include 4 bytes"
#endif

#if (!HAS_BASE_ADDR_1 || BASE_ADDR_1_LEN != 4)
#error "zmk,esb-split :: base-addr-1 must include 4 bytes"
#endif

#if (!HAS_ADDR_PREFIX || ADDR_PREFIX_LEN > CONFIG_ESB_PIPE_COUNT)
#error "zmk,esb-split :: base-addr-0 must include 8 bytes"
#endif

uint8_t esb_base_addr_0[BASE_ADDR_0_LEN] = DT_INST_PROP(0, base_addr_0);
uint8_t esb_base_addr_1[BASE_ADDR_1_LEN] = DT_INST_PROP(0, base_addr_1);
uint8_t esb_addr_prefix[ADDR_PREFIX_LEN] = DT_INST_PROP(0, addr_prefix);

#else
#error "Need to create a node with compatible of 'zmk,esb-split` with `all `address` property set."
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)

#include <zephyr/sys/util_macro.h>

#define RF_CH_IDX(n, _) (CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP_CH_MIN + (n) * CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP_CH_STEP)
static uint8_t esb_rf_ch[CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP_CH_COUNT] = {
    LISTIFY(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP_CH_COUNT, RF_CH_IDX, (,))
};

static uint8_t esb_rf_ch_idx;

uint8_t esb_rf_ch_hop(void) {
    esb_rf_ch_idx = (esb_rf_ch_idx + 1) % ARRAY_SIZE(esb_rf_ch);
    LOG_DBG("rf ch: %d", esb_rf_ch[esb_rf_ch_idx]);
    esb_set_rf_channel(esb_rf_ch[esb_rf_ch_idx]);
    return esb_rf_ch[esb_rf_ch_idx];
}

void esb_rf_ch_reset(void) {
    if (esb_rf_ch_idx != 0) {
        esb_rf_ch_idx = 0;
        esb_set_rf_channel(esb_rf_ch[0]);
        LOG_DBG("rf ch reset to primary: %d", esb_rf_ch[0]);
    }
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */

static app_esb_callback_t m_callback;

// Track msgq full errors
static uint32_t m_msgq_full_last_time;

struct app_esb_tx_item {
    struct esb_payload payload;
    uint16_t msg_id;
    uint8_t retry_left;
    uint8_t max_retry;
};

// Define a buffer of payloads to store TX payloads in between timeslots
K_MSGQ_DEFINE(m_msgq_tx_payloads, sizeof(struct app_esb_tx_item), 
              CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS, 4);

static struct app_esb_tx_item m_active_tx_item;
static bool m_has_active_tx = false;

static app_esb_mode_t m_mode;
static bool m_active = false;
static bool m_enabled = false;

static int pull_packet_from_tx_msgq(void);

static void event_handler(struct esb_evt const *event) {
    app_esb_event_t m_event;
    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            m_has_active_tx = false;
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_SUCCESS;
            m_callback(&m_event);
            pull_packet_from_tx_msgq();
            break;
        case ESB_EVENT_TX_FAILED:
            LOG_WRN("TX FAILED, msg_id: %d, retry left: %d",
                    m_active_tx_item.msg_id, m_active_tx_item.retry_left);

            if (m_active_tx_item.retry_left > 0) {
                m_active_tx_item.retry_left--;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
                esb_rf_ch_hop();
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */

                esb_flush_tx();
                if (esb_write_payload(&m_active_tx_item.payload) == 0) {
                    if (esb_start_tx() == 0) {
                        break;
                    }
                }
            }

            // Retries exhausted or immediate write/start failed
            m_has_active_tx = false;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
            esb_rf_ch_hop();
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */

            esb_flush_tx();
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_FAIL;
            m_callback(&m_event);
            pull_packet_from_tx_msgq();
            break;
        case ESB_EVENT_RX_RECEIVED:
            // LOG_DBG("RX SUCCESS");
            struct esb_payload rx_payload;
            while (esb_read_rx_payload(&rx_payload) == 0) {
                // LOG_DBG("Chunk %d, pipe: %d, len: %d", 
                //     rx_payload.pid, rx_payload.pipe, rx_payload.length);
                uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
                memcpy(buf, rx_payload.data, rx_payload.length);
                // LOG_DBG("Packet len: %d", rx_payload.length);
                // LOG_HEXDUMP_INF(buf, rx_payload.length, "rx_payload");
                m_event.evt_type = APP_ESB_EVT_RX;
                m_event.pipe = rx_payload.pipe;
                m_event.buf = buf;
                m_event.data_length = rx_payload.length;
                m_callback(&m_event);
            }
            if (m_mode == APP_ESB_MODE_PRX) {
              pull_packet_from_tx_msgq();
            }
            break;
    }
}

static int clocks_start(void) {
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

    LOG_DBG("HF clock started");
    return 0;
}

static int esb_initialize(app_esb_mode_t mode) {
    int err;
    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.retransmit_delay = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY;
    config.retransmit_count = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT;
    config.bitrate = ESB_BITRATE_2MBPS;
    config.use_fast_ramp_up = true;
    config.event_handler = event_handler;
    config.mode = (mode == APP_ESB_MODE_PTX) ? ESB_MODE_PTX : ESB_MODE_PRX;
    config.tx_mode = ESB_TXMODE_MANUAL_START;
    config.selective_auto_ack = true;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_CTLR_TX_PWR_PLUS_8)
    config.tx_output_power = ESB_TX_POWER_8DBM;
#endif

    err = esb_init(&config);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP)
    esb_set_rf_channel(esb_rf_ch[esb_rf_ch_idx]);
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_RF_CH_HOP) */

    if (err) {
        return err;
    }

    err = esb_set_base_address_0(esb_base_addr_0);
    if (err) {
        return err;
    }

    err = esb_set_base_address_1(esb_base_addr_1);
    if (err) {
        return err;
    }

    err = esb_set_prefixes(esb_addr_prefix, ARRAY_SIZE(esb_addr_prefix));
    if (err) {
        return err;
    }

    NVIC_SetPriority(RADIO_IRQn, 0);

    if (mode == APP_ESB_MODE_PRX) {
        esb_start_rx();
    }

    return 0;
}

static int pull_packet_from_tx_msgq(void) {
    int ret = 0;
    int esb_ret;

    if (!esb_is_idle() && m_mode == APP_ESB_MODE_PTX) {
        return -EBUSY;
    }

    if (!m_has_active_tx) {
        if (k_msgq_get(&m_msgq_tx_payloads, &m_active_tx_item, K_NO_WAIT) != 0) {
            return 0; // Empty queue
        }
        m_has_active_tx = true;
    }

    ret = esb_write_payload(&m_active_tx_item.payload);

    if (ret == -ENOMEM) {
        esb_flush_tx();
        ret = esb_write_payload(&m_active_tx_item.payload);
    } else if (ret == -EMSGSIZE) {
        LOG_WRN("esb_tx_fifo: tx_payload size too large (%d) > CONFIG_ESB_MAX_PAYLOAD_LENGTH (%d)",
                m_active_tx_item.payload.length, CONFIG_ESB_MAX_PAYLOAD_LENGTH);
        m_has_active_tx = false;
        return ret;
    }

    if (ret == 0) {
        if (m_mode == APP_ESB_MODE_PTX) {
            esb_ret = esb_start_tx();
            if (esb_ret == -EBUSY) {
                LOG_DBG("ESB busy, will retry on next event");
                return -EBUSY;
            } else if (esb_ret < 0 && esb_ret != -ENODATA) {
                LOG_ERR("esb_start_tx failed (%d)", esb_ret);
                return esb_ret;
            }
        }
    } else {
        LOG_WRN("esb_write_payload failed (%d)", ret);
    }

    return ret;
}

int zmk_split_esb_init(app_esb_mode_t mode, app_esb_callback_t callback) {
    int ret;
    m_callback = callback;
    m_mode = mode;
    ret = clocks_start();
    if (ret < 0) {
        return ret;
    }
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT)
    LOG_INF("Timeslothandler init");
    zmk_split_esb_timeslot_init(on_timeslot_start_stop);
#else
    ret = zmk_split_esb_set_enable(true);
    if (ret) {
        LOG_ERR("esb enable failed: %d", ret);
    }
#endif
    return 0;
}

int zmk_split_esb_set_enable(bool enabled) {
    m_enabled = enabled;
    if (enabled) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT)
        zmk_split_esb_timeslot_open_session();
#else
        int ret = esb_initialize(m_mode);
        if (ret) {
            LOG_ERR("set_enable: esb_initialize failed: %d", ret);
            return ret;
        }
        m_active = true;
        if (m_mode == APP_ESB_MODE_PTX) {
            m_has_active_tx = false;
        }
        pull_packet_from_tx_msgq();
#endif
        return 0;
    } else {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT)
        zmk_split_esb_timeslot_close_session();
#else
        m_active = false;
        esb_disable();
#endif
        return 0;
    }
}

int zmk_split_esb_send(app_esb_data_t *tx_packet) {
    int ret = 0;
    if (!tx_packet->len) {
        LOG_WRN("bypass queuing null payload");
        return 0;
    }

    struct app_esb_tx_item item = {
        .msg_id = tx_packet->msg_id,
        .max_retry = tx_packet->max_retry,
        .retry_left = tx_packet->max_retry,
        .payload = {
            .pipe = tx_packet->pipe,
            .length = tx_packet->len,
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK)
            .noack = false,
#else
            .noack = true,
#endif
        }
    };
    memcpy(item.payload.data, tx_packet->data, tx_packet->len);

    ret = k_msgq_put(&m_msgq_tx_payloads, &item, K_NO_WAIT);

    if (ret == 0) {
        m_msgq_full_last_time = 0;
    } else if (ret == -ENOMSG) {
        uint32_t now = k_uptime_get_32();
        if (!m_msgq_full_last_time) {
            m_msgq_full_last_time = now;
        }
        if (now - m_msgq_full_last_time > CONFIG_ZMK_SPLIT_ESB_MSGQ_FULL_TIMEOUT_MS) {
            LOG_WRN("Msgq full for %dms, clearing msgq", 
                CONFIG_ZMK_SPLIT_ESB_MSGQ_FULL_TIMEOUT_MS);
            k_msgq_purge(&m_msgq_tx_payloads);
            m_has_active_tx = false;
            if (m_active) {
                esb_flush_tx(); 
            }
            m_msgq_full_last_time = 0;
        }
    } else {
        LOG_DBG("Failed to queue esb tx_payload_q (%d)", ret);
    }
    if (m_active) {
        pull_packet_from_tx_msgq();
    }
    return ret;
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT)
static int app_esb_suspend(void) {
    m_active = false;
    if (m_mode == APP_ESB_MODE_PTX) {
        uint32_t irq_key = irq_lock();

        irq_disable(RADIO_IRQn);
        NVIC_DisableIRQ(RADIO_IRQn);

        NRF_RADIO->SHORTS = 0;

        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0);

        NRF_TIMER2->TASKS_STOP = 1;
        NRF_RADIO->INTENCLR = 0xFFFFFFFF;
        
        esb_disable();

        NVIC_ClearPendingIRQ(RADIO_IRQn);

        irq_unlock(irq_key);
    }
    else {
        esb_stop_rx();
    }

    // Todo: Figure out how to use the esb_suspend() function 
    // rather than having to disable at the end of every timeslot
    //esb_suspend();
    return 0;
}

static int app_esb_resume(void) {
    if (m_mode == APP_ESB_MODE_PTX) {
        int err = esb_initialize(m_mode);
        m_active = true;
        m_has_active_tx = false;
        pull_packet_from_tx_msgq();
        return err;
    }
    else {
        int err = esb_initialize(m_mode);
        m_active = true;
        pull_packet_from_tx_msgq();
        return err;
    }
}

/* Callback function signalling that a timeslot is started or stopped */
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type) {
    switch (type) {
        case APP_TS_STARTED:
            app_esb_resume();
            break;
        case APP_TS_STOPPED:
            app_esb_suspend();
            break;
    }
}
#endif

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        return 0;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_USE_TIMESLOT)
    if (m_mode == APP_ESB_MODE_PTX) {
        if (state_ev->state != ZMK_ACTIVITY_ACTIVE && m_enabled) {
            zmk_split_esb_set_enable(false);
        }
        else if (state_ev->state == ZMK_ACTIVITY_ACTIVE && !m_enabled) {
            zmk_split_esb_set_enable(true);
        }
    }
#endif

    return 0;
}

ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);
