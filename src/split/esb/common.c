/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "common.h"
#include "app_esb.h"

#include <zephyr/sys/crc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

void zmk_split_esb_tx(struct zmk_split_esb_state *state) {
    while (ring_buf_size_get(state->tx_buf) >= (sizeof(struct esb_msg_prefix) 
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
        + sizeof(struct esb_msg_postfix) 
#endif
        + sizeof(struct esb_msg_meta))) {

        struct esb_msg_prefix prefix;
        if (ring_buf_peek(state->tx_buf, (uint8_t *)&prefix, sizeof(prefix)) != sizeof(prefix)) {
            break;
        }

        if (memcmp(&prefix.magic_prefix, &ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                   sizeof(prefix.magic_prefix)) != 0) {
            LOG_WRN("TX prefix mismatch, skipping 1 byte");
            uint8_t dummy;
            ring_buf_get(state->tx_buf, &dummy, 1);
            continue;
        }

        size_t ota_len = sizeof(prefix) + prefix.payload_size
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
            + sizeof(struct esb_msg_postfix)
#endif
            ;
        size_t total_item_len = ota_len + sizeof(struct esb_msg_meta);

        if (ring_buf_size_get(state->tx_buf) < total_item_len) {
            break;
        }

        if (ota_len > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
            LOG_ERR("Item length %u exceeds CONFIG_ESB_MAX_PAYLOAD_LENGTH %u", ota_len, CONFIG_ESB_MAX_PAYLOAD_LENGTH);
            ring_buf_reset(state->tx_buf);
            break;
        }

        uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH + sizeof(struct esb_msg_meta)];
        uint32_t read_bytes = ring_buf_get(state->tx_buf, buf, total_item_len);
        if (read_bytes != total_item_len) {
            LOG_ERR("Failed to read complete item from tx_buf (%u vs %u)", read_bytes, total_item_len);
            break;
        }

        struct esb_msg_meta meta;
        memcpy(&meta, &buf[ota_len], sizeof(meta));

        app_esb_data_t tx_data = {
            .pipe = state->tx_pipe,
            .data = buf,
            .len = ota_len,
            .msg_id = meta.msg_id,
            .max_retry = meta.max_retry
        };
        zmk_split_esb_send(&tx_data);
    }
}

static K_SEM_DEFINE(esb_cb_sem, 1, 1);

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_state *state) {
    switch(event->evt_type) {
        case APP_ESB_EVT_TX_SUCCESS:
            // LOG_DBG("ESB TX sent");
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_tx(state);
            }
            break;
        case APP_ESB_EVT_TX_FAIL:
            // LOG_WRN("ESB TX failed");
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_tx(state);
            }
            break;
        case APP_ESB_EVT_RX:
            // LOG_DBG("ESB RX received: {%d} %d", event->pipe, event->data_length);

            struct ring_buf *rx_buf = &state->rx_bufs[event->pipe];

            if (ring_buf_space_get(rx_buf) < event->data_length) {
                LOG_WRN("No room to receive (have %d but only space for %d/%d)",
                        event->data_length, ring_buf_space_get(rx_buf), 
                        ring_buf_capacity_get(rx_buf));
                break;
            }

            size_t received = ring_buf_put(rx_buf, event->buf, event->data_length);
            if (received < event->data_length) {
                LOG_ERR("RX overrun! %d < %d", received, event->data_length);
                break;
            }

            // LOG_DBG("RX + %3d and now buffer is %3d", received, ring_buf_size_get(&rx_buf));
            if (state->process_rx_callback) {
                state->process_rx_callback(event->pipe);
            }

            break;
        default:
            LOG_ERR("Unknown APP ESB event!");
            break;
    }
}

int zmk_split_esb_get_item(struct ring_buf *rx_buf, uint8_t *env, size_t env_size) {
    while (ring_buf_size_get(rx_buf) > (sizeof(struct esb_msg_prefix) 
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
            + sizeof(struct esb_msg_postfix)
#endif
    )) {
        struct esb_msg_prefix prefix;

        __ASSERT_EVAL(
            (void)ring_buf_peek(rx_buf, (uint8_t *)&prefix, sizeof(prefix)),
            uint32_t peek_read = ring_buf_peek(rx_buf, (uint8_t *)&prefix, sizeof(prefix)),
            peek_read == sizeof(prefix), "Somehow read less than we expect from the RX buffer");

        if (memcmp(&prefix.magic_prefix, &ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                   sizeof(prefix.magic_prefix)) != 0) {
            uint8_t dummy;
            ring_buf_get(rx_buf, &dummy, 1);
            continue;
        }

        size_t payload_to_read = sizeof(prefix) + prefix.payload_size;

        if (payload_to_read > env_size) {
            LOG_WRN("Invalid message with payload %d bigger than expected max %d", 
                payload_to_read, env_size);
            uint8_t dummy;
            ring_buf_get(rx_buf, &dummy, 1);
            continue;
        }

        if (ring_buf_size_get(rx_buf) < (payload_to_read 
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
            + sizeof(struct esb_msg_postfix)
#endif
        )) {
            return -EAGAIN;
        }

        // Now that prefix matches, read it out so we can read the rest of the payload.
        __ASSERT_EVAL((void)ring_buf_get(rx_buf, env, payload_to_read),
                      uint32_t read = ring_buf_get(rx_buf, env, payload_to_read),
                      read == payload_to_read,
                      "Somehow read less than we expect from the RX buffer");

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_MSG_POSTFIX_CRC)
        struct esb_msg_postfix postfix;
        __ASSERT_EVAL((void)ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
                      uint32_t read = ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
                      read == sizeof(postfix),
                      "Somehow read less of the postfix than we expect from the RX buffer");

        uint32_t crc = crc32_ieee(env, payload_to_read);

        if (crc != postfix.crc) {
            LOG_WRN("Data corruption in received peripheral event, skipping (%d vs %d)",
                    crc, postfix.crc);
            return -EINVAL;
        }
#endif

        return 0;
    }

    return -EAGAIN;
}
