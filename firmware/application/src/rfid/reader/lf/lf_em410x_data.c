#include "lf_reader_data.h"

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "protocols/em410x.h"
#include "protocols/protocols.h"
#include "tag_base_type.h"

#define NRF_LOG_MODULE_NAME em410x
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

#define EM410X_BUFFER_SIZE (128)

// Electra read tuning. A plain RF/64 frame is also the base of an Electra frame,
// and the plain decoder locks ~64 bits sooner, so plain would otherwise always win.
// A plain RF/64 hit is therefore held for EM_ELECTRA_HOLD_MS: if an Electra frame
// appears in that window it is accepted as Electra; otherwise the plain result is
// returned. (A single Electra read is trusted -- a real Electra fob reads cleanly.)
#define EM_ELECTRA_HOLD_MS (130)
#define EM_MAX_ID_SIZE (13)  // Electra 13-byte id (>= any plain 5-byte id)

static circular_buffer cb;

// GPIO interrupt recovery function is used to detect the descending edge
void gpio_int0_cb(void) {
    uint32_t cntr = get_lf_counter_value();
    uint16_t val = 0;
    if (cntr > 0xff) {
        val = 0xff;
    } else {
        val = cntr & 0xff;
    }
    cb_push_back(&cb, &val);
    clear_lf_counter_value();
}

static void init_em410x_hw(void) {
    register_rio_callback(gpio_int0_cb);
    lf_125khz_radio_gpiote_enable();
}

static void uninit_em410x_hw(void) {
    lf_125khz_radio_gpiote_disable();
    unregister_rio_callback();
}

bool em410x_read(uint8_t *data, uint32_t timeout_ms) {
    void **codecs = malloc(em410x_protocols_size * sizeof(void *));
    for (size_t i = 0; i < em410x_protocols_size; i++) {
        codecs[i] = em410x_protocols[i]->alloc();
        em410x_protocols[i]->decoder.start(codecs[i], 0);
    }

    cb_init(&cb, EM410X_BUFFER_SIZE, sizeof(uint16_t));
    init_em410x_hw();
    start_lf_125khz_radio();

    bool ok = false;

    // A plain RF/64 hit is ambiguous with an Electra base, so hold it: return it
    // if no Electra shows up within the hold window, but let Electra take over.
    bool plain_pending = false;
    uint8_t plain_buf[2 + EM_MAX_ID_SIZE];
    uint8_t plain_len = 0;
    uint32_t plain_time = 0;  // when the plain hit occurred

    autotimer *p_at = bsp_obtain_timer(0);
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            for (int i = 0; i < em410x_protocols_size; i++) {
                const protocol *p = em410x_protocols[i];
                if (!p->decoder.feed(codecs[i], val)) {
                    continue;
                }
                uint8_t *pd = p->get_data(codecs[i]);
                if (p->tag_type == TAG_TYPE_EM410X_ELECTRA) {
                    // Accept the first Electra frame -- it wins over the held plain.
                    data[0] = p->tag_type >> 8;
                    data[1] = p->tag_type;
                    memcpy(data + 2, pd, p->data_size);
                    ok = true;
                } else if (p->tag_type == TAG_TYPE_EM410X_64) {
                    if (!plain_pending) {
                        plain_buf[0] = p->tag_type >> 8;
                        plain_buf[1] = p->tag_type;
                        memcpy(plain_buf + 2, pd, p->data_size);
                        plain_len = 2 + p->data_size;
                        plain_pending = true;
                        plain_time = p_at->time;
                    }
                } else {
                    // RF/32 or RF/16 -- Electra impossible, accept immediately.
                    data[0] = p->tag_type >> 8;
                    data[1] = p->tag_type;
                    memcpy(data + 2, pd, p->data_size);
                    ok = true;
                }
                break;
            }

            // No Electra frame within the hold window -> it is a plain tag.
            if (!ok && plain_pending && (p_at->time - plain_time) >= EM_ELECTRA_HOLD_MS) {
                memcpy(data, plain_buf, plain_len);
                ok = true;
            }
        }
    }

    // Timed out with only a held plain frame -> return it (don't drop the read).
    if (!ok && plain_pending) {
        memcpy(data, plain_buf, plain_len);
        ok = true;
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_em410x_hw();
    cb_free(&cb);

    for (size_t i = 0; i < em410x_protocols_size; i++) {
        em410x_protocols[i]->free(codecs[i]);
    }
    free(codecs);
    return ok;
}
