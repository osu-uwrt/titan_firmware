#include "canmore_titan/reg_mapped_client.h"

int reg_mapped_client_read_register(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset, uint32_t *data_out) {
    reg_mapped_request_t req = {.read_pkt = {
        .flags = {.f = {.write = false}},
        .count = 0,
        .page = page,
        .offset = offset
    }};

    if (!cfg->clear_rx_func(cfg->arg)) {
        return REG_MAPPED_CLIENT_RESULT_RX_CLEAR_FAIL;
    }

    if (!cfg->tx_func(req.data, sizeof(req.read_pkt), cfg->arg)) {
        return REG_MAPPED_CLIENT_RESULT_TX_FAIL;
    }

    reg_mapped_response_t resp;
    if (!cfg->rx_func(resp.data, sizeof(resp.read_pkt), cfg->timeout_ms, cfg->arg)) {
        return REG_MAPPED_CLIENT_RESULT_RX_FAIL;
    }

    if (resp.read_pkt.result != REG_MAPPED_RESULT_SUCCESSFUL) {
        return resp.read_pkt.result;
    }

    *data_out = resp.read_pkt.data;
    return REG_MAPPED_RESULT_SUCCESSFUL;
}

int reg_mapped_client_write_register(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset, uint32_t data) {
    reg_mapped_request_t req = {.write_pkt = {
        .flags = {.f = {.write = true}},
        .count = 0,
        .page = page,
        .offset = offset,
        .data = data
    }};

    if (!cfg->clear_rx_func(cfg->arg)) {
        return REG_MAPPED_CLIENT_RESULT_RX_CLEAR_FAIL;
    }

    if (!cfg->tx_func(req.data, sizeof(req.write_pkt), cfg->arg)) {
        return REG_MAPPED_CLIENT_RESULT_TX_FAIL;
    }

    reg_mapped_response_t resp;
    if (!cfg->rx_func(resp.data, sizeof(resp.write_pkt), cfg->timeout_ms, cfg->arg)) {
        return REG_MAPPED_CLIENT_RESULT_RX_FAIL;
    }

    return resp.write_pkt.result;
}

int reg_mapped_client_write_array(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset_start,
                                    const uint32_t *data_array, uint8_t num_words) {
    // Make sure we don't cross the page boundary
    if (((unsigned int)offset_start) + num_words > 0x100) {
        return REG_MAPPED_CLIENT_RESULT_INVALID_ARG;
    }

    if (!cfg->clear_rx_func(cfg->arg)) {
        return REG_MAPPED_CLIENT_RESULT_RX_CLEAR_FAIL;
    }

    if (cfg->transfer_mode == TRNASFER_MODE_BULK) {
        reg_mapped_request_t req = {.write_pkt = {
            .flags = {.f = {.write = true, .bulk_req = true}},
            .count = 0,
            .page = page,
            .offset = offset_start,
        }};

        // Loop until no words left to read
        while (num_words--) {
            if (!num_words) {
                // If last request, set bulk_end to true
                req.write_pkt.flags.f.bulk_end = true;
            }

            req.write_pkt.data = *data_array++;

            if (!cfg->tx_func(req.data, sizeof(req.write_pkt), cfg->arg)) {
                return REG_MAPPED_CLIENT_RESULT_TX_FAIL;
            }

            req.write_pkt.count++;
            req.write_pkt.offset++;
        }

        // Finally read the bulk transfer result
        reg_mapped_response_t resp;
        if (!cfg->rx_func(resp.data, sizeof(resp.write_bulk_pkt), cfg->timeout_ms, cfg->arg)) {
            return REG_MAPPED_CLIENT_RESULT_RX_FAIL;
        }

        // Check for errors
        if (resp.write_bulk_pkt.result != REG_MAPPED_RESULT_SUCCESSFUL) {
            return resp.write_bulk_pkt.result;
        }

        if (resp.write_bulk_pkt.seq_no != (req.write_pkt.count - 1)) {
            return REG_MAPPED_CLIENT_RESULT_INVALID_BULK_COUNT;
        }
    }
    else {
        // TRANSFER_MODE_SINGLE

        uint8_t offset = offset_start;
        for (int i = 0; i < num_words; i++) {
            int ret = reg_mapped_client_write_register(cfg, page, offset++, *data_array++);
            if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
                return ret;
            }
        }
    }

    return REG_MAPPED_RESULT_SUCCESSFUL;
}

int reg_mapped_client_read_array(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset_start,
                                    uint32_t *data_array, uint8_t num_words) {
    // Make sure we don't cross the page boundary
    if (((unsigned int)offset_start) + num_words > 0x100) {
        return REG_MAPPED_CLIENT_RESULT_INVALID_ARG;
    }

    // TRANSFER_MODE_BULK does not support optimized reads
    // Fallback to single transfers
    uint8_t offset = offset_start;
    for (int i = 0; i < num_words; i++) {
        int ret = reg_mapped_client_read_register(cfg, page, offset++, data_array++);
        if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
            return ret;
        }
    }

    return REG_MAPPED_RESULT_SUCCESSFUL;
}