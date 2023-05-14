#include "titan/canmore.h"

static uint8_t reg_mapped_server_handle_single_write(reg_mapped_server_inst_t *inst, struct reg_mapped_write_request *req) {
    if (req->page >= inst->num_pages) {
        return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
    }

    const reg_mapped_server_page_def_t *page = &inst->page_array[req->page];

    if (page->page_type == PAGE_TYPE_MEMORY_MAPPED_WORD) {
        if (req->offset >= page->type.mem_mapped_word.num_words) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }

        if (page->type.mem_mapped_word.perm != REGISTER_PERM_READ_WRITE && page->type.mem_mapped_word.perm != REGISTER_PERM_WRITE_ONLY) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
        }

        page->type.mem_mapped_word.base_addr[req->offset] = req->data;

        return REG_MAPPED_RESULT_SUCCESSFUL;
    }
    if (page->page_type == PAGE_TYPE_MEMORY_MAPPED_BYTE) {
        const unsigned int word_size = sizeof(req->data);

        // If the request is completely outside of the region, return invalid address
        if (req->offset * word_size >= page->type.mem_mapped_byte.size) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }

        if (page->type.mem_mapped_byte.perm != REGISTER_PERM_READ_WRITE && page->type.mem_mapped_byte.perm != REGISTER_PERM_WRITE_ONLY) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
        }

        // Write the word
        // Note that this is done to ensure that it can be written if byte buffer length isn't divisible by 4
        // Also necessary in the event the buffer is not word aligned
        int extra_bytes = ((req->offset * word_size) + word_size) - page->type.mem_mapped_byte.size;
        if (extra_bytes < 0) extra_bytes = 0;
        uint32_t write_word = req->data;
        size_t write_offset = req->offset * word_size;
        for (unsigned int i = 0; i < (word_size - extra_bytes); i++) {
            page->type.mem_mapped_byte.base_addr[write_offset++] = write_word;
            write_word >>= 8;
        }

        return REG_MAPPED_RESULT_SUCCESSFUL;
    }
    else if (page->page_type == PAGE_TYPE_REGISTER_MAPPED) {
        if (req->offset >= page->type.reg_mapped.num_registers) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }

        const reg_mapped_server_register_def_t *reg = &page->type.reg_mapped.reg_array[req->offset];

        if (reg->reg_type == REGISTER_TYPE_MEMORY) {
            if (reg->type.memory.perm != REGISTER_PERM_READ_WRITE && reg->type.memory.perm != REGISTER_PERM_WRITE_ONLY) {
                return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
            }

            *reg->type.memory.reg_ptr = req->data;

            return REG_MAPPED_RESULT_SUCCESSFUL;
        }
        else if (reg->reg_type == REGISTER_TYPE_EXEC) {
            if (reg->type.exec.perm != REGISTER_PERM_READ_WRITE && reg->type.exec.perm != REGISTER_PERM_WRITE_ONLY) {
                return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
            }

            uint32_t data = req->data;
            if (reg->type.exec.callback(reg, true, &data)) {
                return REG_MAPPED_RESULT_SUCCESSFUL;
            } else {
                return REG_MAPPED_RESULT_INVALID_DATA;
            }
        }
        else {
            // register type is REGISTER_TYPE_UNIMPLEMENTED
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }
    }
    else {
        // page_type is PAGE_TYPE_UNIMPLEMENTED
        return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
    }
}

static uint8_t reg_mapped_server_handle_single_read(reg_mapped_server_inst_t *inst, struct reg_mapped_read_request *req, uint32_t *data_out) {
    if (req->page >= inst->num_pages) {
        return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
    }

    const reg_mapped_server_page_def_t *page = &inst->page_array[req->page];

    if (page->page_type == PAGE_TYPE_MEMORY_MAPPED_WORD) {
        if (req->offset >= page->type.mem_mapped_word.num_words) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }

        if (page->type.mem_mapped_word.perm != REGISTER_PERM_READ_WRITE && page->type.mem_mapped_word.perm != REGISTER_PERM_READ_ONLY) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
        }

        *data_out = page->type.mem_mapped_word.base_addr[req->offset];

        return REG_MAPPED_RESULT_SUCCESSFUL;
    }
    if (page->page_type == PAGE_TYPE_MEMORY_MAPPED_BYTE) {
        const unsigned int word_size = sizeof(*data_out);

        // If the request is completely outside of the region, return invalid address
        if (req->offset * word_size >= page->type.mem_mapped_byte.size) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }

        if (page->type.mem_mapped_byte.perm != REGISTER_PERM_READ_WRITE && page->type.mem_mapped_byte.perm != REGISTER_PERM_READ_ONLY) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
        }

        // Handle unaligned buffers or reads at the end of the buffer
        int required_padding = ((req->offset * word_size) + word_size) - page->type.mem_mapped_byte.size;
        if (required_padding < 0) required_padding = 0;
        uint32_t partial_read = 0;
        size_t read_offset = req->offset * word_size;
        for (unsigned int i = 0; i < (word_size - required_padding); i++) {
            partial_read |= page->type.mem_mapped_byte.base_addr[read_offset++] << (8*i);
        }

        *data_out = partial_read;

        return REG_MAPPED_RESULT_SUCCESSFUL;
    }
    else if (page->page_type == PAGE_TYPE_REGISTER_MAPPED) {
        if (req->offset >= page->type.reg_mapped.num_registers) {
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }

        const reg_mapped_server_register_def_t *reg = &page->type.reg_mapped.reg_array[req->offset];

        if (reg->reg_type == REGISTER_TYPE_MEMORY) {
            if (reg->type.memory.perm != REGISTER_PERM_READ_WRITE && reg->type.memory.perm != REGISTER_PERM_READ_ONLY) {
                return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
            }

            *data_out = *reg->type.memory.reg_ptr;

            return REG_MAPPED_RESULT_SUCCESSFUL;
        }
        else if (reg->reg_type == REGISTER_TYPE_EXEC) {
            if (reg->type.exec.perm != REGISTER_PERM_READ_WRITE && reg->type.exec.perm != REGISTER_PERM_READ_ONLY) {
                return REG_MAPPED_RESULT_INVALID_REGISTER_MODE;
            }

            if (reg->type.exec.callback(reg, false, data_out)) {
                return REG_MAPPED_RESULT_SUCCESSFUL;
            } else {
                return REG_MAPPED_RESULT_INVALID_DATA;
            }
        }
        else {
            // register type is REGISTER_TYPE_UNIMPLEMENTED
            return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
        }
    }
    else {
        // page_type is PAGE_TYPE_UNIMPLEMENTED
        return REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS;
    }
}

void reg_mapped_server_handle_request(reg_mapped_server_inst_t *inst, uint8_t *msg, size_t len) {
    if (len < 1) {
        // If request is empty, just return
        // There isn't enough data to determine what format the error repsonse should be sent in
        return;
    }

    // Decode flags
    union reg_mapped_request_flags flags = {.data = msg[0]};
    bool request_type_write = (flags.f.write ? true : false);
    bool request_type_bulk = (flags.f.bulk_req ? true : false);
    bool request_type_bulk_end = (flags.f.bulk_end ? true : false);
    bool request_type_multiword = (flags.f.multiword ? true : false);

    // Initialize decode variables
    reg_mapped_request_t *req = (reg_mapped_request_t *)msg;
    uint8_t result_code = REG_MAPPED_RESULT_MALFORMED_REQUEST;
    uint32_t read_data = 0;
    reg_mapped_response_t response = {0};
    size_t response_size;

    // Check message length
    size_t expected_msg_len = (request_type_write ? sizeof(req->write_pkt) : sizeof(req->read_pkt));
    if (len != expected_msg_len) {
        result_code = REG_MAPPED_RESULT_MALFORMED_REQUEST;
        goto finish_request;
    }

    if (inst->in_bulk_request && !request_type_bulk) {
        result_code = REG_MAPPED_RESULT_BULK_REQUEST_SEQ_ERROR;
        goto finish_request;
    }

    if (request_type_multiword && request_type_bulk) {
        // Bulk and multiword requests are mutually exclusive
        result_code = REG_MAPPED_RESULT_MALFORMED_REQUEST;
        goto finish_request;
    }

    if (request_type_bulk && !request_type_write) {
        // Bulk requests are only supported on write requests
        result_code = REG_MAPPED_RESULT_MALFORMED_REQUEST;
        goto finish_request;
    }

    if (request_type_bulk_end && !request_type_bulk) {
        // Bulk end can only be sent if its a bulk request
        result_code = REG_MAPPED_RESULT_MALFORMED_REQUEST;
        goto finish_request;
    }

    if (flags.f.mode != inst->control_interface_mode) {
        result_code = REG_MAPPED_RESULT_INVALID_MODE;
        goto finish_request;
    }

    // Bulk request handling
    if (request_type_bulk) {
        if (!inst->in_bulk_request) {
            // Handle starting new bulk request
            if (req->write_pkt.count != 0) {
                // Bulk requests must start with sequence number 0
                result_code = REG_MAPPED_RESULT_BULK_REQUEST_SEQ_ERROR;
                goto finish_request;
            }

            inst->in_bulk_request = true;
            inst->bulk_error_code = 0;
            inst->bulk_last_seq_num = 0;
        }
        else {
            // Check bulk request sequence number (but only if an error is not set as this will overwrite the last sequence number with the error)
            if (inst->bulk_error_code == 0) {
                inst->bulk_last_seq_num++;

                if (req->write_pkt.count != inst->bulk_last_seq_num) {
                    result_code = REG_MAPPED_RESULT_BULK_REQUEST_SEQ_ERROR;
                    goto finish_request;
                }
            }
        }
    }

    // TODO: Implement multiword mode

    // If normal request or a bulk request without an error set, perform the transfer
    if (!inst->in_bulk_request || inst->bulk_error_code == 0) {
        if (request_type_write) {
            result_code = reg_mapped_server_handle_single_write(inst, &req->write_pkt);
        } else {
            result_code = reg_mapped_server_handle_single_read(inst, &req->read_pkt, &read_data);
        }
    }


finish_request:
    if (request_type_bulk) {
        // Handle last transfer in request
        if (request_type_bulk_end) {
            if (inst->bulk_error_code != 0) {
                response.write_bulk_pkt.result = inst->bulk_error_code;
            } else {
                response.write_bulk_pkt.result = result_code;
            }
            response.write_bulk_pkt.seq_no = inst->bulk_last_seq_num;
            response_size = sizeof(response.write_bulk_pkt);

            // Send response and exit bulk request mode
            inst->tx_func(response.data, response_size);
            inst->in_bulk_request = false;
        }
        // If it's not the last transfer, store error if occurs in request (but only if it hasn't already errored)
        else if (inst->bulk_error_code == 0) {
            inst->bulk_error_code = result_code;
        }
    } else {
        if (request_type_write) {
            response.write_pkt.result = result_code;
            response_size = sizeof(response.write_pkt);
        } else {
            response.read_pkt.result = result_code;
            response.read_pkt.data = read_data;
            response_size = sizeof(response.read_pkt);
        }

        inst->tx_func(response.data, response_size);
    }
}
