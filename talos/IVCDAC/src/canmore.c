#include "canmore.h"

#include "hardware/clocks.h"

#include <string.h>

static ivc_context_t *ivc_ctx;

static void get_is_writing_cb(size_t argc, const char *const *argv, FILE *fout) {
    bool is_writing = ivc_ctx->tx.flags.is_writing;
    fprintf(fout, "is_writing: [%s]\n", is_writing ? "true" : "false");
}

static void tx_queue_empty_cb(size_t argc, const char *const *argv, FILE *fout) {
    fprintf(fout, "empty: [%s]\n", tx_queue_empty() ? "true" : "false");
}

static void alarms_cb(size_t argc, const char *const *argv, FILE *fout) {
    // fprintf(fout, "num_available: [%d]", alarm_pool_remaining_alarm_count(alarm_pool_get_default()));
}

static void receiving_packet_cb(size_t argc, const char *const *argv, FILE *fout) {
    bool receiving = ivc_ctx->rx.flags.receiving_packet;
    fprintf(fout, "receiving_packet: [%s]\n", receiving ? "true" : "false");
}

// static void set_pinger_mode_cb(size_t argc, const char *const *argv, FILE *fout) {
//     ivc_ctx->rx.flags.is_pinger_mode = true;
// }

static void set_comms_mode_cb(size_t argc, const char *const *argv, FILE *fout) {
    ivc_ctx->rx.flags.is_pinger_mode = false;
}

static void set_pinger_mode_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 1) {
        fprintf(fout, "ERROR: arg must be exactly one of [ 20 | 25 | 30 | 35 | 40 ]\n");
        return;
    }

    if (strcmp(argv[0], "20") == 0) {
        ivc_ctx->rx.pinger_mode = FREQ_20KHZ;
    }
    else if (strcmp(argv[0], "25") == 0) {
        ivc_ctx->rx.pinger_mode = FREQ_25KHZ;
    }
    else if (strcmp(argv[0], "30") == 0) {
        ivc_ctx->rx.pinger_mode = FREQ_30KHZ;
    }
    else if (strcmp(argv[0], "35") == 0) {
        ivc_ctx->rx.pinger_mode = FREQ_35KHZ;
    }
    else if (strcmp(argv[0], "40") == 0) {
        ivc_ctx->rx.pinger_mode = FREQ_40KHZ;
    }
    else {
        fprintf(fout, "ERROR: <%s> is unrecognized\n", argv[0]);
        return;
    }
}

void register_canmore_commands(ivc_context_t *ctx) {
    ivc_ctx = ctx;

    debug_remote_cmd_register("write_lock", "", "get the status of the write lock bool", get_is_writing_cb);
    debug_remote_cmd_register("tx_queue_empty", "", "see if the tx queue is empty", tx_queue_empty_cb);
    debug_remote_cmd_register("receiving_packet", "", "check if the receiving packet flag is set", receiving_packet_cb);
    // debug_remote_cmd_register("set_pinger_mode", "", "set IVC to listen for the pinger", set_pinger_mode_cb);
    debug_remote_cmd_register("set_comms_mode", "", "set IVC to listen for comms signals", set_comms_mode_cb);
    debug_remote_cmd_register("set_pinger_mode", "[pinger freq in khz (20, 25, 30, 35, 40)]",
                              "will set ivc to listen for that frequency\n", set_pinger_mode_cb);
    // debug_remote_cmd_register("alarm_pool_available_count", "", "check the available number of hardware alarms")
}
