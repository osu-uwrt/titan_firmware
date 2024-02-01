#include "titan_debug_internal.h"

#include <malloc.h>
#include <string.h>

extern char __StackLimit;
extern char __StackTop;
extern char __StackBottom;
extern char __data_start__;
extern char end;

static void printMemoryUsage(FILE *fout, const char *label, uint32_t bytes) {
    fprintf(fout, "%s:", label);

    int label_size = strlen(label) + 1;
    int label_spacing = 40 - label_size;

    for (int i = 0; i < label_spacing; i++) {
        fputc(' ', fout);
    }

    if (bytes > 1024 * 1024) {
        fprintf(fout, "%.2f MB\n", (float) bytes / (1024 * 1024));
    }
    else if (bytes > 1024) {
        fprintf(fout, "%.2f KB\n", (float) bytes / 1024);
    }
    else {
        fprintf(fout, "%ld bytes\n", bytes);
    }
}

static int debug_global_cmd_memstats(__unused size_t argc, __unused const char *const *argv, FILE *fout) {
    struct mallinfo mi = mallinfo();

    uint32_t total_mem = (&__StackTop - &__data_start__);
    uint32_t heap_use = (&__StackLimit - &end);
    uint32_t stack_use = (&__StackTop - &__StackBottom);
    uint32_t static_use = (&end - &__data_start__);

    uint32_t reserved_mem = total_mem - heap_use;
    uint32_t total_used = mi.arena - mi.keepcost + reserved_mem;

    fprintf(fout, "Overall Memory Usage:                   %.2f%%\n",
            (((float) total_used) / ((float) total_mem)) * 100.0);
    printMemoryUsage(fout, "Total memory on chip", total_mem);
    printMemoryUsage(fout, "Static memory reserved", static_use);
    printMemoryUsage(fout, "Stack memory reserved", stack_use);
    printMemoryUsage(fout, "Total heap memory", heap_use);
    printMemoryUsage(fout, "Heap reserved [Used & Free] (arena)", mi.arena);
    fprintf(fout, "# of free chunks (ordblks):             %d blocks\n", mi.ordblks);
    printMemoryUsage(fout, "Total allocated (uordblks)", mi.uordblks);
    printMemoryUsage(fout, "Total free space (fordblks)", mi.fordblks);
    printMemoryUsage(fout, "Free space at top of heap (keepcost)", mi.keepcost);
    if (mi.hblks > 0 || mi.hblkhd > 0) {
        fprintf(fout, "# of mem-mapped regions (hblks):        %d blocks\n", mi.hblks);
        printMemoryUsage(fout, "Bytes in mem-mapped regions (hblkhd)", mi.hblkhd);
    }

    return 0;
}

void debug_register_global_remote_cmds(void) {
    // Register all the global commands
    debug_remote_cmd_register("memstats", "", "Shows memory usage statistics", debug_global_cmd_memstats);
}
