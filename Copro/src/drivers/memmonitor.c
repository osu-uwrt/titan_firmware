#include <malloc.h>
#include <stdio.h>

#include "basic_logger/logging.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "memmonitor"

extern char __StackLimit;
extern char __StackTop;
extern char __data_start__;
extern char end;

int memmonitor_get_total_use_percentage(void) {
    struct mallinfo mi = mallinfo();

    size_t total_heap_mem = (&__StackLimit - &end);
	size_t total_mem = (&__StackTop - &__data_start__);
    size_t reserved_mem = total_mem - total_heap_mem;

    return 100*(((double)(mi.arena - mi.keepcost + reserved_mem))/((double)total_mem));
}

void memmonitor_print_stats(void) {
    struct mallinfo mi = mallinfo();

	size_t total_heap_mem = (&__StackLimit - &end);
	size_t total_mem = (&__StackTop - &__data_start__);
	size_t stack_mem = (&__StackTop - &__StackLimit);
	size_t static_mem = (&end - &__data_start__);
	size_t reserved_mem = total_mem - total_heap_mem;


	LOG_INFO("\n==========Allocation Data==========");
	LOG_INFO("Total heap memory available:           %zu", total_heap_mem);
	LOG_INFO("Total memory on chip:                  %zu", total_mem);
	LOG_INFO("Stack memory reserved:                 %zu", stack_mem);
	LOG_INFO("Static memory reserved:                %zu", static_mem);
	LOG_INFO("Memory Usage:                          %.1f%%", 100*(((double)(mi.arena - mi.keepcost + reserved_mem))/((double)total_mem)));
	LOG_INFO("Total non-mmapped bytes (arena):       %zu", mi.arena);
	LOG_INFO("# of free chunks (ordblks):            %zu", mi.ordblks);
	LOG_INFO("# of mapped regions (hblks):           %zu", mi.hblks);
	LOG_INFO("Bytes in mapped regions (hblkhd):      %zu", mi.hblkhd);
	LOG_INFO("Total allocated space (uordblks):      %zu", mi.uordblks);
	LOG_INFO("Total free space (fordblks):           %zu", mi.fordblks);
	LOG_INFO("Topmost releasable block (keepcost):   %zu", mi.keepcost);
}