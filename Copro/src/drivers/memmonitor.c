#include <malloc.h>
#include <stdio.h>

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


	printf("\n==========Allocation Data==========\n");
	printf("Total heap memory available:           %zu\n", total_heap_mem);
	printf("Total memory on chip:                  %zu\n", total_mem);
	printf("Stack memory reserved:                 %zu\n", stack_mem);
	printf("Static memory reserved:                %zu\n", static_mem);
	printf("Memory Usage:                          %.1f%%\n", 100*(((double)(mi.arena - mi.keepcost + reserved_mem))/((double)total_mem)));
	printf("Total non-mmapped bytes (arena):       %zu\n", mi.arena);
           printf("# of free chunks (ordblks):            %zu\n", mi.ordblks);
           printf("# of mapped regions (hblks):           %zu\n", mi.hblks);
           printf("Bytes in mapped regions (hblkhd):      %zu\n", mi.hblkhd);
           printf("Total allocated space (uordblks):      %zu\n", mi.uordblks);
           printf("Total free space (fordblks):           %zu\n", mi.fordblks);
           printf("Topmost releasable block (keepcost):   %zu\n", mi.keepcost);
}