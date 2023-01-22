#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#include "rcl/allocator.h"

#include "micro_ros_pico/custom_allocator.h"

typedef struct custom_allocation_block {
    struct custom_allocation_block * prev;
    struct custom_allocation_block * next;
    void* contents[];
} alloc_block_t;

alloc_block_t custom_allocation_list = {
    .next = &custom_allocation_list,
    .prev = &custom_allocation_list
};

static void append_allocation_block(alloc_block_t *block) {
    if (!block)
        return;

    // Ensure that the linked list is circular
    assert(custom_allocation_list.prev->next == &custom_allocation_list);

    // Add block into end of list
    block->prev = custom_allocation_list.prev;
    block->next = &custom_allocation_list;
    custom_allocation_list.prev->next = block;
    custom_allocation_list.prev = block;
}

static void unlink_allocation_block(alloc_block_t *block) {
    if (!block)
        return;

    if (block->next) {
        block->next->prev = block->prev;
    }

    if (block->prev) {
        block->prev->next = block->next;
    }
}


void custom_allocator_free_all(void) {
    while (custom_allocation_list.prev != &custom_allocation_list) {
        alloc_block_t *block = custom_allocation_list.prev;
        unlink_allocation_block(block);
        free(block);
    }

    // Ensure that the list is still circular
    assert(custom_allocation_list.next == &custom_allocation_list);
}

#define pointer_to_alloc_block(ptr) ((alloc_block_t*) (((uintptr_t)(ptr)) - offsetof(alloc_block_t, contents)))



static void * custom_allocator_allocate(size_t size, void * state) {
    printf("ALLOC\n");
    (void*) state;

    alloc_block_t* alloc_block = malloc(sizeof(alloc_block_t) + size);
    void* pointer = NULL;

    if (alloc_block) {
        append_allocation_block(alloc_block);
        pointer = &alloc_block->contents;

        printf(" - Allocated 0x%p with size 0x%X\n", pointer, size);
    }

    return pointer;
}

static void custom_allocator_deallocate(void * pointer, void * state)
{
    printf("FREE\n");
    (void*) state;

    if (pointer) {
        printf(" - Deallocating 0x%p\n", pointer);
        alloc_block_t *alloc_block = pointer_to_alloc_block(pointer);
        unlink_allocation_block(alloc_block);

        free(alloc_block);
    }
}

static void * custom_allocator_reallocate(void * pointer, size_t size, void * state)
{
    printf("REALLOC\n");
    (void*) state;

    if (pointer) {
        alloc_block_t *alloc_block = pointer_to_alloc_block(pointer);
        unlink_allocation_block(alloc_block);
        pointer = alloc_block;
    }

    alloc_block_t* new_alloc_block = realloc(pointer, sizeof(alloc_block_t) + size);

    void* new_pointer = NULL;
    if (new_alloc_block) {
        append_allocation_block(new_alloc_block);
        new_pointer = &new_alloc_block->contents;

        printf(" - Reallocating 0x%p to 0x%p with size 0x%X\n", pointer, new_pointer, size);
    }

    return new_pointer;
}

static void * custom_allocator_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
    printf("CALLOC\n");
    (void*) state;

    alloc_block_t* alloc_block = calloc(1, (size_of_element * number_of_elements) + sizeof(alloc_block_t));

    void* pointer = NULL;

    if (alloc_block) {
        append_allocation_block(alloc_block);
        pointer = &alloc_block->contents;

        printf(" - Zero allocated 0x%p with 0x%X elements with size 0x%X\n", pointer, number_of_elements, size_of_element);
    }

    return pointer;
}


static rcl_allocator_t default_allocator = {
    .allocate = custom_allocator_allocate,
    .deallocate = custom_allocator_deallocate,
    .reallocate = custom_allocator_reallocate,
    .zero_allocate = custom_allocator_zero_allocate,
    .state = NULL,
};

rcl_allocator_t custom_allocator_get(void) {
    return default_allocator;
}

void custom_allocator_set_default(void) {
    (void)(!rcutils_set_default_allocator(&default_allocator));
}
