#include "titan_debug_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if TITAN_SAFETY

// ========================================
// Binary Tree Code
// ========================================

// Binary Search Tree representation
typedef struct cmd_handler {
    const char *name;
    const char *usage;
    const char *help_msg;
    debug_remote_cmd_cb callback;
    struct cmd_handler *parent;
    struct cmd_handler *left_child;   // Child < current node
    struct cmd_handler *right_child;  // Child > current node
} cmd_handler_t;

static size_t debug_remote_cmd_handler_count = 0;
static cmd_handler_t *debug_remote_cmd_handler_bst = NULL;

void debug_remote_cmd_register(const char *name, const char *usage, const char *help_msg,
                               debug_remote_cmd_cb callback) {
    // Create new command handler object
    cmd_handler_t *new_handler = (cmd_handler_t *) malloc(sizeof(cmd_handler_t));
    new_handler->name = name;
    if (usage)
        new_handler->usage = usage;
    else
        new_handler->usage = "";
    if (help_msg)
        new_handler->help_msg = help_msg;
    else
        new_handler->help_msg = "";
    new_handler->callback = callback;
    new_handler->parent = NULL;
    new_handler->left_child = NULL;
    new_handler->right_child = NULL;

    // Search for location to put the new handler in the tree
    cmd_handler_t *parent_handler = NULL;
    cmd_handler_t **handler_ptr = &debug_remote_cmd_handler_bst;
    while (*handler_ptr != NULL) {
        parent_handler = *handler_ptr;
        int result = strcmp(name, parent_handler->name);
        if (result > 0) {
            handler_ptr = &(parent_handler->right_child);
        }
        else if (result < 0) {
            handler_ptr = &(parent_handler->left_child);
        }
        else {
            panic("Attempting to register already registered command '%s'", name);
        }
    }

    // Store the new handler at that location
    new_handler->parent = parent_handler;
    *handler_ptr = new_handler;
    debug_remote_cmd_handler_count++;
}

static cmd_handler_t *debug_remote_cmd_handler_by_idx(size_t idx) {
    if (idx >= debug_remote_cmd_handler_count) {
        return NULL;
    }

    // Perform inorder traversl of BST
    // I'd like to meme on the horrible efficiency of it, but it's only a bit slower than a linked list
    // And this function should be seldom used except for pulling help info, and the speed of the BST is worth
    // it when executing every command.

    // To go in order, we need to always to go the left most element if possible
    // If no left-most element is available, then we return ourself
    // Then, we go to the right element and try to go left until we can't anymore
    // If there is no right element, we go up to the parent

    // Also, this code is extra fun since we want to do it non-recursively to avoid overflowing the small stack
    // This code here should work for very large BSTs, and has the penalty of needing an extra ptr comparison rather
    // than knowing where you are in the stack.

    cmd_handler_t *ptr = debug_remote_cmd_handler_bst;
    assert(ptr != NULL);  // Should be caught by bounds check above

    enum search_state { GOING_LEFT, GOING_RIGHT, GOING_UP } state = GOING_LEFT;

    for (size_t i = 0; i <= idx; i++) {
        cmd_handler_t *last_child = NULL;
        bool found = false;

        while (!found) {
            if (state == GOING_LEFT) {
                // See if we have a left child
                if (ptr->left_child) {
                    // If so, keep going left
                    ptr = ptr->left_child;
                }
                else {
                    // No left child, then return ourself, next time we'll start going right
                    state = GOING_RIGHT;
                    found = true;
                }
            }
            else if (state == GOING_RIGHT) {
                // We need to go right, see if we have a right child
                if (ptr->right_child) {
                    // Found right child, go to it, and try to begin traversing to the bottom left of that tree
                    state = GOING_LEFT;
                    ptr = ptr->right_child;
                }
                else {
                    // No right child found, we need to go up, enter going up state
                    state = GOING_UP;
                    last_child = NULL;  // Set to NULL to force at least one traversal up
                }
            }
            else if (state == GOING_UP) {
                if (last_child && last_child == ptr->left_child) {
                    // We came back from the left child
                    // We need to return ourself before continuing right
                    // Ptr is already set to us, just update the state then break out to increment for loop
                    state = GOING_RIGHT;
                    found = true;
                }
                else {
                    // If not, then either last_child is NULL (meaning it's our first traversal up, or we were the right
                    // child). In this case, keep traversing back up until we find the next node we were the left child.
                    last_child = ptr;
                    ptr = ptr->parent;

                    // If ptr is NULL, that means we were the right child all the way to the top of the tree, and we've
                    // hit the end. The requested index was too high
                    assert(ptr != NULL);  // This should be caught by the bounds check above
                    if (ptr == NULL) {
                        // We've hit the top of the tree, and can't backtrack any more
                        return NULL;
                    }
                }
            }
        }
    }

    return ptr;
}

static cmd_handler_t *debug_remote_cmd_get_handler(const char *cmd_name) {
    cmd_handler_t *handler = debug_remote_cmd_handler_bst;

    // Loop through tree until a NULL node is hit, searching for the name
    while (handler != NULL) {
        int result = strcmp(cmd_name, handler->name);
        if (result > 0) {
            handler = handler->right_child;
        }
        else if (result < 0) {
            handler = handler->left_child;
        }
        else {
            // Handler matches, return it
            return handler;
        }
    }

    // We reached the end of the tree before finding the child
    return NULL;
}

// ========================================
// Help Command (always present)
// ========================================

static int debug_remote_cmd_help_handler(size_t argc, const char *const *argv, FILE *fout) {
    if (argc == 1) {
        fputs("=====List of Commands=====\n", fout);
        for (size_t i = 0; i < debug_remote_cmd_handler_count; i++) {
            cmd_handler_t *handler = debug_remote_cmd_handler_by_idx(i);
            // This shouldn't happen, but in case it does (e.g. memory corruption), handle it deterministically
            if (!handler)
                return -1;

            fprintf(fout, " %s\n", handler->name);
        }
        return 0;
    }

    // We have arguments, begin processing them
    if (argc == 2 && !strcmp(argv[1], "-n")) {
        // Special flag to get number of commands to allow auto-discovery of help in canmore CLI
        fprintf(fout, "%d", debug_remote_cmd_handler_count);
        return 0;
    }
    else if (argc == 3 && !strcmp(argv[1], "-i")) {
        // Special flag to get help for a specific index, also for command autodiscovery
        int cmd_idx = atoi(argv[2]);
        cmd_handler_t *handler = debug_remote_cmd_handler_by_idx(cmd_idx);
        if (handler == NULL) {
            // This is handled by the canmore CLI code, no need to give a message, just send an error status code
            return 1;
        }

        // Format the response in a way that can be easily decoded by canmore CLI
        // It won't be pretty, but we can easily split by newline
        fprintf(fout, "%s\n%s\n%s", handler->name, handler->usage, handler->help_msg);
        return 0;
    }
    else if (argc == 2) {
        // We have one argument, and there's no special flags, look up the command name
        cmd_handler_t *handler = debug_remote_cmd_get_handler(argv[1]);

        if (handler == NULL) {
            fputs("No command found by name", fout);
            return 1;
        }

        // We successfully looked up the command, give a pretty help message
        fprintf(fout, "%s %s\n\t%s", handler->name, handler->usage, handler->help_msg);
        return 0;
    }
    else {
        // Don't know how to handle the command, give error
        fputs("Unexpected arguments passed\nUsage: help [optional command name]", fout);
        return 1;
    }
}

// ========================================
// Command Parsing Code
// ========================================

static size_t debug_remote_cmd_count_args(const char *args) {
    size_t count = 0;

    size_t last_arg_len;
    do {
        last_arg_len = strlen(args);
        args += last_arg_len + 1;
        count++;
    } while (last_arg_len != 0);

    return count - 1;
}

void debug_remote_cmd_init(void) {
    // Register the help command
    debug_remote_cmd_register("help", "[optional command]",
                              "Shows remote command help\n"
                              "If command is provided, then it shows the help for that command.\n"
                              "If no command is provided, a list of commands is given.",
                              debug_remote_cmd_help_handler);
}

int debug_remote_cmd_handle(const char *args, size_t resp_size, char *resp) {
    // Compute the number of args
    size_t argc = debug_remote_cmd_count_args(args);

    if (argc == 0) {
        strlcpy(resp, "Cannot execute null command", resp_size);
        return 127;
    }

    // The command name can be assumed to args, since args is a null terminated string
    const char *cmd_name = args;
    cmd_handler_t *handler = debug_remote_cmd_get_handler(cmd_name);
    if (handler == NULL) {
        snprintf(resp, resp_size, "Unknown Remote Command: '%s'", cmd_name);
        resp[resp_size - 1] = 0;
        return 127;
    }

    // Create the argv array using the argc count
    const char **argv = malloc(sizeof(const char *) * (argc + 1));
    const char *argv_search_ptr = args;
    for (size_t argv_idx = 0; argv_idx < argc; argv_idx++) {
        argv[argv_idx] = argv_search_ptr;
        argv_search_ptr += strlen(argv_search_ptr) + 1;
    }
    argv[argc] = NULL;

    // Create the FILE backed by the response memory to make it easy for the callback to write data back out
    FILE *resp_out = fmemopen(resp, resp_size, "w");

    // Execute the command
    int returncode = handler->callback(argc, argv, resp_out);

    // Release the memory we allocated
    free(argv);
    fclose(resp_out);

    // Return the return code
    return returncode;
}

#endif
