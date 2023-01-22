#ifndef MICRO_ROS_PICO__CUSTOM_ALLOCATOR_H
#define MICRO_ROS_PICO__CUSTOM_ALLOCATOR_H

#include "rcl/allocator.h"

/**
 * @file custom_allocator.h
 *
 * A custom RCL allocator to keep track of all memory allocated by RCL.
 *
 * This implements a method which can be used to free all memory allocated by RCL. This is useful for destroying all
 * ROS contexts without needing to call cleanup functions which will block for *an undetermined amount of time*.
 * (Although there is a preprocessor definition which controls how long the destruction blocks for, different cleanup
 * functions will use this timeout multiple times, with some like action servers calling it upwards of 10x the set
 * timeout). To avoid waiting for timeouts which *will* occur after an agent disconnect, this frees all memory as
 * there is no need to gracefully disconnect from a non-existent agent.
 *
 * NOTE: After calling `custom_allocator_free_all` do NOT call ANY rcl functions on objects which were initialized
 * using this allocator. Things WILL break in very strange and difficult to debug ways. This includes the _fini and
 * destruction functions. The `custom_allocator_free_all` should be the LAST call used on those objects. After this,
 * zero out all object handles and you must reinitialize RCL from scratch, including rcl support.
 *
 * This method to reset RCL state without calling the destructors should work, as of Jan 2023. One way that this would
 * have issues is if the RCL or any other libraries which save the state data from RCL. However all static allocations
 * in the libmicroros.a file have been checked, and none appear to save state that would not be overwritten during
 * RCL initialization. However if code is updated which has static allocations, memory leaks or other issues may occur
 * during the re-initialization of RCL. Proceed with caution.
 */

/**
 * @brief Returns an rcl allocator object which allocates memory using this custom allocator.
 *
 * @return rcl_allocator_t Allocator using custom allocator calls
 */
rcl_allocator_t custom_allocator_get(void);

/**
 * @brief Configures RCL to make this allocator the default for all subsequent rcl_get_default_allocator calls.
 * Note that this function must be called before rcl_get_default_allocator or else the old allocator will be used.
 */
void custom_allocator_set_default(void);

/**
 * @brief Frees all memory allocated using the custom_allocator.
 * All objects that use this allocator must be invalidated after this call!
 */
void custom_allocator_free_all(void);

#endif