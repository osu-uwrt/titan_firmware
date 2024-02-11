#pragma once

#include "CANSocket.hpp"

#include "titan/canmore.h"

#include <functional>
#include <list>
#include <map>
#include <memory>
#include <span>
#include <vector>

namespace Canmore {

// ========================================
// Function Typedefs
// ========================================

/**
 * @brief Callback for register writes
 *
 * @param addr The complete 16 bit address (upper 8 bits: page, lower 8 bits: offset)
 * @param is_write True if register was written, False if data is read
 * @param data_ptr Pointer containing data written if write, pointer to store read data if read
 * @return True on successful access, false on failure (causes server to transmit error response)
 */
typedef std::function<bool(uint16_t addr, bool is_write, uint32_t *data_ptr)> RegisterCB;

/**
 * @brief Access permission for all registers/pages
 */
typedef reg_mapped_server_register_permissions AccessPerm;

// ========================================
// Register Mapped Page Class
// ========================================

/**
 * @brief Defines a register mapped page.
 *
 * @note This can only be modified after calling create, but before transferring to the RegMappedServer.
 * After ownership is transferred to the RegMappedServer, the uniquie_ptr is cleared, and the register page can no
 * longer be modified.
 */
class RegMappedRegisterPage {
    friend class RegMappedServer;

public:
    /**
     * @brief Creates a new register page. This object can then be modified using the functions listed below
     *
     * @return std::unique_ptr<RegMappedRegisterPage> The new page that can be modified
     */
    static std::unique_ptr<RegMappedRegisterPage> create() {
        return std::unique_ptr<RegMappedRegisterPage>(new RegMappedRegisterPage());
    }

    /**
     * @brief Defines a new memory based register.
     * All reads/writes are transparently performed on the provided pointer without application intervention.
     *
     * @param reg_num The register offset number in the page for this register
     * @param perm The access permissions for this register
     * @param ptr A pointer that the data will be read from/written to
     */
    void addMemoryRegister(uint8_t reg_num, AccessPerm perm, uint32_t *ptr);

    /**
     * @brief Defines a new constant memory based register.
     * This register will be read only, and will always read the provided constant value
     *
     * @param reg_num The register offset number in the page for this register
     * @param val The constant value for this regsiter to hold
     */
    void addConstRegister(uint8_t reg_num, uint32_t val);

    /**
     * @brief Creates a new callback based register.
     * The callback is called on any register read/write.
     *
     * @param reg_num The register offset number in the page for this register
     * @param perm The access permission for this register. The callback will only fire if perm permits the read/write.
     *             e.g. if perm is REGISTER_PERM_READ_ONLY, but a client sends a write packet, callback will not fire.
     * @param callback The callback for all register reads/writes.
     */
    void addCallbackRegister(uint8_t reg_num, AccessPerm perm, RegisterCB callback);

protected:
    /**
     * @brief Protected function for use by the RegMappedServer to take ownership of the page
     *
     * @param page_num The assigned page number by the RegMappedServer
     * @param page_def The page definition struct to populate with data in this file
     */
    void populatePage(uint8_t page_num, reg_mapped_server_page_def_t &page_def);

private:
    /**
     * @brief Private Constructor: create() must be called as only unique_ptr references to this class can be created
     */
    RegMappedRegisterPage() {}

    /**
     * @brief Prevent copying
     */
    RegMappedRegisterPage(const RegMappedRegisterPage &) = delete;
    RegMappedRegisterPage &operator=(RegMappedRegisterPage const &) = delete;

    /**
     * @brief Callback for C reg_mapped_server library.
     */
    static bool register_cb_wrapper(const struct reg_mapped_server_register_definition *reg, bool is_write,
                                    uint32_t *data_ptr);

    /**
     * @brief Gets reference to the requested register. If regs is not large enough, this will add unimplemented entries
     * until regs is large enough to hold the register_def.
     *
     * @param reg_num The register number to get
     */
    reg_mapped_server_register_def_t &getRegAddingPadding(uint8_t reg_num);

    /**
     * @brief The page number for this page (used when calling user register callbacks)
     */
    uint8_t page_num;

    /**
     * @brief Maps callbacks on offsets to the C++ functional callback
     */
    std::map<uint8_t, RegisterCB> callbackMap;

    /**
     * @brief Holds the constant values added to this page
     */
    std::list<uint32_t> constValContainer;

    /**
     * @brief Backing vector for the reg_mapped_server_page_def_t
     */
    std::vector<reg_mapped_server_register_def_t> regs;
};

// ========================================
// Register Mapped Server Class
// ========================================

/**
 * @brief Defines a C++ Register Mapped Server
 */
class RegMappedServer {
public:
    /**
     * @brief Construct a new Reg Mapped Server object
     *
     * @param interfaceMode The interface mode this server implements (see titan/canmore/protocol.h for more info)
     */
    RegMappedServer(uint8_t interfaceMode);

    /**
     * @brief Add a new Register-backed page to the server
     *
     * @param page_num The page number for the page
     * @param description The page description created with RegMappedRegisterPage::create()
     */
    void addRegisterPage(uint8_t page_num, std::unique_ptr<RegMappedRegisterPage> description);

    /**
     * @brief Create a byte mapped page, backed by memory.
     *
     * @attention The data reference in the memory span must remain valid for the entire lifetime of this class.
     * The span, however, may be invalidated after this function call.
     *
     * @param page_num The page number for the page
     * @param perm The access permission for the entire page
     * @param memory The backing memory for this class
     */
    void addByteMappedPage(uint8_t page_num, AccessPerm perm, const std::span<uint8_t> &memory);

    /**
     * @brief Create a word mapped page, backed by memory.
     *
     * @attention The data reference in the memory span must remain valid for the entire lifetime of this class.
     * The span, however, may be invalidated after this function call.
     *
     * @param page_num The page number for the page
     * @param perm The access permission for the entire page
     * @param memory The backing memory for this class
     */
    void addWordMappedPage(uint8_t page_num, AccessPerm perm, const std::span<uint32_t> &memory);

    /**
     * @brief Process the provided packet from the reg mapped client. Must a non-fragmented packet conforming to the
     * register mapped protocol specification.
     *
     * @param data The packet data to process
     */
    void processPacket(const std::span<const uint8_t> &data);

protected:
    /**
     * @brief Transmit Callback to send the requested data packet over can bus (on the requested channel)
     *
     * @param data Data to put in the packet
     */
    virtual void transmit(const std::span<uint8_t> &data) = 0;

private:
    /**
     * @brief Static binding for the underlying reg_mapped_server
     */
    static void transmit_cb_wrapper(uint8_t *msg, size_t len, void *arg);

    /**
     * @brief Gets reference to the requested page. If pages is not large enough, this will add unimplemented entries
     * until pages is large enough to hold the reg_mapped_page.
     *
     * @param page_num The page number to get
     */
    reg_mapped_server_page_def_t &getPageAddingPadding(uint8_t page_num);

    /**
     * @brief Vector holding references to all register-backed pages used in this class
     */
    std::vector<std::unique_ptr<RegMappedRegisterPage>> registerPages;

    /**
     * @brief The backing array for the reg_mapped_server page_array
     */
    std::vector<reg_mapped_server_page_def_t> pages;

    /**
     * @brief The underlying reg_mapped_server instance object
     */
    reg_mapped_server_inst inst;
};

// ========================================
// Register Mapped Server CAN Implementation
// ========================================

class RegMappedCANServer : public Canmore::RegMappedServer, public CANSocket {
public:
    RegMappedCANServer(int ifIndex, uint8_t clientId, uint8_t channel, uint8_t interfaceMode):
        RegMappedServer(interfaceMode), CANSocket(ifIndex), clientId(clientId), channel(channel) {
        // Configure agent to receive agent to client communication on the control interface channel
        struct can_filter rfilter[] = { { .can_id = CANMORE_CALC_UTIL_ID_A2C(clientId, channel),
                                          .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK) } };

        setRxFilters(std::span { rfilter });
    }

    const uint8_t clientId;
    const uint8_t channel;

protected:
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override {
        if (can_id != CANMORE_CALC_UTIL_ID_A2C(clientId, channel)) {
            throw std::logic_error("Received a packet with invalid CAN ID - Somehow the filters broke?");
        }

        processPacket(data);
    }
    void transmit(const std::span<uint8_t> &data) override {
        transmitFrame(CANMORE_CALC_UTIL_ID_C2A(clientId, channel), data);
    }
};

}  // namespace Canmore
