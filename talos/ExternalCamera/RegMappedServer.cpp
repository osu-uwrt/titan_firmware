#include "RegMappedServer.hpp"

using namespace Canmore;

// ========================================
// Register Mapped Server Class
// ========================================

RegMappedServer::RegMappedServer(uint8_t interfaceMode) {
    // Initialize reg_mapped_server instance
    inst.tx_func = &transmit_cb_wrapper;
    inst.arg = this;
    inst.control_interface_mode = interfaceMode;
    // page_array and num_pages will be set in the callback (as the vector is subject to change)

    // Zero out required state fields
    inst.bulk_last_seq_num = 0;
    inst.bulk_error_code = 0;
    inst.in_bulk_request = false;
}

void RegMappedServer::addRegisterPage(uint8_t page_num, std::unique_ptr<RegMappedRegisterPage> description) {
    // Get the page_def reference
    auto &page_def = getPageAddingPadding(page_num);

    // Populate the page definition
    description->populatePage(page_num, page_def);

    // Take ownership of the page description so it doesn't get destroyed until this class gets destroyed
    registerPages.emplace_back(std::move(description));
}

void RegMappedServer::addByteMappedPage(uint8_t page_num, AccessPerm perm, const std::span<uint8_t> &memory) {
    // Get the page_def reference
    auto &page_def = getPageAddingPadding(page_num);

    // Fill out page definition
    page_def.page_type = page_def.PAGE_TYPE_MEMORY_MAPPED_BYTE;
    page_def.type.mem_mapped_byte.perm = perm;
    page_def.type.mem_mapped_byte.base_addr = memory.data();
    page_def.type.mem_mapped_byte.size = memory.size();
}

void RegMappedServer::addWordMappedPage(uint8_t page_num, AccessPerm perm, const std::span<uint32_t> &memory) {
    // Get the page_def reference
    auto &page_def = getPageAddingPadding(page_num);

    // Fill out page definition
    page_def.page_type = page_def.PAGE_TYPE_MEMORY_MAPPED_WORD;
    page_def.type.mem_mapped_word.perm = perm;
    page_def.type.mem_mapped_word.base_addr = memory.data();
    page_def.type.mem_mapped_word.num_words = memory.size();
}

void RegMappedServer::processPacket(const std::span<uint8_t> &data) {
    // Fill out in case the vector was modified since we last called this function
    inst.page_array = pages.data();
    inst.num_pages = pages.size();

    reg_mapped_server_handle_request(&inst, data.data(), data.size());
}

void RegMappedServer::transmit_cb_wrapper(uint8_t *msg, size_t len, void *arg) {
    auto inst = (RegMappedServer *) arg;
    inst->transmit(std::span<uint8_t>(msg, len));
}

reg_mapped_server_page_def_t &RegMappedServer::getPageAddingPadding(uint8_t page_num) {
    // If pages isn't big enough, fill it up with unimplemented pages
    if (pages.size() <= page_num) {
        pages.reserve(page_num + 1);
        reg_mapped_server_page_def_t page_def = {};
        page_def.page_type = page_def.PAGE_TYPE_UNIMPLEMENTED;
        while (pages.size() <= page_num) {
            pages.push_back(page_def);
        }
    }

    // Return reference to page in the pages vector
    return pages.at(page_num);
}

// ========================================
// Register Mapped Page Class
// ========================================

void RegMappedRegisterPage::addMemoryRegister(uint8_t reg_num, AccessPerm perm, uint32_t *ptr) {
    // Get reference to the register poitner
    auto &reg_def = getRegAddingPadding(reg_num);

    // Set the required attributes
    reg_def.reg_type = reg_def.REGISTER_TYPE_MEMORY;
    reg_def.type.memory.perm = perm;
    reg_def.type.memory.reg_ptr = ptr;
}

void RegMappedRegisterPage::addConstRegister(uint8_t reg_num, uint32_t val) {
    // Add constant value to internal constant list and grab its pointer
    constValContainer.push_back(val);
    auto data_ptr = &constValContainer.back();

    // Get reference to the register poitner
    auto &reg_def = getRegAddingPadding(reg_num);

    // Set the required attributes
    reg_def.reg_type = reg_def.REGISTER_TYPE_MEMORY;
    reg_def.type.memory.perm = REGISTER_PERM_READ_ONLY;
    reg_def.type.memory.reg_ptr = data_ptr;
}

void RegMappedRegisterPage::addCallbackRegister(uint8_t reg_num, AccessPerm perm, RegisterCB callback) {
    // Get reference to the register poitner
    auto &reg_def = getRegAddingPadding(reg_num);

    // Set the required attributes
    reg_def.reg_type = reg_def.REGISTER_TYPE_EXEC;
    reg_def.type.exec.perm = perm;
    reg_def.type.exec.callback = &register_cb_wrapper;
    reg_def.type.exec.arg = this;

    // Register the callback in the internal map
    callbackMap.emplace(reg_num, callback);
}

void RegMappedRegisterPage::populatePage(uint8_t page_num, reg_mapped_server_page_def_t &page_def) {
    // Save the page number (used in callbacks)
    this->page_num = page_num;

    // Populate the page definition with this class's definition
    page_def.page_type = page_def.PAGE_TYPE_REGISTER_MAPPED;
    page_def.type.reg_mapped.reg_array = regs.data();
    page_def.type.reg_mapped.num_registers = regs.size();
}

bool RegMappedRegisterPage::register_cb_wrapper(const struct reg_mapped_server_register_definition *reg, bool is_write,
                                                uint32_t *data_ptr) {
    // Get class instance
    auto inst = (RegMappedRegisterPage *) reg->type.exec.arg;

    // Get the register index. We know regs will be within inst (if not, we'll fail the bounds check getting the
    // callback)
    size_t reg_num = reg - inst->regs.data();

    if (reg_num > UINT8_MAX)
        throw std::out_of_range("Register callback fired with invalid reg pointer provided");

    // Look up the callback in the callback map
    auto ref = inst->callbackMap.find(reg_num);
    if (ref == inst->callbackMap.end()) {
        throw std::out_of_range("Register callback fired with invalid reg pointer provided");
    }

    // Call the C++ function
    uint16_t reg_addr = (inst->page_num << 8) | reg_num;
    return ref->second(reg_addr, is_write, data_ptr);
}

reg_mapped_server_register_def_t &RegMappedRegisterPage::getRegAddingPadding(uint8_t reg_num) {
    // If regs isn't big enough, fill it up with unimplemented regs
    if (regs.size() <= reg_num) {
        regs.reserve(reg_num + 1);
        reg_mapped_server_register_def_t reg_def = {};
        reg_def.reg_type = reg_def.REGISTER_TYPE_UNIMPLEMENTED;
        while (regs.size() <= reg_num) {
            regs.push_back(reg_def);
        }
    }

    // Return reference to register in the regs vector
    return regs.at(reg_num);
}
