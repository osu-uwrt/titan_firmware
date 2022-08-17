/*
 * Copyright (C) 2021 Robert Pafford
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <csignal>
#include <cstdio>
#include <map>
#include <iostream>
#include <vector>
#include <set>
#include <array>
#include <cstring>
#include <cstdarg>
#include <algorithm>
#include <iomanip>
#include <numeric>
#include <memory>
#include <functional>
#include <ctime>
#include <unistd.h>

#include "boot/uf2.h"
#include "picoboot_connection_cxx.h"
#include "pico/binary_info.h"

#include "uploader.h"

#define ERROR_ARGS -1
#define ERROR_FORMAT -2
#define ERROR_INCOMPATIBLE -3
#define ERROR_READ_FAILED -4
#define ERROR_WRITE_FAILED -5
#define ERROR_USB -6
#define ERROR_NO_DEVICE -7
#define ERROR_NOT_POSSIBLE -8
#define ERROR_CONNECTION -9
#define ERROR_CANCELLED -10
#define ERROR_VERIFICATION_FAILED -11
#define ERROR_UNKNOWN -99

#define EXPORTED __attribute__ ((visibility ("default")))

using std::string;
using std::vector;
using std::pair;
using std::map;

typedef map<enum picoboot_device_result,vector<pair<libusb_device *, libusb_device_handle *>>> device_map;

typedef unsigned int uint;

auto memory_names = map<enum memory_type, string>{
        {memory_type::sram, "RAM"},
        {memory_type::sram_unstriped, "Unstriped RAM"},
        {memory_type::flash, "Flash"},
        {memory_type::xip_sram, "XIP RAM"},
        {memory_type::rom, "ROM"}
};

static string tool_name = "picotool";

static string hex_string(int value, int width=8, bool prefix=true) {
    std::stringstream ss;
    if (prefix) ss << "0x";
    ss << std::setfill('0') << std::setw(width) << std::hex << value;
    return ss.str();
}

std::array<std::array<string, 30>, 10> pin_functions{{
    {"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""},
    {"SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX", "SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX", "SPI1 RX", "SPI1 CSn", "SPI1 SCK", "SPI1 TX", "SPI1 RX", "SPI1 CSn", "SPI1 SCK", "SPI1 TX", "SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX", "SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX", "SPI1 RX", "SPI1 CSn", "SPI1 SCK", "SPI1 TX", "SPI1 RX", "SPI1 CSn"},
    {"UART0 TX", "UART0 RX", "UART0 CTS", "UART0 RTS", "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS", "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS", "UART0 TX", "UART0 RX", "UART0 CTS", "UART0 RTS", "UART0 TX", "UART0 RX", "UART0 CTS", "UART0 RTS", "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS", "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS", "UART0 TX", "UART0 RX"},
    {"I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL"},
    {"PWM0 A", "PWM0 B", "PWM1 A", "PWM1 B", "PWM2 A", "PWM2 B", "PWM3 A", "PWM3 B", "PWM4 A", "PWM4 B", "PWM5 A", "PWM5 B", "PWM6 A", "PWM6 B", "PWM7 A", "PWM7 B", "PWM0 A", "PWM0 B", "PWM1 A", "PWM1 B", "PWM2 A", "PWM2 B", "PWM3 A", "PWM3 B", "PWM4 A", "PWM4 B", "PWM5 A", "PWM5 B", "PWM6 A", "PWM6 B"},
    {"SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO"},
    {"PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0"},
    {"PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1"},
    {"","","","","","","","","","","","","","","","","","","","","CLOCK GPIN0","CLOCK GPOUT0","CLOCK GPIN1","CLOCK GPOUT1","CLOCK GPOUT2","CLOCK GPOUT3","","","",""},
    {"USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN"},
}};

auto bus_device_string = [](struct libusb_device *device) {
    return string("Device at bus ") + std::to_string(libusb_get_bus_number(device)) + ", address " + std::to_string(libusb_get_device_address(device));
};

enum class filetype {bin, elf, uf2};

struct command_failure : std::exception {
    command_failure(int code, string s) : c(code), s(std::move(s)) {}

    const char *what() const noexcept override {
        return s.c_str();
    }

    int code() const { return c; }
private:
    int c;
    string s;
};

struct cancelled_exception : std::exception { };

struct not_mapped_exception : std::exception {
    const char *what() const noexcept override {
        return "Hmm uncaught not mapped";
    }
};

// from -> to
struct range {
    range() : from(0), to(0) {}
    range(uint32_t from, uint32_t to) : from(from), to(to) {}
    uint32_t from;
    uint32_t to;

    bool empty() const {
        return from >= to;
    }
    bool contains(uint32_t addr) const { return addr>=from && addr<to; }
    uint32_t clamp(uint32_t addr) const {
        if (addr < from) addr = from;
        if (addr > to) addr = to;
        return addr;
    }

    void intersect(const range& other) {
        from = other.clamp(from);
        to = other.clamp(to);
    }

    bool intersects(const range& other) const {
        return !(other.from >= to || other.to < from);
    }

};

static void __noreturn fail(int code, string msg) {
    throw command_failure(code, std::move(msg));
}

static void __noreturn fail(int code, const char *format, ...) {
    va_list args;
    va_start(args, format);
    static char error_msg[512];
    vsnprintf(error_msg, sizeof(error_msg), format, args);
    va_end(args);
    fail(code, string(error_msg));
}

// ranges should not overlap
template <typename T> struct range_map {
    struct mapping {
        mapping(uint32_t offset, uint32_t max_offset) : offset(offset), max_offset(max_offset) {}
        const uint32_t offset;
        const uint32_t max_offset;
    };

    void insert(const range& r, T t) {
        if (r.to != r.from) {
            assert(r.to > r.from);
            // check we don't overlap any existing map entries

            auto f = m.upper_bound(r.from); // first element that starts after r.from
            if (f != m.begin()) f--; // back up, to catch element that starts on or before r.from
            for(; f != m.end() && f->first < r.to; f++) { // loop till we can't possibly overlap
                range r2(f->first, f->second.first);
                if (r2.intersects(r)) {
                    fail(ERROR_FORMAT, "Found overlapping memory ranges 0x%08x->0x%08x and 0x%08x->%08x\n",
                         r.from, r.to, r2.from, r2.to);
                }
            }
            m.insert(std::make_pair(r.from, std::make_pair(r.to, t)));
        }
    }

    pair<mapping, T> get(uint32_t p) {
        auto f = m.upper_bound(p);
        if (f == m.end()) {
            if (m.empty())
                throw not_mapped_exception();
        } else if (f == m.begin()) {
            throw not_mapped_exception();
        }
        f--;
        assert(p >= f->first);
        if (p > f->second.first) {
            throw not_mapped_exception();
        }
        return std::make_pair(mapping(p - f->first, f->second.first - f->first), f->second.second);
    }

    uint32_t next(uint32_t p) {
        auto f = m.upper_bound(p);
        if (f == m.end()) {
            return std::numeric_limits<uint32_t>::max();
        }
        return f->first;
    }

    vector<range> ranges() {
        vector<range> r;
        r.reserve(m.size());
        for(const auto &e : m) {
            r.emplace_back(range(e.first, e.second.first));
        }
        return r;
    }

    size_t size() const { return m.size(); }
private:
    map<uint32_t, pair<uint32_t, T>> m;
};

template <typename T> struct raw_type_mapping {
};

#define SAFE_MAPPING(type) template<> struct raw_type_mapping<type> { typedef type access_type; }

//template<> struct raw_type_mapping<uint32_t> {
//    typedef uint32_t access_type;
//};

// these types may be filled directly from byte representation
SAFE_MAPPING(uint8_t);
SAFE_MAPPING(char);
SAFE_MAPPING(uint16_t);
SAFE_MAPPING(uint32_t);
SAFE_MAPPING(binary_info_core_t);
SAFE_MAPPING(binary_info_id_and_int_t);
SAFE_MAPPING(binary_info_id_and_string_t);
SAFE_MAPPING(binary_info_block_device_t);
SAFE_MAPPING(binary_info_pins_with_func_t);
SAFE_MAPPING(binary_info_pins_with_name_t);
SAFE_MAPPING(binary_info_named_group_t);

#define BOOTROM_MAGIC 0x01754d
#define BOOTROM_MAGIC_ADDR 0x00000010
static inline uint32_t rom_table_code(char c1, char c2) {
    return (c2 << 8u) | c1;
}

struct memory_access {
    virtual void read(uint32_t, uint8_t *buffer, uint size) = 0;

    virtual bool is_device() { return false; }

    virtual uint32_t get_binary_start() = 0;

    uint32_t read_int(uint32_t addr) {
        assert(!(addr & 3u));
        uint32_t rc;
        read(addr, (uint8_t *)&rc, 4);
        return rc;
    }

    uint32_t read_short(uint32_t addr) {
        assert(!(addr & 1u));
        uint16_t rc;
        read(addr, (uint8_t *)&rc, 2);
        return rc;
    }

    // read a vector of types that have a raw_type_mapping
    template <typename T> void read_raw(uint32_t addr, T &v) {
        typename raw_type_mapping<T>::access_type& check = v; // ugly check that we aren't trying to read into something we shouldn't
        read(addr, (uint8_t *)&v, sizeof(typename raw_type_mapping<T>::access_type));
    }

    // read a vector of types that have a raw_type_mapping
    template <typename T> vector<T> read_vector(uint32_t addr, uint count) {
        assert(count);
        vector<typename raw_type_mapping<T>::access_type> buffer(count);
        read(addr, (uint8_t *)buffer.data(), count * sizeof(typename raw_type_mapping<T>::access_type));
        vector<T> v;
        v.reserve(count);
        for(const auto &e : buffer) {
            v.push_back(e);
        }
        return v;
    }

    template <typename T> void read_into_vector(uint32_t addr, uint count, vector<T> &v) {
        vector<typename raw_type_mapping<T>::access_type> buffer(count);
        if (count) read(addr, (uint8_t *)buffer.data(), count * sizeof(typename raw_type_mapping<T>::access_type));
        v.clear();
        v.reserve(count);
        for(const auto &e : buffer) {
            v.push_back(e);
        }
    }
};

uint32_t bootrom_func_lookup(memory_access& access, uint16_t tag) {
    auto magic = access.read_int(BOOTROM_MAGIC_ADDR);
    magic &= 0xffffff; // ignore bootrom version
    if (magic != BOOTROM_MAGIC) {
        if (!((magic ^ BOOTROM_MAGIC)&0xffff))
            fail(ERROR_INCOMPATIBLE, "Incorrect RP2040 BOOT ROM version");
        else
            fail(ERROR_INCOMPATIBLE, "RP2040 BOOT ROM not found");
    }

    // dereference the table pointer
    uint32_t table_entry = access.read_short(BOOTROM_MAGIC_ADDR + 4);
    uint16_t entry_tag;
    do {
        entry_tag = access.read_short(table_entry);
        if (entry_tag == tag) {
            // 16 bit symbol is next
            return access.read_short(table_entry+2);
        }
        table_entry += 4;
    } while (entry_tag);
    fail(ERROR_INCOMPATIBLE, "Reboot function not found in BOOT ROM");
}

struct picoboot_memory_access : public memory_access {
    explicit picoboot_memory_access(picoboot::connection &connection) : connection(connection) {}

    bool is_device() override {
        return true;
    }

    uint32_t get_binary_start() override {
        return FLASH_START;
    }

    void read(uint32_t address, uint8_t *buffer, uint size) override {
        if (flash == get_memory_type(address)) {
            connection.exit_xip();
        }
        if (rom == get_memory_type(address) && (address+size) >= 0x2000) {
            // read by memcpy instead
            uint program_base = SRAM_START + 0x4000;
            // program is "return memcpy(SRAM_BASE, 0, 0x4000);"
            std::vector<uint32_t> program = {
                    0x07482101, // movs r1, #1;       lsls r0, r1, #29
                    0x2100038a, // lsls r2, r1, #14;  movs r1, #0
                    0x47184b00, // ldr  r3, [pc, #0]; bx r3
                    bootrom_func_lookup(*this, rom_table_code('M','C'))
            };
            write_vector(program_base, program);
            connection.exec(program_base);
            // 4k is copied into the start of RAM
            connection.read(SRAM_START + address, (uint8_t *) buffer, size);
        } else if (is_transfer_aligned(address) && is_transfer_aligned(address + size)) {
            connection.read(address, (uint8_t *) buffer, size);
        } else {
            if (flash == get_memory_type(address)) {
                uint32_t aligned_start = address & ~(PAGE_SIZE - 1);
                uint32_t aligned_end = (address + size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
                vector<uint8_t> tmp_buffer(aligned_end - aligned_start);
                connection.read(aligned_start, tmp_buffer.data(), aligned_end - aligned_start);
                    std::copy(tmp_buffer.cbegin() + (address - aligned_start), tmp_buffer.cbegin() + (address + size - aligned_start), buffer);
            } else {
                std::stringstream sstream;
                sstream << "Address range " << hex_string(address) << " + " << hex_string(size);
                throw std::invalid_argument(sstream.str());
            }
        }
    }

    // note this does not automatically erase flash
    void write(uint32_t address, uint8_t *buffer, uint size) {
        if (flash == get_memory_type(address)) {
            connection.exit_xip();
        }
        if (is_transfer_aligned(address) && is_transfer_aligned(address + size)) {
            connection.write(address, (uint8_t *) buffer, size);
        } else {
            // for write, we must be correctly sized/aligned in 256 byte chunks
            std::stringstream sstream;
            sstream << "Address range " << hex_string(address) << " + " << hex_string(size);
            throw std::invalid_argument(sstream.str());
        }
    }

    template <typename T> void write_vector(uint32_t addr, vector<T> v) {
        assert(!v.empty());
        write(addr, (uint8_t *)v.data(), v.size() * sizeof(typename raw_type_mapping<T>::access_type));
    }
private:
    picoboot::connection& connection;
};


struct file_memory_access : public memory_access {
    file_memory_access(FILE *file, range_map<size_t>& rmap, uint32_t binary_start) : file(file), rmap(rmap), binary_start(binary_start) {

    }

    uint32_t get_binary_start() override {
        return binary_start;
    }

    void read(uint32_t address, uint8_t *buffer, uint32_t size) override {
        while (size) {
            auto result = rmap.get(address);
            uint this_size = std::min(size, result.first.max_offset - result.first.offset);
            assert( this_size);
            fseek(file, result.second + result.first.offset, SEEK_SET);
            fread(buffer, this_size, 1, file);
            buffer += this_size;
            address += this_size;
            size -= this_size;
        }
    }

    const range_map<size_t> &get_rmap() {
        return rmap;
    }

    ~file_memory_access() {
        fclose(file);
    }
private:
    FILE *file;
    range_map<size_t> rmap;
    uint32_t binary_start;
};

struct remapped_memory_access : public memory_access {
    remapped_memory_access(memory_access &wrap, range_map<uint32_t> rmap) : wrap(wrap), rmap(rmap) {}

    void read(uint32_t address, uint8_t *buffer, uint size) override {
        while (size) {
            auto result = get_remapped(address);
            uint this_size = std::min(size, result.first.max_offset - result.first.offset);
            assert( this_size);
            wrap.read(result.second + result.first.offset, buffer, this_size);
            buffer += this_size;
            address += this_size;
            size -= this_size;
        }
    }

    bool is_device() override {
        return wrap.is_device();
    }

    uint32_t get_binary_start() override {
        return wrap.get_binary_start(); // this is an absolute address
    }

    pair<range_map<uint32_t>::mapping, uint32_t> get_remapped(uint32_t address) {
        try {
            return rmap.get(address);
        } catch (not_mapped_exception&) {
            return std::make_pair(range_map<uint32_t>::mapping(0, rmap.next(address) - address), address);
        }
    }

private:
    memory_access& wrap;
    range_map<uint32_t> rmap;
};

static void __noreturn fail_read_error() {
    fail(ERROR_READ_FAILED, "Failed to read input file");
}

static void __noreturn fail_write_error() {
    fail(ERROR_WRITE_FAILED, "Failed to write output file");
}

struct binary_info_header {
    vector<uint32_t> bi_addr;
    range_map<uint32_t> reverse_copy_mapping;
};

bool find_binary_info(memory_access& access, binary_info_header &hdr) {
    uint32_t base = access.get_binary_start();
    if (!base) {
        fail(ERROR_FORMAT, "UF2 file does not contain a valid RP2 executable image");
    }
    if (base == FLASH_START) base += 0x100;
    vector<uint32_t> buffer = access.read_vector<uint32_t>(base, 64);
    for(uint i=0;i<64;i++) {
        if (buffer[i] == BINARY_INFO_MARKER_START) {
            if (i + 4 < 64 && buffer[i+4] == BINARY_INFO_MARKER_END) {
                uint32_t from = buffer[i+1];
                uint32_t to = buffer[i+2];
                enum memory_type from_type = get_memory_type(from);
                enum memory_type to_type = get_memory_type(to);
                if (to > from &&
                        from_type == to_type &&
                        is_size_aligned(from, 4) &&
                        is_size_aligned(to, 4)) {
                    access.read_into_vector(from, (to - from) / 4, hdr.bi_addr);
                    uint32_t cpy_table = buffer[i+3];
                    vector<uint32_t> mapping;
                    do {
                        mapping = access.read_vector<uint32_t>(cpy_table, 3);
                        if (!mapping[0]) break;
                        // from, to_start, to_end
                        hdr.reverse_copy_mapping.insert(range(mapping[1], mapping[2]), mapping[0]);
                        cpy_table += 12;
                    } while (hdr.reverse_copy_mapping.size() < 10); // arbitrary max
                    return true;
                }
            }
        }
    }
    return false;
}

string read_string(memory_access &access, uint32_t addr) {
    const uint max_length = 256; // todo better incremental length handling
    auto v = access.read_vector<char>(addr, 256);
    uint length;
    for (length = 0; length < max_length; length++) {
        if (!v[length]) {
            break;
        }
    }
    return string(v.data(), length);
}

struct bi_visitor_base {
    void visit(memory_access& access, const binary_info_header& hdr) {
        for (const auto &a : hdr.bi_addr) {
            visit(access, a);
        }
    }

    void visit(memory_access& access, uint32_t addr) {
        binary_info_core_t bi;
        access.read_raw(addr, bi);
        switch (bi.type) {
            case BINARY_INFO_TYPE_RAW_DATA:
                break;
            case BINARY_INFO_TYPE_SIZED_DATA:
                break;
            case BINARY_INFO_TYPE_BINARY_INFO_LIST_ZERO_TERMINATED:
                zero_terminated_bi_list(access, bi, addr);
                break;
            case BINARY_INFO_TYPE_BSON:
                break;
            case BINARY_INFO_TYPE_ID_AND_INT: {
                binary_info_id_and_int_t value;
                access.read_raw(addr, value);
                id_and_value(bi.tag, value.id, value.value);
                break;
            }
            case BINARY_INFO_TYPE_ID_AND_STRING: {
                binary_info_id_and_string_t value;
                access.read_raw(addr, value);
                string s = read_string(access, value.value);
                id_and_string(bi.tag, value.id, s);
                break;
            }
            case BINARY_INFO_TYPE_BLOCK_DEVICE: {
                binary_info_block_device_t value;
                access.read_raw(addr, value);
                block_device(access, value);
                break;
            }
            case BINARY_INFO_TYPE_PINS_WITH_FUNC: {
                binary_info_pins_with_func_t value;
                access.read_raw(addr, value);
                uint type = value.pin_encoding & 7u;
                uint func = (value.pin_encoding >> 3u) & 0xfu;
                if (type == BI_PINS_ENCODING_RANGE) {
                    // todo pin range
                } else if (type == BI_PINS_ENCODING_MULTI) {
                    uint32_t mask = 0;
                    int last = -1;
                    uint work = value.pin_encoding >> 7u;
                    for(int i=0;i<5;i++) {
                        int cur = (int) (work & 0x1fu);
                        mask |= 1u << cur;
                        if (cur == last) break;
                        last = cur;
                        work >>= 5u;
                    }
                    pins(mask, func, "");
                }
                break;
            }
            case BINARY_INFO_TYPE_PINS_WITH_NAME: {
                binary_info_pins_with_name_t value;
                access.read_raw(addr, value);
                pins(value.pin_mask, -1, read_string(access, value.label));
                break;
            }
            case BINARY_INFO_TYPE_NAMED_GROUP: {
                binary_info_named_group_t value;
                access.read_raw(addr, value);
                named_group(value.core.tag, value.parent_id, value.group_tag, value.group_id, read_string(access, value.label), value.flags);
                break;
            }
            default:
                unknown(access, bi, addr);
        }
    }

    virtual void unknown(memory_access& access, const binary_info_core_t &bi_core, uint32_t addr) {}
    virtual void id_and_value(int tag, uint32_t id, uint32_t value) {
//        printf("ID=0x%08x int value=%d 0x%08x\n", id, value, value);
    }
    virtual void id_and_string(int tag, uint32_t id, const string& value) {
//        printf("ID=0x%08x int value=%s\n", id, value.c_str());
    }
    virtual void block_device(memory_access& access, binary_info_block_device_t &bi_bdev) {
    }

    virtual void pins(uint32_t pin_mask, int func, string name) {
        if (func != -1) {
            if (func >= (int)pin_functions.size())
                return;
        }
        for(uint i=0; i<30; i++) {
            if (pin_mask & (1u << i)) {
                if (func != -1) {
                    pin(i, pin_functions[func][i]);
                } else {
                    auto sep = name.find_first_of('|');
                    auto cur = name.substr(0, sep);
                    if (cur.empty()) continue;
                    pin(i, cur.c_str());
                    if (sep != string::npos) {
                        name = name.substr(sep + 1);
                    }
                }
            }
        }
    }

    virtual void pin(uint i, const string& name) {

    }

    virtual void zero_terminated_bi_list(memory_access& access, const binary_info_core_t &bi_core, uint32_t addr) {
        uint32_t bi_addr;
        access.read_raw<uint32_t>(addr,bi_addr);
        while (bi_addr) {
            visit(access, addr);
            access.read_raw<uint32_t>(addr,bi_addr);
        }
    }

    virtual void named_group(int parent_tag, uint32_t parent_id, int group_tag, uint32_t group_id, const string& label, uint flags) {

    }
};

struct bi_visitor : public bi_visitor_base {
    typedef std::function<void(int tag, uint32_t id, uint32_t value)> id_and_int_fn;
    typedef std::function<void(int tag, uint32_t id, const string &value)> id_and_string_fn;
    typedef std::function<void(uint num, const string &label)> pin_fn;
    typedef std::function<void(int parent_tag, uint32_t parent_id, int group_tag, uint32_t group_id,
                               const string &label, uint flags)> named_group_fn;
    typedef std::function<void(memory_access &access, binary_info_block_device_t &bi_bdev)> block_device_fn;

    id_and_int_fn _id_and_int;
    id_and_string_fn _id_and_string;
    pin_fn _pin;
    named_group_fn _named_group;
    block_device_fn _block_device;

    bi_visitor &id_and_int(id_and_int_fn fn) {
        _id_and_int = std::move(fn);
        return *this;
    }

    bi_visitor &id_and_string(id_and_string_fn fn) {
        _id_and_string = std::move(fn);
        return *this;
    }

    bi_visitor &pin(pin_fn fn) {
        _pin = std::move(fn);
        return *this;
    }

    bi_visitor &named_group(named_group_fn fn) {
        _named_group = std::move(fn);
        return *this;
    }

    bi_visitor &block_device(block_device_fn fn) {
        _block_device = std::move(fn);
        return *this;
    }

protected:
    void id_and_value(int tag, uint32_t id, uint32_t value) override {
        if (_id_and_int) _id_and_int(tag, id, value);
    }

    void id_and_string(int tag, uint32_t id, const string &value) override {
        if (_id_and_string) _id_and_string(tag, id, value);
    }

    void pin(uint i, const string &name) override {
        if (_pin) _pin(i, name);
    }

    void named_group(int parent_tag, uint32_t parent_id, int group_tag, uint32_t group_id, const string &label,
                     uint flags) override {
        if (_named_group) _named_group(parent_tag, parent_id, group_tag, group_id, label, flags);
    }

    void block_device(memory_access &access, binary_info_block_device_t &bi_bdev) override {
        if (_block_device) _block_device(access, bi_bdev);
    }
};

int32_t guess_flash_size(memory_access &access) {
    assert(access.is_device());
    // Check that flash is not erased (TODO should check for second stage)
    auto first_two_pages = access.read_vector<uint8_t>(FLASH_START, 2*PAGE_SIZE);
    bool all_match = std::equal(first_two_pages.begin(),
                                first_two_pages.begin() + PAGE_SIZE,
                                first_two_pages.begin() + PAGE_SIZE);
    if (all_match) {
        return 0;
    }

    // Read at decreasing power-of-two addresses until we don't see the boot pages again
    const int min_size = 16 * PAGE_SIZE;
    const int max_size = 8 * 1024 * 1024;
    int size;
    for (size = max_size; size >= min_size; size >>= 1) {
        auto new_pages = access.read_vector<uint8_t>(FLASH_START + size, 2*PAGE_SIZE);
        if (!std::equal(first_two_pages.begin(), first_two_pages.end(), new_pages.begin())) break;
    }
    return size * 2;
}

FILE *get_file(const char* filename, const char *mode) {
    FILE *file = fopen(filename, mode);
    if (!file) fail(ERROR_READ_FAILED, "Could not open '%s'", filename);
    return file;
}

void build_rmap_uf2(FILE *file, range_map<size_t>& rmap) {
    fseek(file, 0, SEEK_SET);
    uf2_block block;
    uint pos = 0;
    do {
        if (1 != fread(&block, sizeof(uf2_block), 1, file)) {
            if (feof(file)) break;
            fail(ERROR_READ_FAILED, "unexpected end of input file");
        }
        if (block.magic_start0 == UF2_MAGIC_START0 && block.magic_start1 == UF2_MAGIC_START1 &&
            block.magic_end == UF2_MAGIC_END) {
            if (block.flags & UF2_FLAG_FAMILY_ID_PRESENT && block.file_size == RP2040_FAMILY_ID &&
                !(block.flags & UF2_FLAG_NOT_MAIN_FLASH) && block.payload_size == PAGE_SIZE) {
                rmap.insert(range(block.target_addr, block.target_addr + PAGE_SIZE), pos + offsetof(uf2_block, data[0]));
            }
        }
        pos += sizeof(uf2_block);
    } while (true);
}

uint32_t find_binary_start(range_map<size_t>& rmap) {
    range flash(FLASH_START, FLASH_END);
    range sram(SRAM_START, SRAM_END);
    range xip_sram(XIP_SRAM_BASE, XIP_SRAM_END);
    uint32_t binary_start = std::numeric_limits<uint32_t>::max();
    for (const auto &r : rmap.ranges()) {
        if (r.contains(FLASH_START)) {
            return FLASH_START;
        }
        if (sram.contains(r.from) || xip_sram.contains((r.from))) {
            if (r.from < binary_start || (xip_sram.contains(binary_start) && sram.contains(r.from))) {
                binary_start = r.from;
            }
        }
    }
    if (get_memory_type(binary_start) == invalid) {
        return 0;
    }
    return binary_start;
}

file_memory_access get_file_memory_access(const char* filename) {
    FILE *file = get_file(filename, "rb");
    range_map<size_t> rmap;
    uint32_t binary_start;
    try {
        // Only uf2 is going to be supported
        build_rmap_uf2(file, rmap);
        binary_start = find_binary_start(rmap);
        return file_memory_access(file, rmap, binary_start);
    } catch (std::exception&) {
        fclose(file);
        throw;
    }
}

struct progress_bar {
    explicit progress_bar(string prefix, int width = 30) : prefix(std::move(prefix)), width(width) {
        progress(0);
    }

    void progress(int _percent) {
        if (_percent != percent) {
            percent = _percent;
            uint len = (width * percent) / 100;
            std::cout << prefix << "[" << string(len, '=') << string(width-len, ' ') << "]  " << std::to_string(percent) << "%\r" << std::flush;
        }
    }

    void progress(long dividend, long divisor) {
        progress(divisor ? (int)((100 * dividend) / divisor) : 100);
    }

    ~progress_bar() {
        std::cout << "\n";
    }

    std::string prefix;
    int percent = -1;
    int width;
};

vector<range> get_colaesced_ranges(file_memory_access &file_access) {
    auto rmap = file_access.get_rmap();
    auto ranges = rmap.ranges();
    std::sort(ranges.begin(), ranges.end(), [](const range& a, const range &b) {
        return a.from < b.from;
    });
    // coalesce all the contiguous ranges
    for(auto i = ranges.begin(); i < ranges.end(); ) {
        if (i != ranges.end() - 1) {
            if (i->to == (i+1)->from) {
                i->to = (i+1)->to;
                i = ranges.erase(i+1) - 1;
                continue;
            }
        }
        i++;
    }
    return ranges;
}

static picoboot::connection get_single_bootsel_device_connection(rp2040_device* device, bool exclusive = true) {
    return picoboot::connection(device->handle, &device->data.bootrom, exclusive);
}

extern "C" {

EXPORTED bool get_uf2_board_type(const char* filename, char* name_out, size_t name_size) {
    try {
        auto file_access = get_file_memory_access(filename);
        binary_info_header hdr;
        if (find_binary_info(file_access, hdr)) {
            auto access = remapped_memory_access(file_access, hdr.reverse_copy_mapping);
            auto visitor = bi_visitor{};

            string pico_board;

            visitor.id_and_string([&](int tag, uint32_t id, const string& value) {
                if (tag != BINARY_INFO_TAG_RASPBERRY_PI)
                    return;
                if (id == BINARY_INFO_ID_RP_PICO_BOARD) pico_board = value;
            });

            visitor.visit(access, hdr);

            if (!pico_board.empty() && pico_board.length() <= name_size) {
                strncpy(name_out, pico_board.c_str(), name_size);
                return true;
            }
        }
    } catch (const std::exception &exc) {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << "Error during file read: ";
        std::cerr << exc.what() << "\n";
    }
    return false;
}

EXPORTED bool load_file(const char* filename, rp2040_device_t* device, bool verify, bool execute_after) {
    try {
        auto file_access = get_file_memory_access(filename);
        auto con = get_single_bootsel_device_connection(device);
        picoboot_memory_access raw_access(con);
        // Removed overwrite protection
        auto ranges = get_colaesced_ranges(file_access);
        for (auto mem_range : ranges) {
            enum memory_type t1 = get_memory_type(mem_range.from);
            enum memory_type t2 = get_memory_type(mem_range.to);
            if (t1 != t2 || t1 == invalid || t1 == rom) {
                fail(ERROR_FORMAT, "File to load contained an invalid memory range 0x%08x-0x%08x", mem_range.from,
                    mem_range.to);
            }
            // Removing overwrite protection
        }
        for (auto mem_range : ranges) {
            enum memory_type type = get_memory_type(mem_range.from);
            // new scope for progress bar
            {
                progress_bar bar("Loading into " + memory_names[type] + ": ");
                uint32_t batch_size = FLASH_SECTOR_ERASE_SIZE;
                bool ok = true;
                vector<uint8_t> file_buf;
                vector<uint8_t> device_buf;
                for (uint32_t base = mem_range.from; base < mem_range.to && ok; ) {
                    uint32_t this_batch = std::min(mem_range.to - base, batch_size);
                    if (type == flash) {
                        // we have to erase an entire page, so then fill with zeros
                        range aligned_range(base & ~(FLASH_SECTOR_ERASE_SIZE - 1), (base & ~(FLASH_SECTOR_ERASE_SIZE - 1)) + FLASH_SECTOR_ERASE_SIZE);
                        range read_range(base, base + this_batch);
                        read_range.intersect(aligned_range);
                        file_access.read_into_vector(read_range.from, read_range.to - read_range.from, file_buf);
                        // zero padding up to FLASH_SECTOR_ERASE_SIZE
                        file_buf.insert(file_buf.begin(), read_range.from - aligned_range.from, 0);
                        file_buf.insert(file_buf.end(), aligned_range.to - read_range.to, 0);
                        assert(file_buf.size() == FLASH_SECTOR_ERASE_SIZE);
                        con.exit_xip();
                        con.flash_erase(aligned_range.from, FLASH_SECTOR_ERASE_SIZE);
                        raw_access.write_vector(aligned_range.from, file_buf);
                        base = read_range.to; // about to add batch_size
                    } else {
                        file_access.read_into_vector(base, this_batch, file_buf);
                        raw_access.write_vector(base, file_buf);
                        base += this_batch;
                    }
                    bar.progress(base - mem_range.from, mem_range.to - mem_range.from);
                }
            }
            if (verify) {
                bool ok = true;
                {
                    progress_bar bar("Verifying " + memory_names[type] + ":    ");
                    uint32_t batch_size = FLASH_SECTOR_ERASE_SIZE;
                    vector<uint8_t> file_buf;
                    vector<uint8_t> device_buf;
                    uint32_t pos = mem_range.from;
                    for (uint32_t base = mem_range.from; base < mem_range.to && ok; base += batch_size) {
                        uint32_t this_batch = std::min(mem_range.to - base, batch_size);
                        file_access.read_into_vector(base, this_batch, file_buf);
                        raw_access.read_into_vector(base, this_batch, device_buf);
                        assert(file_buf.size() == device_buf.size());
                        for (uint i = 0; i < this_batch; i++) {
                            if (file_buf[i] != device_buf[i]) {
                                pos = base + i;
                                ok = false;
                                break;
                            }
                        }
                        if (ok) {
                            pos = base + this_batch;
                        }
                        bar.progress(pos - mem_range.from, mem_range.to - mem_range.from);
                    }
                }
                if (ok) {
                    std::cout << "  OK\n";
                } else {
                    std::cout << "  FAILED\n";
                    fail(ERROR_VERIFICATION_FAILED, "The device contents did not match the file");
                }
            }
        }
        if (execute_after) {
            uint32_t start = file_access.get_binary_start();
            if (!start) {
                fail(ERROR_FORMAT, "Cannot execute as file does not contain a valid RP2 executable image");
            }
            con.reboot(flash == get_memory_type(start) ? 0 : start, SRAM_END, 500);
            std::cout << "\nThe device was rebooted to start the application.\n";
            return true;
        }
    } catch (const std::exception &exc) {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << "Error during upload: ";
        std::cerr << exc.what() << "\n";
    }

    return false;
}

EXPORTED bool reboot_bootsel_to_bootsel(rp2040_device_t* device, uint32_t disable_interface_mask){
    try {
        auto con = get_single_bootsel_device_connection(device, false);
        picoboot_memory_access raw_access(con);

        disable_interface_mask &= 0xFF;

        uint program_base = SRAM_START;
        std::vector<uint32_t> program = {
                0x20002100 | disable_interface_mask, // movs r0, #0;        movs r1, #disable_interface_mask
                0x47104a00,                          // ldr  r2, [pc, #0];  bx r2
                bootrom_func_lookup(raw_access, rom_table_code('U', 'B'))
        };

        raw_access.write_vector(program_base, program);
        try {
            con.exec(program_base);
        } catch (picoboot::connection_error &e) {
            // the reset_usb_boot above has a very short delay, so it frequently causes libusb to return
            // fairly unpredictable errors... i think it is best to ignore them, because catching a rare
            // case where the reboot command fails, is probably less important than potentially confusing
            // the user with spurious error messages
        }
        return true;
    } catch (const std::exception &exc) {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << "Error during reset:\n";
        std::cerr << exc.what();
    }
    return false;
}
}