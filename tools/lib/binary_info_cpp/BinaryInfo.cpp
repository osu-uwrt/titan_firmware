#include "BinaryInfo.hpp"

#include "boot/uf2.h"

#include "pico/binary_info.h"
#include "titan/binary_info.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <vector>

namespace BinaryInfo {
using std::map;
using std::pair;
using std::string;
using std::vector;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

std::array<std::array<string, 30>, 10> pin_functions { {
    { "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" },
    { "SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX", "SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX",
      "SPI1 RX", "SPI1 CSn", "SPI1 SCK", "SPI1 TX", "SPI1 RX", "SPI1 CSn", "SPI1 SCK", "SPI1 TX",
      "SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX", "SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX",
      "SPI1 RX", "SPI1 CSn", "SPI1 SCK", "SPI1 TX", "SPI1 RX", "SPI1 CSn" },
    { "UART0 TX", "UART0 RX", "UART0 CTS", "UART0 RTS", "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS",
      "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS", "UART0 TX", "UART0 RX", "UART0 CTS", "UART0 RTS",
      "UART0 TX", "UART0 RX", "UART0 CTS", "UART0 RTS", "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS",
      "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS", "UART0 TX", "UART0 RX" },
    { "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL",
      "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL",
      "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL",
      "I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL", "I2C0 SDA", "I2C0 SCL" },
    { "PWM0 A", "PWM0 B", "PWM1 A", "PWM1 B", "PWM2 A", "PWM2 B", "PWM3 A", "PWM3 B", "PWM4 A", "PWM4 B",
      "PWM5 A", "PWM5 B", "PWM6 A", "PWM6 B", "PWM7 A", "PWM7 B", "PWM0 A", "PWM0 B", "PWM1 A", "PWM1 B",
      "PWM2 A", "PWM2 B", "PWM3 A", "PWM3 B", "PWM4 A", "PWM4 B", "PWM5 A", "PWM5 B", "PWM6 A", "PWM6 B" },
    { "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO",
      "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO", "SIO" },
    { "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0",
      "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0",
      "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0", "PIO0" },
    { "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1",
      "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1",
      "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1", "PIO1" },
    { "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "CLOCK GPIN0",
      "CLOCK GPOUT0",
      "CLOCK GPIN1",
      "CLOCK GPOUT1",
      "CLOCK GPOUT2",
      "CLOCK GPOUT3",
      "",
      "",
      "",
      "" },
    { "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN",
      "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN",
      "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN",
      "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN",
      "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN", "USB OVCUR DET", "USB VBUS DET", "USB VBUS EN" },
} };

struct not_mapped_exception : std::exception {
    const char *what() const noexcept override { return "Hmm uncaught not mapped"; }
};

// from -> to
struct range {
    range(): from(0), to(0) {}
    range(uint32_t from, uint32_t to): from(from), to(to) {}
    uint32_t from;
    uint32_t to;

    bool empty() const { return from >= to; }
    bool contains(uint32_t addr) const { return addr >= from && addr < to; }
    uint32_t clamp(uint32_t addr) const {
        if (addr < from)
            addr = from;
        if (addr > to)
            addr = to;
        return addr;
    }

    void intersect(const range &other) {
        from = other.clamp(from);
        to = other.clamp(to);
    }

    bool intersects(const range &other) const { return !(other.from >= to || other.to < from); }
};

// ranges should not overlap
template <typename T> struct range_map {
    struct mapping {
        mapping(uint32_t offset, uint32_t max_offset): offset(offset), max_offset(max_offset) {}
        const uint32_t offset;
        const uint32_t max_offset;
    };

    void insert(const range &r, T t) {
        if (r.to != r.from) {
            assert(r.to > r.from);
            // check we don't overlap any existing map entries

            auto f = m.upper_bound(r.from);  // first element that starts after r.from
            if (f != m.begin())
                f--;                                        // back up, to catch element that starts on or before r.from
            for (; f != m.end() && f->first < r.to; f++) {  // loop till we can't possibly overlap
                range r2(f->first, f->second.first);
                if (r2.intersects(r)) {
                    printf("Found overlapping memory ranges 0x%08x->0x%08x and 0x%08x->%08x\n", r.from, r.to, r2.from,
                           r2.to);
                    throw std::runtime_error("Found overlapping memory ranges");
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
        }
        else if (f == m.begin()) {
            throw not_mapped_exception();
        }
        f--;
        assert(p >= f->first);
        if (p >= f->second.first) {
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
        for (const auto &e : m) {
            r.emplace_back(range(e.first, e.second.first));
        }
        return r;
    }

    size_t size() const { return m.size(); }

private:
    map<uint32_t, pair<uint32_t, T>> m;
};

template <typename T> struct raw_type_mapping {};

#define SAFE_MAPPING(type)                                                                                             \
    template <> struct raw_type_mapping<type> {                                                                        \
        typedef type access_type;                                                                                      \
    }

// template<> struct raw_type_mapping<uint32_t> {
//     typedef uint32_t access_type;
// };

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

struct memory_access {
    virtual void read(uint32_t p, uint8_t *buffer, uint size) { read(p, buffer, size, false); }

    virtual void read(uint32_t, uint8_t *buffer, uint size, bool zero_fill) = 0;

    virtual bool is_device() { return false; }

    virtual uint32_t get_binary_start() = 0;

    uint32_t read_int(uint32_t addr) {
        assert(!(addr & 3u));
        uint32_t rc;
        read(addr, (uint8_t *) &rc, 4);
        return rc;
    }

    uint32_t read_short(uint32_t addr) {
        assert(!(addr & 1u));
        uint16_t rc;
        read(addr, (uint8_t *) &rc, 2);
        return rc;
    }

    // read a vector of types that have a raw_type_mapping
    template <typename T> void read_raw(uint32_t addr, T &v) {
        typename raw_type_mapping<T>::access_type &check =
            v;  // ugly check that we aren't trying to read into something we shouldn't
        (void) check;
        read(addr, (uint8_t *) &v, sizeof(typename raw_type_mapping<T>::access_type));
    }

    // read a vector of types that have a raw_type_mapping
    template <typename T> vector<T> read_vector(uint32_t addr, uint count, bool zero_fill = false) {
        assert(count);
        vector<typename raw_type_mapping<T>::access_type> buffer(count);
        read(addr, (uint8_t *) buffer.data(), count * sizeof(typename raw_type_mapping<T>::access_type), zero_fill);
        vector<T> v;
        v.reserve(count);
        for (const auto &e : buffer) {
            v.push_back(e);
        }
        return v;
    }

    template <typename T> void read_into_vector(uint32_t addr, uint count, vector<T> &v, bool zero_fill = false) {
        vector<typename raw_type_mapping<T>::access_type> buffer(count);
        if (count)
            read(addr, (uint8_t *) buffer.data(), count * sizeof(typename raw_type_mapping<T>::access_type), zero_fill);
        v.clear();
        v.reserve(count);
        for (const auto &e : buffer) {
            v.push_back(e);
        }
    }
};

struct aligned_memory_access : public memory_access {
    uint32_t get_binary_start() override { return getBaseAddress(); }

    virtual uint32_t getBaseAddress() = 0;
    virtual uint32_t getSize() = 0;
    virtual std::array<uint8_t, UF2_PAGE_SIZE> &readBlock(uint32_t address) = 0;

    void read(uint32_t address, uint8_t *buffer, uint32_t size, bool zero_fill) override {
        while (size) {
            uint32_t this_size = std::min(UF2_PAGE_SIZE, size);
            uint32_t page_off = address % UF2_PAGE_SIZE;
            uint32_t page_addr = address - page_off;
            if (page_addr >= getBaseAddress() && page_addr < getBaseAddress() + getSize()) {
                auto block = readBlock(page_addr);
                this_size = std::min(UF2_PAGE_SIZE - page_off, size);
                std::copy_n(&block.at(page_off), this_size, buffer);
            }
            else {
                if (zero_fill) {
                    // address is not in a range, so fill up to next range with zeros
                    memset(buffer, 0, this_size);
                }
                else {
                    throw not_mapped_exception();
                }
            }
            buffer += this_size;
            address += this_size;
            size -= this_size;
        }
    }
};

struct uf2_memory_access : public aligned_memory_access {
    uf2_memory_access(AppImage &uf2): uf2(uf2) {}

    uint32_t getBaseAddress() override { return uf2.getBaseAddress(); }
    uint32_t getSize() override { return uf2.getSize(); }
    std::array<uint8_t, UF2_PAGE_SIZE> &readBlock(uint32_t address) override { return uf2.getAddress(address); }

private:
    AppImage &uf2;
};

struct device_memory_access : public aligned_memory_access {
    device_memory_access(RP2040FlashInterface &dev, uint32_t offset = 0): dev(dev), baseAddr(FLASH_BASE + offset) {}

    uint32_t getBaseAddress() override { return baseAddr; }
    uint32_t getSize() override { return dev.getFlashSize(); }

    std::array<uint8_t, UF2_PAGE_SIZE> &readBlock(uint32_t address) override {
        auto it = readCache.find(address);
        if (it == readCache.end()) {
            std::array<uint8_t, UF2_PAGE_SIZE> block;
            dev.readBytes(address, block);
            readCache.insert({ address, block });

            return readCache.at(address);
        }
        else {
            return it->second;
        }
    }

private:
    RP2040FlashInterface &dev;
    uint32_t baseAddr;
    std::map<uint32_t, std::array<uint8_t, UF2_PAGE_SIZE>> readCache;
};

struct remapped_memory_access : public memory_access {
    remapped_memory_access(memory_access &wrap, range_map<uint32_t> rmap): wrap(wrap), rmap(rmap) {}

    void read(uint32_t address, uint8_t *buffer, uint size, bool zero_fill) override {
        while (size) {
            auto result = get_remapped(address);
            uint this_size = std::min(size, result.first.max_offset - result.first.offset);
            assert(this_size);
            wrap.read(result.second + result.first.offset, buffer, this_size, zero_fill);
            buffer += this_size;
            address += this_size;
            size -= this_size;
        }
    }

    bool is_device() override { return wrap.is_device(); }

    uint32_t get_binary_start() override {
        return wrap.get_binary_start();  // this is an absolute address
    }

    pair<range_map<uint32_t>::mapping, uint32_t> get_remapped(uint32_t address) {
        try {
            return rmap.get(address);
        } catch (not_mapped_exception &) {
            return std::make_pair(range_map<uint32_t>::mapping(0, rmap.next(address) - address), address);
        }
    }

private:
    memory_access &wrap;
    range_map<uint32_t> rmap;
};

struct binary_info_header {
    vector<uint32_t> bi_addr;
    range_map<uint32_t> reverse_copy_mapping;
};

bool find_binary_info(memory_access &access, binary_info_header &hdr, uint32_t base = 0, uint32_t search_size = 128) {
    // If no base passed, get it from the binary start
    if (!base) {
        base = access.get_binary_start();
    }

    if (!base) {
        return false;
    }
    // Read first two pages
    // In the event boot2 isn't present, it'll be in the first page
    // However if boot2 is present, we need to search the second page
    vector<uint32_t> buffer = access.read_vector<uint32_t>(base, 128);
    for (uint i = 0; i < search_size; i++) {
        if (buffer[i] == BINARY_INFO_MARKER_START) {
            try {
                if (i + 4 < search_size && buffer[i + 4] == BINARY_INFO_MARKER_END) {
                    uint32_t from = buffer[i + 1];
                    uint32_t to = buffer[i + 2];
                    if (to > from && (from % 4) == 0 && (to % 4) == 0) {
                        access.read_into_vector(from, (to - from) / 4, hdr.bi_addr);
                        uint32_t cpy_table = buffer[i + 3];
                        vector<uint32_t> mapping;
                        do {
                            mapping = access.read_vector<uint32_t>(cpy_table, 3);
                            if (!mapping[0])
                                break;
                            // from, to_start, to_end
                            hdr.reverse_copy_mapping.insert(range(mapping[1], mapping[2]), mapping[0]);
                            cpy_table += 12;
                        } while (hdr.reverse_copy_mapping.size() < 10);  // arbitrary max
                        return true;
                    }
                }
            } catch (not_mapped_exception &e) {
                // Ignore not mapped exception, that means we tried to load an invalid address
                // Probably tried to decode invalid binary info
            }
        }
    }
    return false;
}

string read_string(memory_access &access, uint32_t addr) {
    const uint max_length = 512;
    auto v = access.read_vector<char>(addr, max_length, true);  // zero fill
    uint length;
    for (length = 0; length < max_length; length++) {
        if (!v[length]) {
            break;
        }
    }
    return string(v.data(), length);
}

struct bi_visitor_base {
    void visit(memory_access &access, const binary_info_header &hdr) {
        for (const auto &a : hdr.bi_addr) {
            visit(access, a);
        }
    }

    void visit(memory_access &access, uint32_t addr) {
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
            }
            else if (type == BI_PINS_ENCODING_MULTI) {
                uint32_t mask = 0;
                int last = -1;
                uint work = value.pin_encoding >> 7u;
                for (int i = 0; i < 5; i++) {
                    int cur = (int) (work & 0x1fu);
                    mask |= 1u << cur;
                    if (cur == last)
                        break;
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
            named_group(value.core.tag, value.parent_id, value.group_tag, value.group_id,
                        read_string(access, value.label), value.flags);
            break;
        }
        default:
            unknown(access, bi, addr);
        }
    }

    virtual void unknown(memory_access &access, const binary_info_core_t &bi_core, uint32_t addr) {}
    virtual void id_and_value(int tag, uint32_t id, uint32_t value) {
        //        printf("ID=0x%08x int value=%d 0x%08x\n", id, value, value);
    }
    virtual void id_and_string(int tag, uint32_t id, const string &value) {
        //        printf("ID=0x%08x int value=%s\n", id, value.c_str());
    }
    virtual void block_device(memory_access &access, binary_info_block_device_t &bi_bdev) {}

    virtual void pins(uint32_t pin_mask, int func, string name) {
        if (func != -1) {
            if (func >= (int) pin_functions.size())
                return;
        }
        for (uint i = 0; i < 30; i++) {
            if (pin_mask & (1u << i)) {
                if (func != -1) {
                    pin(i, pin_functions[func][i]);
                }
                else {
                    auto sep = name.find_first_of('|');
                    auto cur = name.substr(0, sep);
                    if (cur.empty())
                        continue;
                    pin(i, cur.c_str());
                    if (sep != string::npos) {
                        name = name.substr(sep + 1);
                    }
                }
            }
        }
    }

    virtual void pin(uint i, const string &name) {}

    virtual void zero_terminated_bi_list(memory_access &access, const binary_info_core_t &bi_core, uint32_t addr) {
        uint32_t bi_addr;
        access.read_raw<uint32_t>(addr, bi_addr);
        while (bi_addr) {
            visit(access, addr);
            access.read_raw<uint32_t>(addr, bi_addr);
        }
    }

    virtual void named_group(int parent_tag, uint32_t parent_id, int group_tag, uint32_t group_id, const string &label,
                             uint flags) {}
};

struct bi_visitor : public bi_visitor_base {
    typedef std::function<void(int tag, uint32_t id, uint32_t value)> id_and_int_fn;
    typedef std::function<void(int tag, uint32_t id, const string &value)> id_and_string_fn;
    typedef std::function<void(uint num, const string &label)> pin_fn;
    typedef std::function<void(int parent_tag, uint32_t parent_id, int group_tag, uint32_t group_id,
                               const string &label, uint flags)>
        named_group_fn;
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
        if (_id_and_int)
            _id_and_int(tag, id, value);
    }

    void id_and_string(int tag, uint32_t id, const string &value) override {
        if (_id_and_string)
            _id_and_string(tag, id, value);
    }

    void pin(uint i, const string &name) override {
        if (_pin)
            _pin(i, name);
    }

    void named_group(int parent_tag, uint32_t parent_id, int group_tag, uint32_t group_id, const string &label,
                     uint flags) override {
        if (_named_group)
            _named_group(parent_tag, parent_id, group_tag, group_id, label, flags);
    }

    void block_device(memory_access &access, binary_info_block_device_t &bi_bdev) override {
        if (_block_device)
            _block_device(access, bi_bdev);
    }
};

#pragma GCC diagnostic pop

std::string intToIp(uint32_t ipWord) {
    std::stringstream ss;
    for (int i = 0; i < 4; i++) {
        ss << (ipWord & 0xFF);
        ipWord >>= 8;
        if (i < 3)
            ss << ".";
    }
    return ss.str();
}

void extractAppInfo(memory_access &raw_access, AppInfo &appData, uint32_t base) {
    try {
        binary_info_header hdr;
        if (find_binary_info(raw_access, hdr, base)) {
            auto access = remapped_memory_access(raw_access, hdr.reverse_copy_mapping);
            auto visitor = bi_visitor {};

            // do a pass first to find named groups
            visitor.named_group([&](int parent_tag, uint32_t parent_id, int group_tag, uint32_t group_id,
                                    const string &label, uint flags) {
                if (parent_tag != BINARY_INFO_TAG_RASPBERRY_PI)
                    return;
                if (parent_id != BINARY_INFO_ID_RP_PROGRAM_FEATURE)
                    return;
                appData.namedFeatureGroups[std::make_pair(group_tag, group_id)] = std::make_pair(label, flags);
            });

            visitor.visit(access, hdr);

            // Do full pass
            visitor = bi_visitor {};
            visitor.id_and_int([&](int tag, uint32_t id, uint32_t value) {
                if (tag == BINARY_INFO_TAG_UWRT) {
                    if (id == BINARY_INFO_ID_UW_BOOTLOADER_ENABLED)
                        appData.isBootloader = !!value;
                    if (id == BINARY_INFO_ID_UW_APPLICATION_BASE)
                        appData.blAppBase = value;
                    if (id == BINARY_INFO_ID_UW_DEVICE_IP_ADDRESS)
                        appData.deviceIpAddress = intToIp(value);
                    if (id == BINARY_INFO_ID_UW_AGENT_IP_ADDRESS)
                        appData.agentIpAddress = intToIp(value);
                    if (id == BINARY_INFO_ID_UW_AGENT_PORT)
                        appData.agentPort = value;
                    if (id == BINARY_INFO_ID_UW_CLIENT_ID)
                        appData.clientIds.push_back(value);
                }

                if (tag != BINARY_INFO_TAG_RASPBERRY_PI)
                    return;
                if (id == BINARY_INFO_ID_RP_BINARY_END)
                    appData.binaryEnd = value;
            });
            visitor.id_and_string([&](int tag, uint32_t id, const string &value) {
                const auto &nfg = appData.namedFeatureGroups.find(std::make_pair(tag, id));
                if (nfg != appData.namedFeatureGroups.end()) {
                    appData.namedFeatureGroupValues[nfg->second.first].push_back(value);
                    return;
                }

                if (tag != BINARY_INFO_TAG_RASPBERRY_PI)
                    return;
                if (id == BINARY_INFO_ID_RP_PROGRAM_NAME)
                    appData.programName = value;
                else if (id == BINARY_INFO_ID_RP_PROGRAM_VERSION_STRING)
                    appData.programVersion = value;
                else if (id == BINARY_INFO_ID_RP_PROGRAM_BUILD_DATE_STRING)
                    appData.programBuildDate = value;
                else if (id == BINARY_INFO_ID_RP_PROGRAM_URL)
                    appData.programUrl = value;
                else if (id == BINARY_INFO_ID_RP_PROGRAM_DESCRIPTION)
                    appData.programDescription = value;
                else if (id == BINARY_INFO_ID_RP_PROGRAM_FEATURE)
                    appData.programFeatures.push_back(value);
                else if (id == BINARY_INFO_ID_RP_PROGRAM_BUILD_ATTRIBUTE)
                    appData.buildAttributes.push_back(value);
                else if (id == BINARY_INFO_ID_RP_PICO_BOARD)
                    appData.boardType = value;
                else if (id == BINARY_INFO_ID_RP_SDK_VERSION)
                    appData.sdkVersion = value;
                else if (id == BINARY_INFO_ID_RP_BOOT2_NAME)
                    appData.boot2Name = value;
            });
            visitor.pin([&](uint pin, const string &name) { appData.pins[pin].push_back(name); });

            visitor.visit(access, hdr);
        }
    } catch (not_mapped_exception &) {
        // Ignore memory read failure, just let whatever is found return
    }
}

void extractAppInfo(AppImage &uf2, AppInfo &appData, uint32_t base) {
    auto raw_access = uf2_memory_access(uf2);
    extractAppInfo(raw_access, appData, base);
}

void extractAppInfo(RP2040FlashInterface &itf, AppInfo &appData, uint32_t base) {
    auto raw_access = device_memory_access(itf);
    appData.binaryStart = base;
    extractAppInfo(raw_access, appData, base);
}

void reportVersionInfo(std::vector<std::pair<std::string, std::string>> &infoOut, AppInfo &firstApp,
                       AppInfo &nestedApp) {
    // If has bootloader, report it
    if (firstApp.isBootloader) {
        // Show bootloader version if present
        if (firstApp.programName.size() > 0) {
            if (firstApp.programVersion.size() > 0) {
                infoOut.emplace_back("BL Version", firstApp.programName + " " + firstApp.programVersion);
            }
            else {
                infoOut.emplace_back("BL Name", firstApp.programName);
            }
        }

        // Show app version if present
        if (nestedApp.programName.size() > 0) {
            if (nestedApp.programVersion.size() > 0) {
                infoOut.emplace_back("App Version", nestedApp.programName + " " + nestedApp.programVersion);
            }
            else {
                infoOut.emplace_back("App Name", nestedApp.programName);
            }
        }
    }
    // If not, its either app version or it doesn't have UF2 data
    else {
        if (firstApp.programName.size() > 0) {
            if (firstApp.programVersion.size() > 0) {
                infoOut.emplace_back("App Version", firstApp.programName + " " + firstApp.programVersion);
            }
            else {
                infoOut.emplace_back("App Name", firstApp.programName);
            }
        }
    }
}

}  // namespace BinaryInfo
