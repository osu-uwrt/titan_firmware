#include <cxxabi.h>
#include <iostream>
#include <memory>
#include <vector>
#include <thread>
#include <net/if.h>

#include "UploadTool.hpp"
#include "titan/canmore.h"
#include "canmore_cpp/Discovery.hpp"
#include "canmore_cpp/BootloaderClient.hpp"
#include "canmore_cpp/RegMappedClient.hpp"
#include "pico_usb/USBDiscovery.hpp"

bool isValidCANDevice(const char *name) {
    return (name[0] == 'c' && name[1] == 'a' && name[2] == 'n' &&
            (name[3] >= '0' && name[3] <= '9'));
}

static const char* const positionalArgsNames[] = {"uf2"};

class CANBlToolArgs {
public:
    // Arguments
    bool waitInBootDelay;
    bool justPullInfo;
    bool allowBootloaderOverwrite;
    bool alwaysPromptForDev;
    bool forceOpenocd;
    bool showHelpAndQuit;
    const char *filename;
    const char *progname;

    bool parseSuccessful;

    CANBlToolArgs(int argc, char** argv):
            // Define default args
            waitInBootDelay(false), justPullInfo(false), allowBootloaderOverwrite(false), alwaysPromptForDev(false), forceOpenocd(false), showHelpAndQuit(false), filename(""),

            // Attributes
            progname("[???]"), parseSuccessful(false), argc(argc), argv(argv), positionalIndex(0) {

        // Do parse
        parseSuccessful = tryParse();
    }

    void printHelp() {
        std::cout << "Usage: " << progname << " [-fiopw] [uf2]" << std::endl;
        std::cout << "\t-h: Show this help message" << std::endl;
        std::cout << "\t-f: Full Image Flash (if omitted, uf2 is assumed ota file)" << std::endl;
        std::cout << "\t\tAllows flashing of images containing a bootloader rather than restricting to OTA" << std::endl;
        std::cout << "\t-i: Print Info" << std::endl;
        std::cout << "\t\tPrints information from the passed file and quits" << std::endl;
        std::cout << "\t-o: Force OpenOCD" << std::endl;
        std::cout << "\t\tForces uploading via OpenOCD, bypassing device discovery." << std::endl;
        std::cout << "\t\tUseful with UPLOADTOOL_OPENOCD_CUSTOM_INIT_SCRIPT to enable flashing with non-picoprobe devices." << std::endl;
        std::cout << "\t-p: Always Prompt for Device" << std::endl;
        std::cout << "\t\tDisable automatic device selection, and instead always prompts the user for which RP2040 to upload to" << std::endl;
        std::cout << "\t-w: Wait for Boot" << std::endl;
        std::cout << "\t\tPrompts to wait for CANmore device in boot" << std::endl;
        std::cout << "\tuf2: A UF2 file to flash" << std::endl;
        std::cout << std::endl;
        std::cout << "Environment Variables:" << std::endl;
        std::cout << "\tUPLOADTOOL_OPENOCD_PATH:" << std::endl;
        std::cout << "\t\tSet this to override the openocd exectuable path. Useful if using custom openocd version" << std::endl;
        std::cout << "\tUPLOADTOOL_OPENOCD_CUSTOM_INIT_SCRIPT:" << std::endl;
        std::cout << "\t\tSet this to override the openocd picoprobe initialization routine." << std::endl;
        std::cout << "\t\tUseful when combined with -o flag to run with non-picoprobe interfaces." << std::endl;
        std::cout << "\tUPLOADTOOL_OPENOCD_EN_STDERR:" << std::endl;
        std::cout << "\t\tSet to 1 to enable stderr on openocd subprocess, 0 to supress stderr output (Default)" << std::endl;
    }

private:
    int argc;
    char** argv;
    size_t positionalIndex;

    const char* tryGetArgument(const char *argname) {
        if (!argc) {
            std::cout << "Error: Expected argument '" << argname << "'" << std::endl;
            return NULL;
        }
        argc--;
        return *argv++;
    }

    bool tryParsePositional(const char *arg) {
        switch (positionalIndex) {
            case 0:
                filename = arg;
                break;
            default:
                std::cout << "Unexpected positional argument '" << arg << "'" << std::endl;
                return false;
        }
        positionalIndex++;
        return true;
    }


    bool tryParse() {
        const char* prognameTemp = tryGetArgument("progname"); \
        if (!prognameTemp) {return false;}
        progname = prognameTemp;

        // Loop until all positional arguments found (or help, which is special)
        while ((positionalIndex < (sizeof(positionalArgsNames)/sizeof(*positionalArgsNames)) || argc > 0) && !showHelpAndQuit) {
            const char* arg = tryGetArgument(positionalArgsNames[positionalIndex]);
            if (!arg) return false;

            // Check if flag
            if (arg[0] == '-') {
                if (arg[2] != '\0') {
                    std::cout << "Flag '" << arg << "' is not single character" << std::endl;
                    return false;
                }

                switch (arg[1]) {
                case 'f':
                    allowBootloaderOverwrite = true;
                    break;
                case 'i':
                    justPullInfo = true;
                    break;
                case 'o':
                    forceOpenocd = true;
                    break;
                case 'p':
                    alwaysPromptForDev = true;
                    break;
                case 'w':
                    waitInBootDelay = true;
                    break;
                case 'h':
                    showHelpAndQuit = true;
                    break;
                default:
                    std::cout << "Unexpected flag: -" << arg[1] << std::endl;
                    return false;
                }
            }
            // If not it's a positional argument
            else {
                if (!tryParsePositional(arg)) {
                    return false;
                }
            }
        }
        return true;
    }
};

int main(int argc, char** argv) {
    try {
        auto devMap = DeviceMap::create();

        // Pull arguments
        CANBlToolArgs blArgs(argc, argv);
        if (!blArgs.parseSuccessful) {
            std::cout << "Run '" << blArgs.progname << " -h' to show help." << std::endl;
            return 1;
        }
        if (blArgs.showHelpAndQuit) {
            blArgs.printHelp();
            return 0;
        }

        // Load in file
        UploadTool::RP2040UF2 uf2(blArgs.filename);

        // Command line arg overrides
        if (blArgs.justPullInfo) {
            UploadTool::dumpUF2(uf2);
            return 0;
        }

        if (blArgs.forceOpenocd) {
            auto itf = std::make_shared<PicoUSB::PicoprobeClient>("");
            UploadTool::flashImage(itf, uf2, !blArgs.allowBootloaderOverwrite);
            return 0;
        }

        // ===== Discover devices =====
        std::vector<std::shared_ptr<RP2040Discovery>> discoverySources;
        discoverySources.push_back(Canmore::EthernetDiscovery::create());
        discoverySources.push_back(PicoUSB::USBDiscovery::create());

        // Discover all CAN interfaces
        struct if_nameindex *if_nidxs, *intf;
        if_nidxs = if_nameindex();
        if (if_nidxs != NULL)
        {
            for (intf = if_nidxs; intf->if_index != 0 || intf->if_name != NULL; intf++)
            {
                if (isValidCANDevice(intf->if_name)) {
                    discoverySources.push_back(Canmore::CANDiscovery::create(intf->if_index));
                }
            }
            if_freenameindex(if_nidxs);
        }

        std::shared_ptr<RP2040FlashInterface> interface;
        if (blArgs.waitInBootDelay) {
            interface = UploadTool::catchInBootDelay(discoverySources, devMap, uf2);
        }
        else {
            // Wait for devices to appear
            std::cout << "Waiting for devices..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // Find all discovered devices
            std::vector<std::shared_ptr<RP2040Device>> discovered;
            for (auto discovery : discoverySources) {
                std::vector<std::shared_ptr<RP2040Device>> interfaceDiscovered;
                discovery->discoverDevices(interfaceDiscovered);
                discovered.insert(std::end(discovered), std::begin(interfaceDiscovered), std::end(interfaceDiscovered));
            }

            if (discovered.size() == 0) {
                std::cout << "No devices to select" << std::endl;
                return 1;
            }

            auto dev = UploadTool::selectDevice(discovered, devMap, uf2.boardType, !blArgs.alwaysPromptForDev);
            if (!dev) {
                return 1;
            }
            interface = dev->getFlashInterface();
        }

        if (UploadTool::flashImage(interface, uf2, !blArgs.allowBootloaderOverwrite)) {
            return 0;
        }
        else {
            return 1;
        }
    }
    catch (std::exception &e) {
        int status;
        char *exceptionName = abi::__cxa_demangle(abi::__cxa_current_exception_type()->name(), 0, 0, &status);
        // const char *exceptionName = typeid(i).name();
        std::cerr << std::endl << "[EXCEPTION] Exception '" << exceptionName << "' caused program termination" << std::endl;
        free(exceptionName);
        std::cerr << "  what():  " << e.what() << std::endl;
        return 255;
    }
    catch (...) {
        std::cerr << std::endl << "[EXCEPTION] Unknown exception caused program termination" << std::endl;
        return 255;
    }
}
