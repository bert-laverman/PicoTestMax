// Copyright 2024 Bert Laverman, All Rights Reserved.
// Created: 2024-08-06

#include <boards/pico.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>

#include <cstdint>
#include <random>
#include <vector>
#include <array>
#include <list>

#include <pico.hpp>

#include <devices/led.hpp>
#include <devices/local-max7219.hpp>
#include <protocols/max7219-messages.hpp>
#include <protocols/max7219-handler.hpp>

#include <protocols/pico-i2c-protocol-driver.hpp>


using namespace nl::rakis::raspberrypi;
using namespace nl::rakis::raspberrypi::interfaces;
using namespace nl::rakis::raspberrypi::protocols;
using namespace nl::rakis::raspberrypi::devices;



#if !defined(TARGET_PICO)
#error "This example is for the Raspberry Pi Pico only"
#endif
#if !defined(HAVE_I2C)
#error "This example needs I2C support enabled"
#endif
#if !defined(HAVE_SPI)
#error "This example needs SPI support enabled"
#endif
#if !defined(HAVE_MAX7219)
#error "This example needs MAX7219 support enabled"
#endif


static std::array<int, 8> testValues1 = { 1, 20, 300, 4000, 50000, 600000, 7000000, 80000000 };

static std::array<std::array<uint8_t, devices::MAX7219_DIGITS>, 8> testValues2 = {{
    {{ 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08 }},
    {{ 0x0a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0a }},
    {{ 0x0f, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0f }},
    {{ 0x0f, 0x0a, 0x08, 0x08, 0x08, 0x08, 0x0a, 0x0f }},
    {{ 0x0f, 0x0f, 0x08, 0x08, 0x08, 0x08, 0x0f, 0x0f }},
    {{ 0x0f, 0x0f, 0x0a, 0x08, 0x08, 0x0a, 0x0f, 0x0f }},
    {{ 0x0f, 0x0f, 0x0f, 0x08, 0x08, 0x0f, 0x0f, 0x0f }},
    {{ 0x0f, 0x0f, 0x0f, 0x0a, 0x0a, 0x0f, 0x0f, 0x0f }}
}};

static void runTestPattern(PICO& berry, LocalMAX7219& max, uint8_t numModules)
{
    printf("Clearing display.\n");
    max.writeImmediately(false);
    max.clear();
    max.sendData();

    printf("Running first test pattern.\n");
    for (unsigned n1 = 0; n1 < 8; n1++) {
        for (uint8_t mod = 0; mod < numModules; mod++) {
            max.setNumber(mod, testValues1[n1]);
        }
        max.sendData();
        berry.sleepMs(1000);
    }

    printf("Running second test pattern.\n");
    for (unsigned n2 = 0; n2 < 8; n2++) {
        for (uint8_t mod = 0; mod < numModules; mod++) {
            max.setBuffer(mod, testValues2[n2]);
        }
        max.sendData();
        berry.sleepMs(300);
    }
    max.clear();
    max.sendData();

    printf("Display tests done.\n");
}


static void errorExit(PICO& berry, Led& led, unsigned numBlips) {
    while (true) {
        for (unsigned i = 0; i < numBlips; i++) {
            led.on();
            berry.sleepMs(500);
            led.off();
            berry.sleepMs(500);
        }

        berry.sleepMs(1000);
    }
}

static BoardId myBoardId;

static constexpr uint8_t NUM_MODULES = 3;

int main([[maybe_unused]] int argc, [[maybe_unused]] char*argv[])
{
    PICO& berry(PICO::instance(false));

    stdio_init_all();
    berry.sleepMs(1000);

    printf("Starting up.\n");
    pico_unique_board_id_t myId;
    pico_get_unique_board_id(&myId);
    for (unsigned i = 0; i < 8; i++) { myBoardId.bytes[i] = myId.id[i]; }
    printf("My board ID is %02x%02x%02x%02x-%02x%02x%02x%02x\n",
        myId.id[0], myId.id[1], myId.id[2], myId.id[3], myId.id[4], myId.id[5], myId.id[6], myId.id[7]);

    Led internalLed(PICO_DEFAULT_LED_PIN);

    try {
        berry.addInterface(interfaces::PicoSPI());
        LocalMAX7219 max(berry.spi());

        berry.spi().numModules(NUM_MODULES);
        max.reset();
        berry.sleepMs(5000);

        runTestPattern(berry, max, NUM_MODULES);

        printf("Setting up communication buses.\n");
        PicoI2C pico2piBus(i2c0, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
        PicoI2C pi2picoBus(i2c1, 14, 15);

        uint8_t controllerAddress{ 0x00 };
        uint8_t myAddress{ 0x00 };

        PicoI2CProtocolDriver driver(pico2piBus, pi2picoBus);

        printf("At first listening for general calls only.\n");
        driver.enableControllerMode();
        driver.enableResponderMode(myAddress);

        MAX7219Handler maxHandler(max);
        while (true) {
            internalLed.toggle();

            Command command;
            uint8_t sender;
            std::vector<uint8_t> data;
            while (driver.popMessage(command, sender, data)) {
                switch (command) {
                case Command::Hello:
                if (data.size() == sizeof(protocols::MsgHello)) {
                    auto msg = reinterpret_cast<const protocols::MsgHello*>(data.data());
                    if ((controllerAddress == 0) && (msg->boardId.id == protocols::ControllerId)) {
                        printf("We have a controller at address 0x%02x.\n", sender);
                        controllerAddress = sender;
                    }
                } else {
                    printf("Invalid payload size for Hello command\n");
                }
                break;

                case Command::SetAddress:
                    printf("MsgSetAddress received.\n");
                    if (data.size() == sizeof(MsgSetAddress)) {
                        auto msg = reinterpret_cast<const MsgSetAddress*>(data.data());

                        if (msg->boardId.id == myBoardId.id) {
                            if (msg->address == driver.listenAddress()) {
                                // Ignore
                            }
                            else {
                                printf("Controller told us our address is 0x%02x.", msg->address);
                                myAddress = msg->address;
                                driver.disableResponderMode();
                                driver.enableResponderMode(myAddress);
                            }
                        }
                    } else {
                        printf("Invalid payload size for SetAddress command\n");
                    }
                    break;

                case Command::Max7219:
                    if (data.size() == sizeof(MsgMax7219)) {
                        auto msg = reinterpret_cast<MsgMax7219*>(data.data());
                        printf("MsgMax7219 received (command=0x%02x, module=0x%02x, value=%ld)\n", msg->command, msg->module, msg->value);
                        maxHandler.handle(*msg);
                    } else {
                        printf("Invalid payload size for Max7219 command\n");
                    }
                    break;

                default:
                    printf("Unknown message received (command=0x%02x)\n", toInt(command));
                    break;
                }
            }
            if ((myAddress == 0x00) && (controllerAddress != 0x00)) {
                // Please, can I have an address?
                if (!driver.sendHello(controllerAddress, myBoardId)) {
                    printf("Controller is not responding, will retry later on.\n");
                }
            }
            berry.sleepMs(500);
        }
    }
    catch (std::runtime_error& e) {
        printf("Runtime error: %s\n", e.what());
        errorExit(berry, internalLed, 2);
    }
    catch (...) {
        printf("Unknown exception caught\n");
        errorExit(berry, internalLed, 3);
    }
}
