/*
 * Copyright (c) 2024 by Bert Laverman. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <boards/pico.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>

#include <cstdint>
#include <random>
#include <vector>
#include <array>
#include <list>

#include <raspberry-pi.hpp>
#include <interfaces/pico-spi.hpp>

#include <components/local-led.hpp>

#include <devices/local-max7219.hpp>


using namespace nl::rakis::raspberrypi;
using namespace nl::rakis::raspberrypi::interfaces;
using namespace nl::rakis::raspberrypi::devices;



#if !defined(TARGET_PICO)
#error "This example is for the Raspberry Pi Pico only"
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

static void runTestPattern(RaspberryPi& berry, std::shared_ptr<LocalMAX7219> max)
{
    printf("Clearing display.\n");
    max->writeImmediately(false);
    max->clear();
    max->sendData();

    printf("Running first test pattern.\n");
    for (unsigned n1 = 0; n1 < 8; n1++) {
        for (uint8_t mod = 0; mod < max->numDevices(); mod++) {
            max->setNumber(mod, testValues1[n1]);
        }
        max->sendData();
        berry.sleepMs(300);
    }

    printf("Running second test pattern.\n");
    for (unsigned n2 = 0; n2 < 8; n2++) {
        for (uint8_t mod = 0; mod < max->numDevices(); mod++) {
            max->setBuffer(mod, testValues2[n2]);
        }
        max->sendData();
        berry.sleepMs(300);
    }
    max->clear();
    max->sendData();

    printf("Display tests done.\n");
}


static void errorExit(RaspberryPi& berry, components::LocalLed& led, unsigned numBlips) {
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


static constexpr uint8_t NUM_MODULES = 3;

int main([[maybe_unused]] int argc, [[maybe_unused]] char*argv[])
{
    RaspberryPi& berry(RaspberryPi::instance());

    stdio_init_all();
    berry.sleepMs(1000);

    printf("Starting up.\n");

    components::LocalLed internalLed(berry, PICO_DEFAULT_LED_PIN);


    try {
        auto spi = berry.addSPI<PicoSPI>("pico-spi-0");
        // spi->verbose(true);
        spi->baudRate(500000);

        auto max = std::make_shared<devices::LocalMAX7219>();
        spi->device(max);

        max->numDevices(NUM_MODULES);
        max->reset();
        berry.sleepMs(500);

        runTestPattern(berry, max);

        unsigned numBlips = 0;
        while (true) {
            if (numBlips >= 50) {
                internalLed.toggle();
                numBlips = 0;
            }

            berry.sleepMs(10);
            numBlips++;
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
