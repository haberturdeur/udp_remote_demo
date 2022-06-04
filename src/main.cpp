#include "RBControl.hpp" // for encoders
#include "WiFi.h"
#include <Arduino.h> //; from Roboruka
#include <esp_log.h>
#include <string>
using namespace rb;

using namespace std;

bool debug = false;
bool g_handLocked = false;
bool g_leftBatteryState = false;
bool g_rightBatteryState = false;

constexpr rb::MotorId leftMotor = rb::MotorId::M1;
constexpr rb::MotorId rightMotor = rb::MotorId::M2;

constexpr std::size_t bufferSize = 64;

constexpr size_t axisOpCode = 0x80;
constexpr size_t axisCount = 6;
constexpr size_t xAxisPosition = 2;
constexpr size_t yAxisPosition = 3;
constexpr size_t armAxisPosition = 5;

constexpr size_t buttonOpCode = 0x81;
constexpr size_t buttonCount = 12;
constexpr size_t buttonIdPosition = 1;
constexpr size_t buttonStatePosition = 2;

constexpr uint8_t armId = 0;
const rb::Angle armDown = 0_deg;
const rb::Angle armUp = 90_deg;

constexpr uint8_t handId = 1;
const rb::Angle handOpened = 70_deg;
const rb::Angle handClosed = 0_deg;

constexpr uint8_t leftBatteryId = 2;
const rb::Angle leftBatteryDown = 0_deg;
const rb::Angle leftBatteryUp = 90_deg;

constexpr uint8_t rightBatteryId = 3;
const rb::Angle rightBatteryDown = 0_deg;
const rb::Angle rightBatteryUp = 90_deg;

void handleAxes(const char buffer[bufferSize]);
void handleButton(const char buffer[bufferSize]);

void setup() {
    auto& man = rb::Manager::get();

    man.install();
    man.initSmartServoBus(4, GPIO_NUM_14);

    if (!man.expander().digitalRead(SW3)) {
        man.leds().green(true);

        vTaskDelay(500 / portTICK_PERIOD_MS);

        bool up = false;

        while (man.expander().digitalRead(SW3)) {
            while (man.expander().digitalRead(SW1)) {
                vTaskDelay(10);
            }
            up = !up;

            if (up) {
                man.servoBus().set(armId, armUp);
                man.servoBus().set(handId, handOpened);
                man.servoBus().set(leftBatteryId, leftBatteryUp);
                man.servoBus().set(rightBatteryId, rightBatteryUp);
            } else {
                man.servoBus().set(armId, armDown);
                man.servoBus().set(handId, handClosed);
                man.servoBus().set(leftBatteryId, leftBatteryDown);
                man.servoBus().set(rightBatteryId, rightBatteryDown);
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        man.leds().green(false);
    }

    if (!man.expander().digitalRead(SW2)) {
        man.leds().blue(true);

        std::array<uint8_t, 4> ids = { handId, armId, rightBatteryId, leftBatteryId };

        for (auto&& i : ids) {
            man.leds().green(true);
            while (man.expander().digitalRead(SW1)) {
                vTaskDelay(10);
            }
            man.leds().green(false);
            man.servoBus().setId(i);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        man.leds().blue(false);
    }
    // man.servoBus().setId(armId);
    // man.servoBus().limit(armId, armDown, armUp);

    // WiFi.begin("Technika", "materidouska");
    WiFi.softAP("falcon", "LongLiveMedvedice");
    printf("%s\n", WiFi.softAPIP().toString().c_str());
    man.leds().yellow(true);

    WiFiUDP udp;

    udp.begin(80);

    int count = 0;

    while (true) {
        char buffer[bufferSize] = { 0 };
        memset(buffer, 0, bufferSize);

        udp.parsePacket();
        if ((count = udp.read(buffer, bufferSize - 1)) > 0) {
            // fwrite(buffer, 1, count, stdout);
            // fflush(stdout);

            if (buffer[0] == axisOpCode) {
                handleAxes(buffer);
            } else if (buffer[0] == buttonOpCode) {
                handleButton(buffer);
            } else {
                man.leds().red(true);

                if (debug) {
                    man.motor(leftMotor).stop();
                    man.motor(rightMotor).stop();
                    do {
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    } while (!man.expander().digitalRead(rb::EB0));
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);

                man.leds().red(false);
            }
        }

        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// extern "C" void app_main() {
//     setup();
// }

void loop() {
}

void handleAxes(const char buffer[bufferSize]) {
    auto& man = rb::Manager::get();

    if (buffer[1] != axisCount) {
        ESP_LOGE("UDP Parser", "Wrong axis count");
    }

    int x = static_cast<int8_t>(buffer[xAxisPosition]);
    int y = -static_cast<int8_t>(buffer[yAxisPosition]);

    int r = ((y - (x / 1.5f)));
    int l = ((y + (x / 1.5f)));

    r = rb::clamp(r, -50, 50);
    l = rb::clamp(l, -50, 50);

    if (r < 0 && l < 0) {
        std::swap(r, l);
    }

    man.motor(leftMotor).power(-l);
    man.motor(rightMotor).power(-r);

    if (g_leftBatteryState) {
        int leftBatteryAngle = static_cast<int8_t>(buffer[armAxisPosition]);
        leftBatteryAngle = map(leftBatteryAngle, -128, 128, leftBatteryDown.deg(), leftBatteryUp.deg());
        leftBatteryAngle = rb::clamp(leftBatteryAngle, (int)leftBatteryDown.deg(), (int)leftBatteryUp.deg());

        man.servoBus().set(leftBatteryId, Angle::deg(leftBatteryAngle));
    }

    if (g_rightBatteryState) {
        int rightBatteryAngle = static_cast<int8_t>(buffer[armAxisPosition]);
        rightBatteryAngle = map(rightBatteryAngle, -128, 128, rightBatteryDown.deg(), rightBatteryUp.deg());
        rightBatteryAngle = rb::clamp(rightBatteryAngle, (int)rightBatteryDown.deg(), (int)rightBatteryUp.deg());

        man.servoBus().set(rightBatteryId, Angle::deg(rightBatteryAngle));
    }

    if (!g_leftBatteryState && !g_rightBatteryState) {
        int armAngle = static_cast<int8_t>(buffer[armAxisPosition]);
        armAngle = map(armAngle, -128, 128, armDown.deg(), armUp.deg());
        armAngle = rb::clamp(armAngle, (int)armDown.deg(), (int)armUp.deg());

        man.servoBus().set(armId, Angle::deg(armAngle));
    }
    // if (static_cast<int8_t>(buffer[armAxisPosition]) > 0) {
    //     man.servoBus().set(armId, armDown);
    // }
    // else
    //     man.servoBus().set(armId, armUp);
}

void handleButton(const char buffer[bufferSize]) {
    auto& man = rb::Manager::get();

    size_t id = buffer[buttonIdPosition];
    uint8_t state = buffer[buttonStatePosition];

    if (id >= buttonCount) {
        ESP_LOGE("UDP Parser", "Button id out of bounds");
    }

    ESP_LOGI("UDP Parser", "Button %u changed to %u\n", id, state);
    switch (id) {
    case 0:
        if (state) {
            g_handLocked = false;
            man.servoBus().set(handId, handClosed);
        } else {
            if (!g_handLocked)
                man.servoBus().set(handId, handOpened);
        }
        break;
    case 1:
        g_handLocked = true;
        break;

    case 6:
        g_leftBatteryState = state;

    case 7:
        g_rightBatteryState = state;
    default:
        break;
    }
}
