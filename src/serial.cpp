#include <Arduino.h>
#include <queue>

extern HardwareSerial toRK3588Serial;

std::queue<std::vector<uint8_t>> send_queue;

void send(std::vector<uint8_t> data) {
    send_queue.push(data);
}

void serial_send_task() {
    if (!send_queue.empty()) {
        const std::vector<uint8_t>& data = send_queue.front();
        toRK3588Serial.write(data.data(), data.size());
        send_queue.pop();
    }
}