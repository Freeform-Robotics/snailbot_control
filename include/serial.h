#ifndef SERIAL_H
#define SERIAL_H

#include <vector>

void send(std::vector<uint8_t> data);
void serial_send_task(void);

#endif // SERIAL_H