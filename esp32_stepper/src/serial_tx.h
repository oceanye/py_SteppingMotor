#pragma once

// Initialize the shared USB-serial transmit mutex. Call once from setup()
// immediately after Serial.begin(), before any worker task is started.
bool serial_tx_init();

// Emit exactly one CRLF-terminated line while holding the shared transmit
// mutex. Callers must assemble the complete line before entering this API.
void serial_tx_line(const char* line);

// Format a complete line into a bounded local buffer, then emit it atomically.
void serial_tx_printf(const char* format, ...)
    __attribute__((format(printf, 1, 2)));
