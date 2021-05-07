#pragma once

bool past_timeout(
    unsigned long current_time,
    unsigned long previous_time,
    unsigned long timeout) {
    return current_time - previous_time > timeout;
}