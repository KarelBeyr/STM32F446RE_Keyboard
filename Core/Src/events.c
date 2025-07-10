#include "events.h"

static AppEvent event_queue[EVENT_QUEUE_SIZE];
static uint8_t head = 0;
static uint8_t tail = 0;

void event_queue_init(void) {
    head = tail = 0;
}

bool event_queue_push(AppEvent event) {
    uint8_t next = (head + 1) % EVENT_QUEUE_SIZE;
    if (next == tail) return false;  // Queue full
    event_queue[head] = event;
    head = next;
    return true;
}

bool event_queue_pop(AppEvent* out_event) {
    if (head == tail) return false;  // Queue empty
    *out_event = event_queue[tail];
    tail = (tail + 1) % EVENT_QUEUE_SIZE;
    return true;
}
