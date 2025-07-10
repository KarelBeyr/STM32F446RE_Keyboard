#ifndef EVENTS_H
#define EVENTS_H

#include <stdint.h>
#include <stdbool.h>

#define EVENT_QUEUE_SIZE 8

typedef enum {
    EVENT_NONE = 0,
    EVENT_KEY_PRESSED,
    EVENT_TIMER_TICK,
} AppEventType;

typedef struct {
    AppEventType type;
    uint8_t key;  // Optional, used for key press
} AppEvent;

void event_queue_init(void);
bool event_queue_push(AppEvent event);
bool event_queue_pop(AppEvent* out_event);

#endif
