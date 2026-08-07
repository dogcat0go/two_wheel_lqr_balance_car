#include "shared_state.h"

#include <Arduino.h>

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static ControlSnapshot s_snapshot;
static CommandInput s_command;

void publishSnapshot(const ControlSnapshot& s)
{
    portENTER_CRITICAL(&s_mux);
    s_snapshot = s;
    portEXIT_CRITICAL(&s_mux);
}

ControlSnapshot fetchSnapshot()
{
    portENTER_CRITICAL(&s_mux);
    ControlSnapshot s = s_snapshot;
    portEXIT_CRITICAL(&s_mux);
    return s;
}

void publishCommand(const CommandInput& c)
{
    portENTER_CRITICAL(&s_mux);
    s_command = c;
    portEXIT_CRITICAL(&s_mux);
}

CommandInput fetchCommand()
{
    portENTER_CRITICAL(&s_mux);
    CommandInput c = s_command;
    portEXIT_CRITICAL(&s_mux);
    return c;
}
