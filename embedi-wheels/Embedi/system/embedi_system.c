#include "embedi_module_init.h"

static int run_state;

int embedi_get_run_state(void)
{
    return run_state;
}

void embedi_set_run_state(int state)
{
    run_state = state;
}

void embedi_system_init(void)
{
    embedi_module_init();
}
