#include "embedi_module_init.h"

extern int embedi_init$$Base;
extern int embedi_init$$Length;

void embedi_module_init(void)
{
    init_t *init_call = (init_t *)&embedi_init$$Base;
    unsigned char count = (int)(&embedi_init$$Length) / sizeof(init_t);

    while (count--) {
        (*init_call)();
        init_call++;
    }
}
