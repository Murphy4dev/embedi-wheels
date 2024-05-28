#include "stdio.h"
#include "embedi_module_init.h"

extern int embedi_init$$Base;
extern int embedi_init$$Length;
extern int embedi_driver_init$$Base;
extern int embedi_driver_init$$Length;
extern int embedi_platform_init$$Base;
extern int embedi_platform_init$$Length;
extern int embedi_app_init$$Base;
extern int embedi_app_init$$Length;

void embedi_module_init(void)
{
    init_t *init_call = (init_t *)&embedi_init$$Base;
    init_t *init_driver_call = (init_t *)&embedi_driver_init$$Base;
    init_t *init_platform_call = (init_t *)&embedi_platform_init$$Base;
    init_t *init_app_call = (init_t *)&embedi_app_init$$Base;
    int count = 0;

    /*early --> late*/
    count = (int)(&embedi_platform_init$$Length) / sizeof(init_t);
    while (count--) {
        (*init_platform_call)();
        init_platform_call++;
    }
    printf("platform init \n");

    count = (int)(&embedi_init$$Length) / sizeof(init_t);
    while (count--) {
        (*init_call)();
        init_call++;
    }
    printf("module init \n");

    count = (int)(&embedi_driver_init$$Length) / sizeof(init_t);
    while (count--) {
        (*init_driver_call)();
        init_driver_call++;
    }
    printf("driver init \n");

    count = (int)(&embedi_app_init$$Length) / sizeof(init_t);
    while (count--) {
        (*init_app_call)();
        init_app_call++;
    }
    printf("app init \n");
}
