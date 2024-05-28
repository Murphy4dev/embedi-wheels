#ifndef __EMBEDI_MODULE_INIT_H
#define __EMBEDI_MODULE_INIT_H

typedef void (*init_t)(void);

#define module_init(fn) \
const init_t __embedi_##fn __attribute__((section("embedi_init")))  __attribute__((used)) = fn

void embedi_module_init(void);

#endif
