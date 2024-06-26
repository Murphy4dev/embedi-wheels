#ifndef __EMBEDI_MODULE_INIT_H
#define __EMBEDI_MODULE_INIT_H

typedef void (*init_t)(void);

#define module_init(fn) \
const init_t __embedi_##fn __attribute__((section("embedi_init")))  __attribute__((used)) = fn

#define driver_init(fn) \
const init_t __embedi_##fn __attribute__((section("embedi_driver_init")))  __attribute__((used)) = fn

#define platform_init(fn) \
const init_t __embedi_##fn __attribute__((section("embedi_platform_init")))  __attribute__((used)) = fn

#define app_init(fn) \
const init_t __embedi_##fn __attribute__((section("embedi_app_init")))  __attribute__((used)) = fn

void embedi_module_init(void);

typedef void (*irq_cb_t)(int irq_num);

#define irq_register(fn) \
const irq_cb_t __embedi_##fn __attribute__((section("embedi_irq")))  __attribute__((used)) = fn
void embedi_irq_call(int irq_num);

#endif
