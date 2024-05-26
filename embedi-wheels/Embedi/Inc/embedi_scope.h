#ifndef __EMBEDI_SCOPE_H
#define __EMBEDI_SCOPE_H

enum {
    CHANNEL_1 = 1,
    CHANNEL_2 = 2,
    CHANNEL_3 = 3,
    CHANNEL_4 = 4,
    CHANNEL_5 = 5,
    CHANNEL_6 = 6,
    CHANNEL_7 = 7,
    CHANNEL_8 = 8,
    CHANNEL_9 = 9,
    CHANNEL_10 = 10
};
void embedi_data_to_scope(float data, unsigned char channel);
void embedi_scope_show(void);

#endif
