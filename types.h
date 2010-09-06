#ifndef TYPES_H
#define TYPES_H

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

#define hton16(x) (((((x) >> 8)) & 0xff) | (((x) & 0xff) << 8))
#define hton32(x) ((hton16(x & 0xffff) << 16) | hton16(x >> 16))
#define ntoh16(x) (((((x) >> 8)) & 0xff) | (((x) & 0xff) << 8))
#define ntoh32(x) ((ntoh16(x & 0xffff) << 16) | ntoh16(x >> 16))

#endif
