#ifndef SEGACD_H_
#define SEGACD_H_
#include <stdint.h>
#include "system.h"
#include "m68k_core.h"

typedef struct {
	m68k_context *m68k;
	system_media *media;
	uint16_t     gate_array[0x100];
	uint8_t      busreq;
	uint8_t      busack;
	uint8_t      reset;
	uint16_t     *rom;     //unaltered ROM, needed for mirrored locations
	uint16_t     *rom_mut; //ROM with low 16-bit of HINT vector modified by register write
	uint16_t     *prog_ram;
	uint16_t     *work_ram;
	uint8_t      *pcm_ram;
	uint8_t      *bram;
} segacd_context;

segacd_context *alloc_configure_segacd(system_media *media, uint32_t opts, uint8_t force_region, rom_info *info);

#endif //SEGACD_H_
