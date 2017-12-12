#include <stdlib.h>
#include <string.h>
#include "segacd.h"
#include "genesis.h"
#include "util.h"

static void *prog_ram_wp_write16(uint32_t address, void *vcontext, uint16_t value)
{
	return vcontext;
}

static void *prog_ram_wp_write8(uint32_t address, void *vcontext, uint8_t value)
{
	return vcontext;
}

static uint16_t work_ram_2M_read16(uint32_t address, void *vcontext)
{
	return 0;
}

static uint8_t work_ram_2M_read8(uint32_t address, void *vcontext)
{
	return 0;
}

static void *work_ram_2M_write16(uint32_t address, void *vcontext, uint16_t value)
{
	return vcontext;
}

static void *work_ram_2M_write8(uint32_t address, void *vcontext, uint8_t value)
{
	return vcontext;
}

static uint16_t work_ram_1M_read16(uint32_t address, void *vcontext)
{
	return 0;
}

static uint8_t work_ram_1M_read8(uint32_t address, void *vcontext)
{
	return 0;
}

static void *work_ram_1M_write16(uint32_t address, void *vcontext, uint16_t value)
{
	return vcontext;
}

static void *work_ram_1M_write8(uint32_t address, void *vcontext, uint8_t value)
{
	return vcontext;
}

static uint8_t pcm_read8(uint32_t address, void *vcontext)
{
	return 0;
}

static uint16_t pcm_read16(uint32_t address, void *vcontext)
{
	return 0xFF00 | pcm_read8(address+1, vcontext);
}

static void *pcm_write8(uint32_t address, void *vcontext, uint8_t value)
{
	return vcontext;
}

static void *pcm_write16(uint32_t address, void *vcontext, uint16_t value)
{
	return pcm_write8(address+1, vcontext, value);
}

static uint16_t sub_gate_read16(uint32_t address, void *vcontext)
{
	m68k_context *m68k = vcontext;
	segacd_context *cd = m68k->system;
	return cd->gate_array[(address & 0x1FF) >> 1];
}

static uint8_t sub_gate_read8(uint32_t address, void *vcontext)
{
	m68k_context *m68k = vcontext;
	segacd_context *cd = m68k->system;
	uint16_t val = cd->gate_array[(address & 0x1FF) >> 1];
	return address & 1 ? val : val >> 8;
}

static void *sub_gate_write16(uint32_t address, void *vcontext, uint16_t value)
{
	m68k_context *m68k = vcontext;
	segacd_context *cd = m68k->system;
	uint32_t reg = (address & 0x1FF) >> 1;
	switch (reg)
	{
	case 0x7:
		cd->gate_array[reg] &= 0xFF00;
		cd->gate_array[reg] |= value & 0xFF;
		break;
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
	case 0x14:
	case 0x15:
	case 0x16:
	case 0x17:
		//no effects for these other than saving the value
		cd->gate_array[reg] = value;
		break;
	default:
		printf("Unhandled gate array write %X:%X\n", address, value);
	}
	return vcontext;
}

static void *sub_gate_write8(uint32_t address, void *vcontext, uint8_t value)
{
	m68k_context *m68k = vcontext;
	segacd_context *cd = m68k->system;
	uint32_t reg = (address & 0x1FF) >> 1;
	uint16_t value16;
	if (address & 1) {
		value16 = cd->gate_array[reg] & 0xFF00 | value;
	} else {
		value16 = cd->gate_array[reg] & 0xFF | (value << 8);
	}
	return sub_gate_write16(address, vcontext, value16);
}

static uint16_t main_gate_read16(uint32_t address, void *vcontext)
{
	m68k_context *m68k = vcontext;
	segacd_context *cd = m68k->system;
	return cd->gate_array[(address & 0x1FF) >> 1];
}

static uint8_t main_gate_read8(uint32_t address, void *vcontext)
{
	m68k_context *m68k = vcontext;
	segacd_context *cd = m68k->system;
	uint16_t val = cd->gate_array[(address & 0x1FF) >> 1];
	return address & 1 ? val : val >> 8;
}

static void *main_gate_write16(uint32_t address, void *vcontext, uint16_t value)
{
	m68k_context *m68k = vcontext;
	genesis_context *gen = m68k->system;
	segacd_context *cd = gen->expansion;
	uint32_t reg = (address & 0x1FF) >> 1;
	switch (reg)
	{
	case 0x7:
		cd->gate_array[reg] &= 0xFF;
		cd->gate_array[reg] |= value & 0xFF00;
		break;
	case 0x8:
	case 0x9:
	case 0xA:
	case 0xB:
	case 0xC:
	case 0xD:
	case 0xE:
	case 0xF:
		//no effects for these other than saving the value
		cd->gate_array[reg] = value;
		break;
	default:
		printf("Unhandled gate array write %X:%X\n", address, value);
	}
	return vcontext;
}

static void *main_gate_write8(uint32_t address, void *vcontext, uint8_t value)
{
	m68k_context *m68k = vcontext;
	genesis_context *gen = m68k->system;
	segacd_context *cd = gen->expansion;
	uint32_t reg = (address & 0x1FF) >> 1;
	uint16_t value16;
	if (address & 1) {
		value16 = cd->gate_array[reg] & 0xFF00 | value;
	} else {
		value16 = cd->gate_array[reg] & 0xFF | (value << 8);
	}
	return main_gate_write16(address, vcontext, value16);
}

segacd_context *alloc_configure_segacd(system_media *media, uint32_t opts, uint8_t force_region, rom_info *info)
{
	static memmap_chunk sub_cpu_map[] = {
		{0x000000, 0x00FEFF, 0x0000, .flags=MMAP_READ | MMAP_CODE, .write_16 = prog_ram_wp_write16, .write_8 = prog_ram_wp_write8},
		{0x00FF00, 0x07FFFF, 0x0000, .flags=MMAP_READ | MMAP_WRITE | MMAP_CODE},
		{0x080000, 0x0BFFFF, 0x0000, .flags=MMAP_READ | MMAP_WRITE | MMAP_CODE | MMAP_PTR_IDX | MMAP_FUNC_NULL, .ptr_index = 0,
			.read_16 = work_ram_2M_read16, .write_16 = work_ram_2M_write16, .read_8 = work_ram_2M_read8, .write_8 = work_ram_2M_write8},
		{0x0C0000, 0x0DFFFF, 0x0000, .flags=MMAP_READ | MMAP_WRITE | MMAP_CODE | MMAP_PTR_IDX | MMAP_FUNC_NULL, .ptr_index = 1,
			.read_16 = work_ram_1M_read16, .write_16 = work_ram_1M_write16, .read_8 = work_ram_1M_read8, .write_8 = work_ram_1M_write8},
		{0xFE0000, 0xFEFFFF, 0x3FFF, .flags=MMAP_READ | MMAP_WRITE | MMAP_ONLY_ODD},
		{0xFF0000, 0xFF7FFF, 0x0000, .read_16 = pcm_read16, .write_16 = pcm_write16, .read_8 = pcm_read8, .write_8 = pcm_write8},
		{0xFF8000, 0xFF81FF, 0x0000, .read_16 = sub_gate_read16, .write_16 = sub_gate_write16, .read_8 = sub_gate_read8, .write_8 = sub_gate_write8}
	};
	segacd_context *cd = calloc(sizeof(segacd_context), 1);
	FILE *f = fopen("cdbios.bin", "rb");
	if (!f) {
		fatal_error("Failed to open CD firmware for reading");
	}
	long firmware_size = file_size(f);
	uint32_t adjusted_size = nearest_pow2(firmware_size);
	cd->rom = malloc(adjusted_size);
	if (firmware_size != fread(cd->rom, 1, firmware_size, f)) {
		fatal_error("Failed to read CD firmware");
	}
	cd->rom_mut = malloc(adjusted_size);
	memcpy(cd->rom_mut, cd->rom, adjusted_size);
	byteswap_rom(firmware_size, cd->rom);
	cd->prog_ram = malloc(512*1024);
	cd->work_ram = malloc(256*1024);
	cd->pcm_ram = malloc(64*1024);
	//TODO: Load state from file
	cd->bram = malloc(8*1024);
	
	sub_cpu_map[0].buffer = sub_cpu_map[1].buffer = cd->prog_ram;
	sub_cpu_map[4].buffer = cd->bram;
	m68k_options *mopts = malloc(sizeof(m68k_options));
	init_m68k_opts(mopts, sub_cpu_map, sizeof(sub_cpu_map) / sizeof(*sub_cpu_map), 4);
	cd->m68k = init_68k_context(mopts, NULL);
	cd->m68k->system = cd;
	cd->busreq = 1;
	cd->busack = 1;
	
	return cd;
}

memmap_chunk *segacd_main_cpu_map(segacd_context *cd, uint32_t *num_chunks)
{
	static memmap_chunk main_cpu_map[] = {
		{0x000000, 0x01FFFF, 0x00000, .flags=MMAP_READ},
		{0x020000, 0x03FFFF, 0x1FFFF, .flags=MMAP_READ|MMAP_WRITE|MMAP_PTR_IDX|MMAP_FUNC_NULL, .ptr_index = 0},//TODO: support running main CPU code from here
		{0x040000, 0x05FFFF, 0x1FFFF, .flags=MMAP_READ}, //first ROM alias
		//TODO: additional ROM/prog RAM aliases
		{0x200000, 0x01FFFF, 0x1FFFF, .flags=MMAP_READ|MMAP_WRITE|MMAP_PTR_IDX|MMAP_FUNC_NULL, .ptr_index = 1},
		{0x220000, 0x03FFFF, 0x1FFFF, .flags=MMAP_READ|MMAP_WRITE|MMAP_PTR_IDX|MMAP_FUNC_NULL, .ptr_index = 2},
	};
	//TODO: support cart boot maps
	//TODO: support BRAM cart
	main_cpu_map[0].buffer = cd->rom_mut;
	main_cpu_map[2].buffer = cd->rom;
	main_cpu_map[1].buffer = cd->prog_ram;
	main_cpu_map[3].buffer = cd->work_ram;
	main_cpu_map[3].buffer = cd->work_ram + 0x10000;
	*num_chunks = sizeof(main_cpu_map) / sizeof(*main_cpu_map);
	return main_cpu_map;
}
