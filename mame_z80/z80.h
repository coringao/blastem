// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
#pragma once

#ifndef __Z80_H__
#define __Z80_H__

#include "../backend.h"

typedef struct
{
	cpu_options        gen;
	memmap_chunk const *iomap;
	uint32_t           io_chunks;
	uint32_t           io_address_mask;
} z80_options;

#define LSB_FIRST

typedef union
{
#ifdef LSB_FIRST
        struct { uint8_t l,h,h2,h3; } b;
        struct { uint16_t l,h; } w;
        struct { int8_t l,h,h2,h3; } sb;
        struct { int16_t l,h; } sw;
#else
        struct { uint8_t h3,h2,h,l; } b;
        struct { int8_t h3,h2,h,l; } sb;
        struct { uint16_t h,l; } w;
        struct { int16_t h,l; } sw;
#endif
        uint32_t d;
        int32_t sd;
} PAIR;

#define ZNUM_MEM_AREAS 4
typedef struct z80_device z80_device;
//typedefs for compatibility with existing BlastEm code
typedef z80_device z80_context;
typedef void (*z80_ctx_fun)(z80_context * context);

struct z80_device
{
	z80_options     *options;
	uint8_t *       mem_pointers[ZNUM_MEM_AREAS];
	void            *system;
	z80_ctx_fun     next_int_pulse;
	
	PAIR            m_prvpc;
	PAIR            m_pc;
	PAIR            m_sp;
	PAIR            m_af;
	PAIR            m_bc;
	PAIR            m_de;
	PAIR            m_hl;
	PAIR            m_ix;
	PAIR            m_iy;
	PAIR            m_wz;
	PAIR            m_af2;
	PAIR            m_bc2;
	PAIR            m_de2;
	PAIR            m_hl2;
	uint8_t           m_r;
	uint8_t           m_r2;
	uint8_t           m_iff1;
	uint8_t           m_iff2;
	uint8_t           m_halt;
	uint8_t           m_im;
	uint8_t           m_i;
	uint8_t           m_nmi_state;          /* nmi line state */
	uint8_t           m_nmi_pending;        /* nmi pending */
	uint8_t           m_irq_state;          /* irq line state */
	int             m_wait_state;         // wait line state
	int             busreq;        // bus request line state
	int             busack;        // bus ack line state
	int             reset;
	uint8_t           m_after_ei;           /* are we in the EI shadow? */
	uint8_t           m_after_ldair;        /* same, but for LD A,I or LD A,R */
	uint32_t          m_ea;

	int             m_icount;
	uint32_t          current_cycle;
	uint32_t          nmi_start;
	uint32_t          int_pulse_start;
	uint32_t          int_pulse_end;
	uint16_t          bank_reg;
	uint8_t           m_rtemp;
	uint8_t           int_is_nmi;
	uint8_t           im2_vector;
	const uint8_t *   m_cc_op;
	const uint8_t *   m_cc_cb;
	const uint8_t *   m_cc_ed;
	const uint8_t *   m_cc_xy;
	const uint8_t *   m_cc_xycb;
	const uint8_t *   m_cc_ex;
	uint8_t           *read_pointers[64/8];
	uint8_t           *write_pointers[64/8];
};

#define z80_invalidate_code_range(Z, S, E) 
#define z80_handle_code_write(A, Z)

void init_z80_opts(z80_options * options, memmap_chunk const * chunks, uint32_t num_chunks, memmap_chunk const * io_chunks, uint32_t num_io_chunks, uint32_t clock_divider, uint32_t io_address_mask);
z80_context *init_z80_context(z80_options *opts);
void z80_assert_reset(z80_context *z80, uint32_t cycle);
void z80_clear_reset(z80_context *z80, uint32_t cycle);
void z80_assert_busreq(z80_context *z80, uint32_t cycle);
void z80_clear_busreq(z80_context *z80, uint32_t cycle);
void z80_run(z80_context *z80, uint32_t target_cycle);
uint8_t z80_get_busack(z80_context * context, uint32_t cycle);
void z80_adjust_cycles(z80_context * context, uint32_t deduction);
void z80_serialize(z80_context *context, serialize_buffer *buf);
void z80_deserialize(deserialize_buffer *buf, void *vcontext);
void z80_options_free(z80_options *opts);
void z80_assert_nmi(z80_context *context, uint32_t cycle);

#endif /* __Z80_H__ */
