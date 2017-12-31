// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
/*****************************************************************************
 *
 *   z80.c
 *   Portable Z80 emulator V3.9
 *
 *   TODO:
 *    - Interrupt mode 0 should be able to execute arbitrary opcodes
 *    - If LD A,I or LD A,R is interrupted, P/V flag gets reset, even if IFF2
 *      was set before this instruction (implemented, but not enabled: we need
 *      document Z80 types first, see below)
 *    - WAIT only stalls between instructions now, it should stall immediately.
 *    - Ideally, the tiny differences between Z80 types should be supported,
 *      currently known differences:
 *       - LD A,I/R P/V flag reset glitch is fixed on CMOS Z80
 *       - OUT (C),0 outputs 0 on NMOS Z80, $FF on CMOS Z80
 *       - SCF/CCF X/Y flags is ((flags | A) & 0x28) on SGS/SHARP/ZiLOG NMOS Z80,
 *         (flags & A & 0x28) on NEC NMOS Z80, other models unknown.
 *         However, people from the Speccy scene mention that SCF/CCF X/Y results
 *         are inconsistant and may be influenced by I and R registers.
 *      This Z80 emulator assumes a ZiLOG NMOS model.
 *
 *   Changes in 3.9:
 *    - Fixed cycle counts for LD IYL/IXL/IYH/IXH,n [Marshmellow]
 *    - Fixed X/Y flags in CCF/SCF/BIT, ZEXALL is happy now [hap]
 *    - Simplified DAA, renamed MEMPTR (3.8) to WZ, added TODO [hap]
 *    - Fixed IM2 interrupt cycles [eke]
 *   Changes in 3.8 [Miodrag Milanovic]:
 *    - Added MEMPTR register (according to informations provided
 *      by Vladimir Kladov
 *    - BIT n,(HL) now return valid values due to use of MEMPTR
 *    - Fixed BIT 6,(XY+o) undocumented instructions
 *   Changes in 3.7 [Aaron Giles]:
 *    - Changed NMI handling. NMIs are now latched in set_irq_state
 *      but are not taken there. Instead they are taken at the start of the
 *      execute loop.
 *    - Changed IRQ handling. IRQ state is set in set_irq_state but not taken
 *      except during the inner execute loop.
 *    - Removed x86 assembly hacks and obsolete timing loop catchers.
 *   Changes in 3.6:
 *    - Got rid of the code that would inexactly emulate a Z80, i.e. removed
 *      all the #if Z80_EXACT #else branches.
 *    - Removed leading underscores from local register name shortcuts as
 *      this violates the C99 standard.
 *    - Renamed the registers inside the Z80 context to lower case to avoid
 *      ambiguities (shortcuts would have had the same names as the fields
 *      of the structure).
 *   Changes in 3.5:
 *    - Implemented OTIR, INIR, etc. without look-up table for PF flag.
 *      [Ramsoft, Sean Young]
 *   Changes in 3.4:
 *    - Removed Z80-MSX specific code as it's not needed any more.
 *    - Implemented DAA without look-up table [Ramsoft, Sean Young]
 *   Changes in 3.3:
 *    - Fixed undocumented flags XF & YF in the non-asm versions of CP,
 *      and all the 16 bit arithmetic instructions. [Sean Young]
 *   Changes in 3.2:
 *    - Fixed undocumented flags XF & YF of RRCA, and CF and HF of
 *      INI/IND/OUTI/OUTD/INIR/INDR/OTIR/OTDR [Sean Young]
 *   Changes in 3.1:
 *    - removed the REPEAT_AT_ONCE execution of LDIR/CPIR etc. opcodes
 *      for readabilities sake and because the implementation was buggy
 *      (and i was not able to find the difference)
 *   Changes in 3.0:
 *    - 'finished' switch to dynamically overrideable cycle count tables
 *   Changes in 2.9:
 *    - added methods to access and override the cycle count tables
 *    - fixed handling and timing of multiple DD/FD prefixed opcodes
 *   Changes in 2.8:
 *    - OUTI/OUTD/OTIR/OTDR also pre-decrement the B register now.
 *      This was wrong because of a bug fix on the wrong side
 *      (astrocade sound driver).
 *   Changes in 2.7:
 *    - removed z80_vm specific code, it's not needed (and never was).
 *   Changes in 2.6:
 *    - BUSY_LOOP_HACKS needed to call change_pc() earlier, before
 *      checking the opcodes at the new address, because otherwise they
 *      might access the old (wrong or even nullptr) banked memory region.
 *      Thanks to Sean Young for finding this nasty bug.
 *   Changes in 2.5:
 *    - Burning cycles always adjusts the ICount by a multiple of 4.
 *    - In REPEAT_AT_ONCE cases the r register wasn't incremented twice
 *      per repetition as it should have been. Those repeated opcodes
 *      could also underflow the ICount.
 *    - Simplified TIME_LOOP_HACKS for BC and added two more for DE + HL
 *      timing loops. i think those hacks weren't endian safe before too.
 *   Changes in 2.4:
 *    - z80_reset zaps the entire context, sets IX and IY to 0xffff(!) and
 *      sets the Z flag. With these changes the Tehkan World Cup driver
 *      _seems_ to work again.
 *   Changes in 2.3:
 *    - External termination of the execution loop calls z80_burn() and
 *      z80_vm_burn() to burn an amount of cycles (r adjustment)
 *    - Shortcuts which burn CPU cycles (BUSY_LOOP_HACKS and TIME_LOOP_HACKS)
 *      now also adjust the r register depending on the skipped opcodes.
 *   Changes in 2.2:
 *    - Fixed bugs in CPL, SCF and CCF instructions flag handling.
 *    - Changed variable ea and arg16() function to uint32_t; this
 *      produces slightly more efficient code.
 *    - The DD/FD XY CB opcodes where XY is 40-7F and Y is not 6/E
 *      are changed to calls to the X6/XE opcodes to reduce object size.
 *      They're hardly ever used so this should not yield a speed penalty.
 *   New in 2.0:
 *    - Optional more exact Z80 emulation (#define Z80_EXACT 1) according
 *      to a detailed description by Sean Young which can be found at:
 *      http://www.msxnet.org/tech/z80-documented.pdf
 *****************************************************************************/

#include "z80.h"
#include "../util.h"
#include <string.h>
#include <stdlib.h>
#include <limits.h>

#define VERBOSE             0

/* On an NMOS Z80, if LD A,I or LD A,R is interrupted, P/V flag gets reset,
   even if IFF2 was set before this instruction. This issue was fixed on
   the CMOS Z80, so until knowing (most) Z80 types on hardware, it's disabled */
#define HAS_LDAIR_QUIRK     0

#define LOG(x)  do { if (VERBOSE) warning x; } while (0)


/****************************************************************************/
/* The Z80 registers. halt is set to 1 when the CPU is halted, the refresh  */
/* register is calculated as follows: refresh=(r&127)|(r2&128)    */
/****************************************************************************/

#define CF      0x01
#define NF      0x02
#define PF      0x04
#define VF      PF
#define XF      0x08
#define HF      0x10
#define YF      0x20
#define ZF      0x40
#define SF      0x80

#define INT_IRQ 0x01
#define NMI_IRQ 0x02

#define PRVPC   z80->m_prvpc.d     /* previous program counter */

#define PCD     z80->m_pc.d
#define PC      z80->m_pc.w.l

#define SPD     z80->m_sp.d
#define SP      z80->m_sp.w.l

#define AFD     z80->m_af.d
#define AF      z80->m_af.w.l
#define A       z80->m_af.b.h
#define F       z80->m_af.b.l

#define BCD     z80->m_bc.d
#define BC      z80->m_bc.w.l
#define B       z80->m_bc.b.h
#define C       z80->m_bc.b.l

#define DED     z80->m_de.d
#define DE      z80->m_de.w.l
#define D       z80->m_de.b.h
#define E       z80->m_de.b.l

#define HLD     z80->m_hl.d
#define HL      z80->m_hl.w.l
#define H       z80->m_hl.b.h
#define L       z80->m_hl.b.l

#define IXD     z80->m_ix.d
#define IX      z80->m_ix.w.l
#define HX      z80->m_ix.b.h
#define LX      z80->m_ix.b.l

#define IYD     z80->m_iy.d
#define IY      z80->m_iy.w.l
#define HY      z80->m_iy.b.h
#define LY      z80->m_iy.b.l

#define WZ      z80->m_wz.w.l
#define WZ_H    z80->m_wz.b.h
#define WZ_L    z80->m_wz.b.l


static uint8_t tables_initialised = 0;
static uint8_t SZ[256];       /* zero and sign flags */
static uint8_t SZ_BIT[256];   /* zero, sign and parity/overflow (=zero) flags for BIT opcode */
static uint8_t SZP[256];      /* zero, sign and parity flags */
static uint8_t SZHV_inc[256]; /* zero, sign, half carry and overflow flags INC r8 */
static uint8_t SZHV_dec[256]; /* zero, sign, half carry and overflow flags DEC r8 */

static uint8_t SZHVC_add[2*256*256];
static uint8_t SZHVC_sub[2*256*256];

static const uint8_t cc_op[0x100] = {
	4,10, 7, 6, 4, 4, 7, 4, 4,11, 7, 6, 4, 4, 7, 4,
	8,10, 7, 6, 4, 4, 7, 4,12,11, 7, 6, 4, 4, 7, 4,
	7,10,16, 6, 4, 4, 7, 4, 7,11,16, 6, 4, 4, 7, 4,
	7,10,13, 6,11,11,10, 4, 7,11,13, 6, 4, 4, 7, 4,
	4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
	4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
	4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
	7, 7, 7, 7, 7, 7, 4, 7, 4, 4, 4, 4, 4, 4, 7, 4,
	4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
	4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
	4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
	4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
	5,10,10,10,10,11, 7,11, 5,10,10, 0,10,17, 7,11, /* cb -> cc_cb */
	5,10,10,11,10,11, 7,11, 5, 4,10,11,10, 0, 7,11, /* dd -> cc_xy */
	5,10,10,19,10,11, 7,11, 5, 4,10, 4,10, 0, 7,11, /* ed -> cc_ed */
	5,10,10, 4,10,11, 7,11, 5, 6,10, 4,10, 0, 7,11      /* fd -> cc_xy */
};

static const uint8_t cc_cb[0x100] = {
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,12, 8, 8, 8, 8, 8, 8, 8,12, 8,
	8, 8, 8, 8, 8, 8,12, 8, 8, 8, 8, 8, 8, 8,12, 8,
	8, 8, 8, 8, 8, 8,12, 8, 8, 8, 8, 8, 8, 8,12, 8,
	8, 8, 8, 8, 8, 8,12, 8, 8, 8, 8, 8, 8, 8,12, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8,
	8, 8, 8, 8, 8, 8,15, 8, 8, 8, 8, 8, 8, 8,15, 8
};

static const uint8_t cc_ed[0x100] = {
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
12,12,15,20, 8,14, 8, 9,12,12,15,20, 8,14, 8, 9,
12,12,15,20, 8,14, 8, 9,12,12,15,20, 8,14, 8, 9,
12,12,15,20, 8,14, 8,18,12,12,15,20, 8,14, 8,18,
12,12,15,20, 8,14, 8, 8,12,12,15,20, 8,14, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
16,16,16,16, 8, 8, 8, 8,16,16,16,16, 8, 8, 8, 8,
16,16,16,16, 8, 8, 8, 8,16,16,16,16, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8
};

/* ix/iy: with the exception of (i+offset) opcodes, t-states are main_opcode_table + 4 */
static const uint8_t cc_xy[0x100] = {
	4+4,10+4, 7+4, 6+4, 4+4, 4+4, 7+4, 4+4, 4+4,11+4, 7+4, 6+4, 4+4, 4+4, 7+4, 4+4,
	8+4,10+4, 7+4, 6+4, 4+4, 4+4, 7+4, 4+4,12+4,11+4, 7+4, 6+4, 4+4, 4+4, 7+4, 4+4,
	7+4,10+4,16+4, 6+4, 4+4, 4+4, 7+4, 4+4, 7+4,11+4,16+4, 6+4, 4+4, 4+4, 7+4, 4+4,
	7+4,10+4,13+4, 6+4,23  ,23  ,19  , 4+4, 7+4,11+4,13+4, 6+4, 4+4, 4+4, 7+4, 4+4,
	4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
	4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
	4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
19  ,19  ,19  ,19  ,19  ,19  , 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
	4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
	4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
	4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
	4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4, 4+4, 4+4, 4+4, 4+4, 4+4, 4+4,19  , 4+4,
	5+4,10+4,10+4,10+4,10+4,11+4, 7+4,11+4, 5+4,10+4,10+4, 0  ,10+4,17+4, 7+4,11+4, /* cb -> cc_xycb */
	5+4,10+4,10+4,11+4,10+4,11+4, 7+4,11+4, 5+4, 4+4,10+4,11+4,10+4, 4  , 7+4,11+4, /* dd -> cc_xy again */
	5+4,10+4,10+4,19+4,10+4,11+4, 7+4,11+4, 5+4, 4+4,10+4, 4+4,10+4, 4  , 7+4,11+4, /* ed -> cc_ed */
	5+4,10+4,10+4, 4+4,10+4,11+4, 7+4,11+4, 5+4, 6+4,10+4, 4+4,10+4, 4  , 7+4,11+4      /* fd -> cc_xy again */
};

static const uint8_t cc_xycb[0x100] = {
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23
};

/* extra cycles if jr/jp/call taken and 'interrupt latency' on rst 0-7 */
static const uint8_t cc_ex[0x100] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* DJNZ */
	5, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, /* JR NZ/JR Z */
	5, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, /* JR NC/JR C */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	5, 5, 5, 5, 0, 0, 0, 0, 5, 5, 5, 5, 0, 0, 0, 0, /* LDIR/CPIR/INIR/OTIR LDDR/CPDR/INDR/OTDR */
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2,
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2,
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2,
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2
};

#define m_cc_dd   m_cc_xy
#define m_cc_fd   m_cc_xy

/***************************************************************
 * define an opcode function
 ***************************************************************/
#define OP(prefix,opcode) static inline void prefix##_##opcode(z80_device *z80)

/***************************************************************
 * adjust cycle count by n T-states
 ***************************************************************/
#define CC(prefix,opcode) do { z80->m_icount -= z80->m_cc_##prefix[opcode]; } while (0)

#define EXEC(prefix,opcode) do { \
	unsigned op = opcode; \
	CC(prefix,op); \
	switch(op) \
	{  \
	case 0x00:prefix##_##00(z80);break; case 0x01:prefix##_##01(z80);break; case 0x02:prefix##_##02(z80);break; case 0x03:prefix##_##03(z80);break; \
	case 0x04:prefix##_##04(z80);break; case 0x05:prefix##_##05(z80);break; case 0x06:prefix##_##06(z80);break; case 0x07:prefix##_##07(z80);break; \
	case 0x08:prefix##_##08(z80);break; case 0x09:prefix##_##09(z80);break; case 0x0a:prefix##_##0a(z80);break; case 0x0b:prefix##_##0b(z80);break; \
	case 0x0c:prefix##_##0c(z80);break; case 0x0d:prefix##_##0d(z80);break; case 0x0e:prefix##_##0e(z80);break; case 0x0f:prefix##_##0f(z80);break; \
	case 0x10:prefix##_##10(z80);break; case 0x11:prefix##_##11(z80);break; case 0x12:prefix##_##12(z80);break; case 0x13:prefix##_##13(z80);break; \
	case 0x14:prefix##_##14(z80);break; case 0x15:prefix##_##15(z80);break; case 0x16:prefix##_##16(z80);break; case 0x17:prefix##_##17(z80);break; \
	case 0x18:prefix##_##18(z80);break; case 0x19:prefix##_##19(z80);break; case 0x1a:prefix##_##1a(z80);break; case 0x1b:prefix##_##1b(z80);break; \
	case 0x1c:prefix##_##1c(z80);break; case 0x1d:prefix##_##1d(z80);break; case 0x1e:prefix##_##1e(z80);break; case 0x1f:prefix##_##1f(z80);break; \
	case 0x20:prefix##_##20(z80);break; case 0x21:prefix##_##21(z80);break; case 0x22:prefix##_##22(z80);break; case 0x23:prefix##_##23(z80);break; \
	case 0x24:prefix##_##24(z80);break; case 0x25:prefix##_##25(z80);break; case 0x26:prefix##_##26(z80);break; case 0x27:prefix##_##27(z80);break; \
	case 0x28:prefix##_##28(z80);break; case 0x29:prefix##_##29(z80);break; case 0x2a:prefix##_##2a(z80);break; case 0x2b:prefix##_##2b(z80);break; \
	case 0x2c:prefix##_##2c(z80);break; case 0x2d:prefix##_##2d(z80);break; case 0x2e:prefix##_##2e(z80);break; case 0x2f:prefix##_##2f(z80);break; \
	case 0x30:prefix##_##30(z80);break; case 0x31:prefix##_##31(z80);break; case 0x32:prefix##_##32(z80);break; case 0x33:prefix##_##33(z80);break; \
	case 0x34:prefix##_##34(z80);break; case 0x35:prefix##_##35(z80);break; case 0x36:prefix##_##36(z80);break; case 0x37:prefix##_##37(z80);break; \
	case 0x38:prefix##_##38(z80);break; case 0x39:prefix##_##39(z80);break; case 0x3a:prefix##_##3a(z80);break; case 0x3b:prefix##_##3b(z80);break; \
	case 0x3c:prefix##_##3c(z80);break; case 0x3d:prefix##_##3d(z80);break; case 0x3e:prefix##_##3e(z80);break; case 0x3f:prefix##_##3f(z80);break; \
	case 0x40:prefix##_##40(z80);break; case 0x41:prefix##_##41(z80);break; case 0x42:prefix##_##42(z80);break; case 0x43:prefix##_##43(z80);break; \
	case 0x44:prefix##_##44(z80);break; case 0x45:prefix##_##45(z80);break; case 0x46:prefix##_##46(z80);break; case 0x47:prefix##_##47(z80);break; \
	case 0x48:prefix##_##48(z80);break; case 0x49:prefix##_##49(z80);break; case 0x4a:prefix##_##4a(z80);break; case 0x4b:prefix##_##4b(z80);break; \
	case 0x4c:prefix##_##4c(z80);break; case 0x4d:prefix##_##4d(z80);break; case 0x4e:prefix##_##4e(z80);break; case 0x4f:prefix##_##4f(z80);break; \
	case 0x50:prefix##_##50(z80);break; case 0x51:prefix##_##51(z80);break; case 0x52:prefix##_##52(z80);break; case 0x53:prefix##_##53(z80);break; \
	case 0x54:prefix##_##54(z80);break; case 0x55:prefix##_##55(z80);break; case 0x56:prefix##_##56(z80);break; case 0x57:prefix##_##57(z80);break; \
	case 0x58:prefix##_##58(z80);break; case 0x59:prefix##_##59(z80);break; case 0x5a:prefix##_##5a(z80);break; case 0x5b:prefix##_##5b(z80);break; \
	case 0x5c:prefix##_##5c(z80);break; case 0x5d:prefix##_##5d(z80);break; case 0x5e:prefix##_##5e(z80);break; case 0x5f:prefix##_##5f(z80);break; \
	case 0x60:prefix##_##60(z80);break; case 0x61:prefix##_##61(z80);break; case 0x62:prefix##_##62(z80);break; case 0x63:prefix##_##63(z80);break; \
	case 0x64:prefix##_##64(z80);break; case 0x65:prefix##_##65(z80);break; case 0x66:prefix##_##66(z80);break; case 0x67:prefix##_##67(z80);break; \
	case 0x68:prefix##_##68(z80);break; case 0x69:prefix##_##69(z80);break; case 0x6a:prefix##_##6a(z80);break; case 0x6b:prefix##_##6b(z80);break; \
	case 0x6c:prefix##_##6c(z80);break; case 0x6d:prefix##_##6d(z80);break; case 0x6e:prefix##_##6e(z80);break; case 0x6f:prefix##_##6f(z80);break; \
	case 0x70:prefix##_##70(z80);break; case 0x71:prefix##_##71(z80);break; case 0x72:prefix##_##72(z80);break; case 0x73:prefix##_##73(z80);break; \
	case 0x74:prefix##_##74(z80);break; case 0x75:prefix##_##75(z80);break; case 0x76:prefix##_##76(z80);break; case 0x77:prefix##_##77(z80);break; \
	case 0x78:prefix##_##78(z80);break; case 0x79:prefix##_##79(z80);break; case 0x7a:prefix##_##7a(z80);break; case 0x7b:prefix##_##7b(z80);break; \
	case 0x7c:prefix##_##7c(z80);break; case 0x7d:prefix##_##7d(z80);break; case 0x7e:prefix##_##7e(z80);break; case 0x7f:prefix##_##7f(z80);break; \
	case 0x80:prefix##_##80(z80);break; case 0x81:prefix##_##81(z80);break; case 0x82:prefix##_##82(z80);break; case 0x83:prefix##_##83(z80);break; \
	case 0x84:prefix##_##84(z80);break; case 0x85:prefix##_##85(z80);break; case 0x86:prefix##_##86(z80);break; case 0x87:prefix##_##87(z80);break; \
	case 0x88:prefix##_##88(z80);break; case 0x89:prefix##_##89(z80);break; case 0x8a:prefix##_##8a(z80);break; case 0x8b:prefix##_##8b(z80);break; \
	case 0x8c:prefix##_##8c(z80);break; case 0x8d:prefix##_##8d(z80);break; case 0x8e:prefix##_##8e(z80);break; case 0x8f:prefix##_##8f(z80);break; \
	case 0x90:prefix##_##90(z80);break; case 0x91:prefix##_##91(z80);break; case 0x92:prefix##_##92(z80);break; case 0x93:prefix##_##93(z80);break; \
	case 0x94:prefix##_##94(z80);break; case 0x95:prefix##_##95(z80);break; case 0x96:prefix##_##96(z80);break; case 0x97:prefix##_##97(z80);break; \
	case 0x98:prefix##_##98(z80);break; case 0x99:prefix##_##99(z80);break; case 0x9a:prefix##_##9a(z80);break; case 0x9b:prefix##_##9b(z80);break; \
	case 0x9c:prefix##_##9c(z80);break; case 0x9d:prefix##_##9d(z80);break; case 0x9e:prefix##_##9e(z80);break; case 0x9f:prefix##_##9f(z80);break; \
	case 0xa0:prefix##_##a0(z80);break; case 0xa1:prefix##_##a1(z80);break; case 0xa2:prefix##_##a2(z80);break; case 0xa3:prefix##_##a3(z80);break; \
	case 0xa4:prefix##_##a4(z80);break; case 0xa5:prefix##_##a5(z80);break; case 0xa6:prefix##_##a6(z80);break; case 0xa7:prefix##_##a7(z80);break; \
	case 0xa8:prefix##_##a8(z80);break; case 0xa9:prefix##_##a9(z80);break; case 0xaa:prefix##_##aa(z80);break; case 0xab:prefix##_##ab(z80);break; \
	case 0xac:prefix##_##ac(z80);break; case 0xad:prefix##_##ad(z80);break; case 0xae:prefix##_##ae(z80);break; case 0xaf:prefix##_##af(z80);break; \
	case 0xb0:prefix##_##b0(z80);break; case 0xb1:prefix##_##b1(z80);break; case 0xb2:prefix##_##b2(z80);break; case 0xb3:prefix##_##b3(z80);break; \
	case 0xb4:prefix##_##b4(z80);break; case 0xb5:prefix##_##b5(z80);break; case 0xb6:prefix##_##b6(z80);break; case 0xb7:prefix##_##b7(z80);break; \
	case 0xb8:prefix##_##b8(z80);break; case 0xb9:prefix##_##b9(z80);break; case 0xba:prefix##_##ba(z80);break; case 0xbb:prefix##_##bb(z80);break; \
	case 0xbc:prefix##_##bc(z80);break; case 0xbd:prefix##_##bd(z80);break; case 0xbe:prefix##_##be(z80);break; case 0xbf:prefix##_##bf(z80);break; \
	case 0xc0:prefix##_##c0(z80);break; case 0xc1:prefix##_##c1(z80);break; case 0xc2:prefix##_##c2(z80);break; case 0xc3:prefix##_##c3(z80);break; \
	case 0xc4:prefix##_##c4(z80);break; case 0xc5:prefix##_##c5(z80);break; case 0xc6:prefix##_##c6(z80);break; case 0xc7:prefix##_##c7(z80);break; \
	case 0xc8:prefix##_##c8(z80);break; case 0xc9:prefix##_##c9(z80);break; case 0xca:prefix##_##ca(z80);break; case 0xcb:prefix##_##cb(z80);break; \
	case 0xcc:prefix##_##cc(z80);break; case 0xcd:prefix##_##cd(z80);break; case 0xce:prefix##_##ce(z80);break; case 0xcf:prefix##_##cf(z80);break; \
	case 0xd0:prefix##_##d0(z80);break; case 0xd1:prefix##_##d1(z80);break; case 0xd2:prefix##_##d2(z80);break; case 0xd3:prefix##_##d3(z80);break; \
	case 0xd4:prefix##_##d4(z80);break; case 0xd5:prefix##_##d5(z80);break; case 0xd6:prefix##_##d6(z80);break; case 0xd7:prefix##_##d7(z80);break; \
	case 0xd8:prefix##_##d8(z80);break; case 0xd9:prefix##_##d9(z80);break; case 0xda:prefix##_##da(z80);break; case 0xdb:prefix##_##db(z80);break; \
	case 0xdc:prefix##_##dc(z80);break; case 0xdd:prefix##_##dd(z80);break; case 0xde:prefix##_##de(z80);break; case 0xdf:prefix##_##df(z80);break; \
	case 0xe0:prefix##_##e0(z80);break; case 0xe1:prefix##_##e1(z80);break; case 0xe2:prefix##_##e2(z80);break; case 0xe3:prefix##_##e3(z80);break; \
	case 0xe4:prefix##_##e4(z80);break; case 0xe5:prefix##_##e5(z80);break; case 0xe6:prefix##_##e6(z80);break; case 0xe7:prefix##_##e7(z80);break; \
	case 0xe8:prefix##_##e8(z80);break; case 0xe9:prefix##_##e9(z80);break; case 0xea:prefix##_##ea(z80);break; case 0xeb:prefix##_##eb(z80);break; \
	case 0xec:prefix##_##ec(z80);break; case 0xed:prefix##_##ed(z80);break; case 0xee:prefix##_##ee(z80);break; case 0xef:prefix##_##ef(z80);break; \
	case 0xf0:prefix##_##f0(z80);break; case 0xf1:prefix##_##f1(z80);break; case 0xf2:prefix##_##f2(z80);break; case 0xf3:prefix##_##f3(z80);break; \
	case 0xf4:prefix##_##f4(z80);break; case 0xf5:prefix##_##f5(z80);break; case 0xf6:prefix##_##f6(z80);break; case 0xf7:prefix##_##f7(z80);break; \
	case 0xf8:prefix##_##f8(z80);break; case 0xf9:prefix##_##f9(z80);break; case 0xfa:prefix##_##fa(z80);break; case 0xfb:prefix##_##fb(z80);break; \
	case 0xfc:prefix##_##fc(z80);break; case 0xfd:prefix##_##fd(z80);break; case 0xfe:prefix##_##fe(z80);break; case 0xff:prefix##_##ff(z80);break; \
	} \
} while (0)

/***************************************************************
 * Enter halt state; write 1 to fake port on first execution
 ***************************************************************/
static inline void halt(z80_device *z80)
{
	PC--;
	z80->m_halt = 1;
}

/***************************************************************
 * Leave halt state; write 0 to fake port
 ***************************************************************/
static inline void leave_halt(z80_device *z80)
{
	if( z80->m_halt )
	{
		z80->m_halt = 0;
		PC++;
	}
}

/***************************************************************
 * Input a byte from given I/O port
 ***************************************************************/
static inline uint8_t in(z80_device *z80, uint16_t port)
{
	memmap_chunk const *map_tmp = z80->options->gen.memmap;
	uint32_t chunks_tmp = z80->options->gen.memmap_chunks;
	z80->options->gen.memmap = z80->options->iomap;
	z80->options->gen.memmap_chunks = z80->options->io_chunks;
	uint8_t value = read_byte(port, (void **)z80->mem_pointers, &z80->options->gen, z80);
	z80->options->gen.memmap = map_tmp;
	z80->options->gen.memmap_chunks = chunks_tmp;
	return value;
}

/***************************************************************
 * Output a byte to given I/O port
 ***************************************************************/
static inline void out(z80_device *z80, uint16_t port, uint8_t value)
{
	memmap_chunk const *map_tmp = z80->options->gen.memmap;
	uint32_t chunks_tmp = z80->options->gen.memmap_chunks;
	z80->options->gen.memmap = z80->options->iomap;
	z80->options->gen.memmap_chunks = z80->options->io_chunks;
	write_byte(port & z80->options->io_address_mask, value, (void **)z80->mem_pointers, &z80->options->gen, z80);
	z80->options->gen.memmap = map_tmp;
	z80->options->gen.memmap_chunks = chunks_tmp;
}

/***************************************************************
 * Read a byte from given memory location
 ***************************************************************/
static inline uint8_t rm(z80_device *z80, uint16_t addr)
{
	uint16_t index = addr >> 13;
	if (z80->read_pointers[index]) {
		return z80->read_pointers[index][addr & 0x1FFF];
	}
	return read_byte(addr, (void **)z80->mem_pointers, &z80->options->gen, z80);
}

/***************************************************************
 * Read a word from given memory location
 ***************************************************************/
static inline void rm16(z80_device *z80, uint16_t addr, PAIR *r)
{
	r->b.l = rm(z80, addr);
	r->b.h = rm(z80, (addr+1));
}

/***************************************************************
 * Write a byte to given memory location
 ***************************************************************/
static inline void wm(z80_device *z80, uint16_t addr, uint8_t value)
{
	uint16_t index = addr >> 13;
	if (z80->write_pointers[index]) {
		z80->write_pointers[index][addr & 0x1FFF] = value;
		return;
	}
	write_byte(addr, value, (void **)z80->mem_pointers, &z80->options->gen, z80);
}

/***************************************************************
 * Write a word to given memory location
 ***************************************************************/
static inline void wm16(z80_device *z80, uint16_t addr, PAIR *r)
{
	wm(z80, addr, r->b.l);
	wm(z80, (addr+1), r->b.h);
}

/***************************************************************
 * rop() is identical to rm() except it is used for
 * reading opcodes. In case of system with memory mapped I/O,
 * this function can be used to greatly speed up emulation
 ***************************************************************/
static inline uint8_t rop(z80_device *z80)
{
	unsigned pc = PCD;
	PC++;
	return rm(z80, pc);
}

/****************************************************************
 * arg(z80) is identical to rop() except it is used
 * for reading opcode arguments. This difference can be used to
 * support systems that use different encoding mechanisms for
 * opcodes and opcode arguments
 ***************************************************************/
#define arg(Z) rop(Z)

static inline uint16_t arg16(z80_device *z80)
{
	unsigned pc = PCD;
	PC += 2;
	return rm(z80, pc) | (rm(z80, (pc+1)&0xffff) << 8);
}

/***************************************************************
 * Calculate the effective address EA of an opcode using
 * IX+offset resp. IY+offset addressing.
 ***************************************************************/
static inline void eax(z80_device *z80)
{
	z80->m_ea = (uint32_t)(uint16_t)(IX + (int8_t)arg(z80));
	WZ = z80->m_ea;
}

static inline void eay(z80_device *z80)
{
	z80->m_ea = (uint32_t)(uint16_t)(IY + (int8_t)arg(z80));
	WZ = z80->m_ea;
}

/***************************************************************
 * POP
 ***************************************************************/
static inline void pop(z80_device *z80, PAIR *r)
{
	rm16(z80, SPD, r);
	SP += 2;
}

/***************************************************************
 * PUSH
 ***************************************************************/
static inline void push(z80_device *z80, PAIR *r)
{
	SP -= 2;
	wm16(z80, SPD, r);
}

/***************************************************************
 * JP
 ***************************************************************/
static inline void jp(z80_device *z80)
{
	PCD = arg16(z80);
	WZ = PCD;
}

/***************************************************************
 * JP_COND
 ***************************************************************/
static inline void jp_cond(z80_device *z80, uint8_t cond)
{
	if (cond)
	{
		PCD = arg16(z80);
		WZ = PCD;
	}
	else
	{
		WZ = arg16(z80); /* implicit do PC += 2 */
	}
}

/***************************************************************
 * JR
 ***************************************************************/
static inline void jr(z80_device *z80)
{
	int8_t a = (int8_t)arg(z80);    /* arg(z80) also increments PC */
	PC += a;             /* so don't do PC += arg(z80) */
	WZ = PC;
}

/***************************************************************
 * JR_COND
 ***************************************************************/
static inline void jr_cond(z80_device *z80, uint8_t cond, uint8_t opcode)
{
	if (cond)
	{
		jr(z80);
		CC(ex, opcode);
	}
	else
		PC++;
}

/***************************************************************
 * CALL
 ***************************************************************/
static inline void call(z80_device *z80)
{
	z80->m_ea = arg16(z80);
	WZ = z80->m_ea;
	push(z80, &z80->m_pc);
	PCD = z80->m_ea;
}

/***************************************************************
 * CALL_COND
 ***************************************************************/
static inline void call_cond(z80_device *z80, uint8_t cond, uint8_t opcode)
{
	if (cond)
	{
		z80->m_ea = arg16(z80);
		WZ = z80->m_ea;
		push(z80, &z80->m_pc);
		PCD = z80->m_ea;
		CC(ex, opcode);
	}
	else
	{
		WZ = arg16(z80);  /* implicit call PC+=2;   */
	}
}

/***************************************************************
 * RET_COND
 ***************************************************************/
static inline void ret_cond(z80_device *z80, uint8_t cond, uint8_t opcode)
{
	if (cond)
	{
		pop(z80, &z80->m_pc);
		WZ = PC;
		CC(ex, opcode);
	}
}

/***************************************************************
 * RETN
 ***************************************************************/
static inline void retn(z80_device *z80)
{
	LOG(("Z80 RETN m_iff1:%d m_iff2:%d\n",
		z80->m_iff1, z80->m_iff2));
	pop(z80, &z80->m_pc);
	WZ = PC;
	z80->m_iff1 = z80->m_iff2;
}

/***************************************************************
 * RETI
 ***************************************************************/
static inline void reti(z80_device *z80)
{
	pop(z80, &z80->m_pc);
	WZ = PC;
	z80->m_iff1 = z80->m_iff2;
	//daisy_call_reti_device(z80);
}

/***************************************************************
 * LD   R,A
 ***************************************************************/
static inline void ld_r_a(z80_device *z80)
{
	z80->m_r = A;
	z80->m_r2 = A & 0x80;            /* keep bit 7 of r */
}

/***************************************************************
 * LD   A,R
 ***************************************************************/
static inline void ld_a_r(z80_device *z80)
{
	A = (z80->m_r & 0x7f) | z80->m_r2;
	F = (F & CF) | SZ[A] | (z80->m_iff2 << 2);
	z80->m_after_ldair = 1;
}

/***************************************************************
 * LD   I,A
 ***************************************************************/
static inline void ld_i_a(z80_device *z80)
{
	z80->m_i = A;
}

/***************************************************************
 * LD   A,I
 ***************************************************************/
static inline void ld_a_i(z80_device *z80)
{
	A = z80->m_i;
	F = (F & CF) | SZ[A] | (z80->m_iff2 << 2);
	z80->m_after_ldair = 1;
}

/***************************************************************
 * RST
 ***************************************************************/
static inline void rst(z80_device *z80, uint16_t addr)
{
	push(z80, &z80->m_pc);
	PCD = addr;
	WZ = PC;
}

/***************************************************************
 * INC  r8
 ***************************************************************/
static inline uint8_t inc(z80_device *z80, uint8_t value)
{
	uint8_t res = value + 1;
	F = (F & CF) | SZHV_inc[res];
	return (uint8_t)res;
}

/***************************************************************
 * DEC  r8
 ***************************************************************/
static inline uint8_t dec(z80_device *z80, uint8_t value)
{
	uint8_t res = value - 1;
	F = (F & CF) | SZHV_dec[res];
	return res;
}

/***************************************************************
 * RLCA
 ***************************************************************/
static inline void rlca(z80_device *z80)
{
	A = (A << 1) | (A >> 7);
	F = (F & (SF | ZF | PF)) | (A & (YF | XF | CF));
}

/***************************************************************
 * RRCA
 ***************************************************************/
static inline void rrca(z80_device *z80)
{
	F = (F & (SF | ZF | PF)) | (A & CF);
	A = (A >> 1) | (A << 7);
	F |= (A & (YF | XF));
}

/***************************************************************
 * RLA
 ***************************************************************/
static inline void rla(z80_device *z80)
{
	uint8_t res = (A << 1) | (F & CF);
	uint8_t c = (A & 0x80) ? CF : 0;
	F = (F & (SF | ZF | PF)) | c | (res & (YF | XF));
	A = res;
}

/***************************************************************
 * RRA
 ***************************************************************/
static inline void rra(z80_device *z80)
{
	uint8_t res = (A >> 1) | (F << 7);
	uint8_t c = (A & 0x01) ? CF : 0;
	F = (F & (SF | ZF | PF)) | c | (res & (YF | XF));
	A = res;
}

/***************************************************************
 * RRD
 ***************************************************************/
static inline void rrd(z80_device *z80)
{
	uint8_t n = rm(z80, HL);
	WZ = HL+1;
	wm(z80, HL, (n >> 4) | (A << 4));
	A = (A & 0xf0) | (n & 0x0f);
	F = (F & CF) | SZP[A];
}

/***************************************************************
 * RLD
 ***************************************************************/
static inline void rld(z80_device *z80)
{
	uint8_t n = rm(z80, HL);
	WZ = HL+1;
	wm(z80, HL, (n << 4) | (A & 0x0f));
	A = (A & 0xf0) | (n >> 4);
	F = (F & CF) | SZP[A];
}

/***************************************************************
 * ADD  A,n
 ***************************************************************/
static inline void add_a(z80_device *z80, uint8_t value)
{
	uint32_t ah = AFD & 0xff00;
	uint32_t res = (uint8_t)((ah >> 8) + value);
	F = SZHVC_add[ah | res];
	A = res;
}

/***************************************************************
 * ADC  A,n
 ***************************************************************/
static inline void adc_a(z80_device *z80, uint8_t value)
{
	uint32_t ah = AFD & 0xff00, c = AFD & 1;
	uint32_t res = (uint8_t)((ah >> 8) + value + c);
	F = SZHVC_add[(c << 16) | ah | res];
	A = res;
}

/***************************************************************
 * SUB  n
 ***************************************************************/
static inline void sub(z80_device *z80, uint8_t value)
{
	uint32_t ah = AFD & 0xff00;
	uint32_t res = (uint8_t)((ah >> 8) - value);
	F = SZHVC_sub[ah | res];
	A = res;
}

/***************************************************************
 * SBC  A,n
 ***************************************************************/
static inline void sbc_a(z80_device *z80, uint8_t value)
{
	uint32_t ah = AFD & 0xff00, c = AFD & 1;
	uint32_t res = (uint8_t)((ah >> 8) - value - c);
	F = SZHVC_sub[(c<<16) | ah | res];
	A = res;
}

/***************************************************************
 * NEG
 ***************************************************************/
static inline void neg(z80_device *z80)
{
	uint8_t value = A;
	A = 0;
	sub(z80, value);
}

/***************************************************************
 * DAA
 ***************************************************************/
static inline void daa(z80_device *z80)
{
	uint8_t a = A;
	if (F & NF) {
		if ((F&HF) | ((A&0xf)>9)) a-=6;
		if ((F&CF) | (A>0x99)) a-=0x60;
	}
	else {
		if ((F&HF) | ((A&0xf)>9)) a+=6;
		if ((F&CF) | (A>0x99)) a+=0x60;
	}

	F = (F&(CF|NF)) | (A>0x99) | ((A^a)&HF) | SZP[a];
	A = a;
}

/***************************************************************
 * AND  n
 ***************************************************************/
static inline void and_a(z80_device *z80, uint8_t value)
{
	A &= value;
	F = SZP[A] | HF;
}

/***************************************************************
 * OR   n
 ***************************************************************/
static inline void or_a(z80_device *z80, uint8_t value)
{
	A |= value;
	F = SZP[A];
}

/***************************************************************
 * XOR  n
 ***************************************************************/
static inline void xor_a(z80_device *z80, uint8_t value)
{
	A ^= value;
	F = SZP[A];
}

/***************************************************************
 * CP   n
 ***************************************************************/
static inline void cp(z80_device *z80, uint8_t value)
{
	unsigned val = value;
	uint32_t ah = AFD & 0xff00;
	uint32_t res = (uint8_t)((ah >> 8) - val);
	F = (SZHVC_sub[ah | res] & ~(YF | XF)) |
		(val & (YF | XF));
}

/***************************************************************
 * EX   AF,AF'
 ***************************************************************/
static inline void ex_af(z80_device *z80)
{
	PAIR tmp;
	tmp = z80->m_af; z80->m_af = z80->m_af2; z80->m_af2 = tmp;
}

/***************************************************************
 * EX   DE,HL
 ***************************************************************/
static inline void ex_de_hl(z80_device *z80)
{
	PAIR tmp;
	tmp = z80->m_de; z80->m_de = z80->m_hl; z80->m_hl = tmp;
}

/***************************************************************
 * EXX
 ***************************************************************/
static inline void exx(z80_device *z80)
{
	PAIR tmp;
	tmp = z80->m_bc; z80->m_bc = z80->m_bc2; z80->m_bc2 = tmp;
	tmp = z80->m_de; z80->m_de = z80->m_de2; z80->m_de2 = tmp;
	tmp = z80->m_hl; z80->m_hl = z80->m_hl2; z80->m_hl2 = tmp;
}

/***************************************************************
 * EX   (SP),r16
 ***************************************************************/
static inline void ex_sp(z80_device *z80, PAIR *r)
{
	PAIR tmp = { { 0, 0, 0, 0 } };
	rm16(z80, SPD, &tmp);
	wm16(z80, SPD, r);
	*r = tmp;
	WZ = r->d;
}

/***************************************************************
 * ADD16
 ***************************************************************/
static inline void add16(z80_device *z80, PAIR *dr, PAIR *sr)
{
	uint32_t res = dr->d + sr->d;
	WZ = dr->d + 1;
	F = (F & (SF | ZF | VF)) |
		(((dr->d ^ res ^ sr->d) >> 8) & HF) |
		((res >> 16) & CF) | ((res >> 8) & (YF | XF));
	dr->w.l = (uint16_t)res;
}

/***************************************************************
 * ADC  HL,r16
 ***************************************************************/
static inline void adc_hl(z80_device *z80, PAIR *r)
{
	uint32_t res = HLD + r->d + (F & CF);
	WZ = HL + 1;
	F = (((HLD ^ res ^ r->d) >> 8) & HF) |
		((res >> 16) & CF) |
		((res >> 8) & (SF | YF | XF)) |
		((res & 0xffff) ? 0 : ZF) |
		(((r->d ^ HLD ^ 0x8000) & (r->d ^ res) & 0x8000) >> 13);
	HL = (uint16_t)res;
}

/***************************************************************
 * SBC  HL,r16
 ***************************************************************/
static inline void sbc_hl(z80_device *z80, PAIR *r)
{
	uint32_t res = HLD - r->d - (F & CF);
	WZ = HL + 1;
	F = (((HLD ^ res ^ r->d) >> 8) & HF) | NF |
		((res >> 16) & CF) |
		((res >> 8) & (SF | YF | XF)) |
		((res & 0xffff) ? 0 : ZF) |
		(((r->d ^ HLD) & (HLD ^ res) &0x8000) >> 13);
	HL = (uint16_t)res;
}

/***************************************************************
 * RLC  r8
 ***************************************************************/
static inline uint8_t rlc(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x80) ? CF : 0;
	res = ((res << 1) | (res >> 7)) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * RRC  r8
 ***************************************************************/
static inline uint8_t rrc(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x01) ? CF : 0;
	res = ((res >> 1) | (res << 7)) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * RL   r8
 ***************************************************************/
static inline uint8_t rl(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x80) ? CF : 0;
	res = ((res << 1) | (F & CF)) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * RR   r8
 ***************************************************************/
static inline uint8_t rr(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x01) ? CF : 0;
	res = ((res >> 1) | (F << 7)) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * SLA  r8
 ***************************************************************/
static inline uint8_t sla(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x80) ? CF : 0;
	res = (res << 1) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * SRA  r8
 ***************************************************************/
static inline uint8_t sra(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x01) ? CF : 0;
	res = ((res >> 1) | (res & 0x80)) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * SLL  r8
 ***************************************************************/
static inline uint8_t sll(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x80) ? CF : 0;
	res = ((res << 1) | 0x01) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * SRL  r8
 ***************************************************************/
static inline uint8_t srl(z80_device *z80, uint8_t value)
{
	unsigned res = value;
	unsigned c = (res & 0x01) ? CF : 0;
	res = (res >> 1) & 0xff;
	F = SZP[res] | c;
	return res;
}

/***************************************************************
 * BIT  bit,r8
 ***************************************************************/
static inline void bit(z80_device *z80, int bit, uint8_t value)
{
	F = (F & CF) | HF | (SZ_BIT[value & (1<<bit)] & ~(YF|XF)) | (value & (YF|XF));
}

/***************************************************************
 * BIT  bit,(HL)
 ***************************************************************/
static inline void bit_hl(z80_device *z80, int bit, uint8_t value)
{
	F = (F & CF) | HF | (SZ_BIT[value & (1<<bit)] & ~(YF|XF)) | (WZ_H & (YF|XF));
}

/***************************************************************
 * BIT  bit,(IX/Y+o)
 ***************************************************************/
static inline void bit_xy(z80_device *z80, int bit, uint8_t value)
{
	F = (F & CF) | HF | (SZ_BIT[value & (1<<bit)] & ~(YF|XF)) | ((z80->m_ea>>8) & (YF|XF));
}

/***************************************************************
 * RES  bit,r8
 ***************************************************************/
static inline uint8_t res(int bit, uint8_t value)
{
	return value & ~(1<<bit);
}

/***************************************************************
 * SET  bit,r8
 ***************************************************************/
static inline uint8_t set(int bit, uint8_t value)
{
	return value | (1<<bit);
}

/***************************************************************
 * LDI
 ***************************************************************/
static inline void ldi(z80_device *z80)
{
	uint8_t io = rm(z80, HL);
	wm(z80, DE, io);
	F &= SF | ZF | CF;
	if ((A + io) & 0x02) F |= YF; /* bit 1 -> flag 5 */
	if ((A + io) & 0x08) F |= XF; /* bit 3 -> flag 3 */
	HL++; DE++; BC--;
	if(BC) F |= VF;
}

/***************************************************************
 * CPI
 ***************************************************************/
static inline void cpi(z80_device *z80)
{
	uint8_t val = rm(z80, HL);
	uint8_t res = A - val;
	WZ++;
	HL++; BC--;
	F = (F & CF) | (SZ[res]&~(YF|XF)) | ((A^val^res)&HF) | NF;
	if (F & HF) res -= 1;
	if (res & 0x02) F |= YF; /* bit 1 -> flag 5 */
	if (res & 0x08) F |= XF; /* bit 3 -> flag 3 */
	if (BC) F |= VF;
}

/***************************************************************
 * INI
 ***************************************************************/
static inline void ini(z80_device *z80)
{
	unsigned t;
	uint8_t io = in(z80, BC);
	WZ = BC + 1;
	B--;
	wm(z80, HL, io);
	HL++;
	F = SZ[B];
	t = (unsigned)((C + 1) & 0xff) + (unsigned)io;
	if (io & SF) F |= NF;
	if (t & 0x100) F |= HF | CF;
	F |= SZP[(uint8_t)(t & 0x07) ^ B] & PF;
}

/***************************************************************
 * OUTI
 ***************************************************************/
static inline void outi(z80_device *z80)
{
	unsigned t;
	uint8_t io = rm(z80, HL);
	B--;
	WZ = BC + 1;
	out(z80, BC, io);
	HL++;
	F = SZ[B];
	t = (unsigned)L + (unsigned)io;
	if (io & SF) F |= NF;
	if (t & 0x100) F |= HF | CF;
	F |= SZP[(uint8_t)(t & 0x07) ^ B] & PF;
}

/***************************************************************
 * LDD
 ***************************************************************/
static inline void ldd(z80_device *z80)
{
	uint8_t io = rm(z80, HL);
	wm(z80, DE, io);
	F &= SF | ZF | CF;
	if ((A + io) & 0x02) F |= YF; /* bit 1 -> flag 5 */
	if ((A + io) & 0x08) F |= XF; /* bit 3 -> flag 3 */
	HL--; DE--; BC--;
	if (BC) F |= VF;
}

/***************************************************************
 * CPD
 ***************************************************************/
static inline void cpd(z80_device *z80)
{
	uint8_t val = rm(z80, HL);
	uint8_t res = A - val;
	WZ--;
	HL--; BC--;
	F = (F & CF) | (SZ[res]&~(YF|XF)) | ((A^val^res)&HF) | NF;
	if (F & HF) res -= 1;
	if (res & 0x02) F |= YF; /* bit 1 -> flag 5 */
	if (res & 0x08) F |= XF; /* bit 3 -> flag 3 */
	if (BC) F |= VF;
}

/***************************************************************
 * IND
 ***************************************************************/
static inline void ind(z80_device *z80)
{
	unsigned t;
	uint8_t io = in(z80, BC);
	WZ = BC - 1;
	B--;
	wm(z80, HL, io);
	HL--;
	F = SZ[B];
	t = ((unsigned)(C - 1) & 0xff) + (unsigned)io;
	if (io & SF) F |= NF;
	if (t & 0x100) F |= HF | CF;
	F |= SZP[(uint8_t)(t & 0x07) ^ B] & PF;
}

/***************************************************************
 * OUTD
 ***************************************************************/
static inline void outd(z80_device *z80)
{
	unsigned t;
	uint8_t io = rm(z80, HL);
	B--;
	WZ = BC - 1;
	out(z80, BC, io);
	HL--;
	F = SZ[B];
	t = (unsigned)L + (unsigned)io;
	if (io & SF) F |= NF;
	if (t & 0x100) F |= HF | CF;
	F |= SZP[(uint8_t)(t & 0x07) ^ B] & PF;
}

/***************************************************************
 * LDIR
 ***************************************************************/
static inline void ldir(z80_device *z80)
{
	ldi(z80);
	if (BC != 0)
	{
		PC -= 2;
		WZ = PC + 1;
		CC(ex, 0xb0);
	}
}

/***************************************************************
 * CPIR
 ***************************************************************/
static inline void cpir(z80_device *z80)
{
	cpi(z80);
	if (BC != 0 && !(F & ZF))
	{
		PC -= 2;
		WZ = PC + 1;
		CC(ex, 0xb1);
	}
}

/***************************************************************
 * INIR
 ***************************************************************/
static inline void inir(z80_device *z80)
{
	ini(z80);
	if (B != 0)
	{
		PC -= 2;
		CC(ex, 0xb2);
	}
}

/***************************************************************
 * OTIR
 ***************************************************************/
static inline void otir(z80_device *z80)
{
	outi(z80);
	if (B != 0)
	{
		PC -= 2;
		CC(ex, 0xb3);
	}
}

/***************************************************************
 * LDDR
 ***************************************************************/
static inline void lddr(z80_device *z80)
{
	ldd(z80);
	if (BC != 0)
	{
		PC -= 2;
		WZ = PC + 1;
		CC(ex, 0xb8);
	}
}

/***************************************************************
 * CPDR
 ***************************************************************/
static inline void cpdr(z80_device *z80)
{
	cpd(z80);
	if (BC != 0 && !(F & ZF))
	{
		PC -= 2;
		WZ = PC + 1;
		CC(ex, 0xb9);
	}
}

/***************************************************************
 * INDR
 ***************************************************************/
static inline void indr(z80_device *z80)
{
	ind(z80);
	if (B != 0)
	{
		PC -= 2;
		CC(ex, 0xba);
	}
}

/***************************************************************
 * OTDR
 ***************************************************************/
static inline void otdr(z80_device *z80)
{
	outd(z80);
	if (B != 0)
	{
		PC -= 2;
		CC(ex, 0xbb);
	}
}

/***************************************************************
 * EI
 ***************************************************************/
static inline void ei(z80_device *z80)
{
	z80->m_iff1 = z80->m_iff2 = 1;
	z80->m_after_ei = 1;
}

#define PROTOTYPES(prefix) \
	static inline void prefix##_00(z80_device *z80); static inline void prefix##_01(z80_device *z80); static inline void prefix##_02(z80_device *z80); static inline void prefix##_03(z80_device *z80); \
	static inline void prefix##_04(z80_device *z80); static inline void prefix##_05(z80_device *z80); static inline void prefix##_06(z80_device *z80); static inline void prefix##_07(z80_device *z80); \
	static inline void prefix##_08(z80_device *z80); static inline void prefix##_09(z80_device *z80); static inline void prefix##_0a(z80_device *z80); static inline void prefix##_0b(z80_device *z80); \
	static inline void prefix##_0c(z80_device *z80); static inline void prefix##_0d(z80_device *z80); static inline void prefix##_0e(z80_device *z80); static inline void prefix##_0f(z80_device *z80); \
	static inline void prefix##_10(z80_device *z80); static inline void prefix##_11(z80_device *z80); static inline void prefix##_12(z80_device *z80); static inline void prefix##_13(z80_device *z80); \
	static inline void prefix##_14(z80_device *z80); static inline void prefix##_15(z80_device *z80); static inline void prefix##_16(z80_device *z80); static inline void prefix##_17(z80_device *z80); \
	static inline void prefix##_18(z80_device *z80); static inline void prefix##_19(z80_device *z80); static inline void prefix##_1a(z80_device *z80); static inline void prefix##_1b(z80_device *z80); \
	static inline void prefix##_1c(z80_device *z80); static inline void prefix##_1d(z80_device *z80); static inline void prefix##_1e(z80_device *z80); static inline void prefix##_1f(z80_device *z80); \
	static inline void prefix##_20(z80_device *z80); static inline void prefix##_21(z80_device *z80); static inline void prefix##_22(z80_device *z80); static inline void prefix##_23(z80_device *z80); \
	static inline void prefix##_24(z80_device *z80); static inline void prefix##_25(z80_device *z80); static inline void prefix##_26(z80_device *z80); static inline void prefix##_27(z80_device *z80); \
	static inline void prefix##_28(z80_device *z80); static inline void prefix##_29(z80_device *z80); static inline void prefix##_2a(z80_device *z80); static inline void prefix##_2b(z80_device *z80); \
	static inline void prefix##_2c(z80_device *z80); static inline void prefix##_2d(z80_device *z80); static inline void prefix##_2e(z80_device *z80); static inline void prefix##_2f(z80_device *z80); \
	static inline void prefix##_30(z80_device *z80); static inline void prefix##_31(z80_device *z80); static inline void prefix##_32(z80_device *z80); static inline void prefix##_33(z80_device *z80); \
	static inline void prefix##_34(z80_device *z80); static inline void prefix##_35(z80_device *z80); static inline void prefix##_36(z80_device *z80); static inline void prefix##_37(z80_device *z80); \
	static inline void prefix##_38(z80_device *z80); static inline void prefix##_39(z80_device *z80); static inline void prefix##_3a(z80_device *z80); static inline void prefix##_3b(z80_device *z80); \
	static inline void prefix##_3c(z80_device *z80); static inline void prefix##_3d(z80_device *z80); static inline void prefix##_3e(z80_device *z80); static inline void prefix##_3f(z80_device *z80); \
	static inline void prefix##_40(z80_device *z80); static inline void prefix##_41(z80_device *z80); static inline void prefix##_42(z80_device *z80); static inline void prefix##_43(z80_device *z80); \
	static inline void prefix##_44(z80_device *z80); static inline void prefix##_45(z80_device *z80); static inline void prefix##_46(z80_device *z80); static inline void prefix##_47(z80_device *z80); \
	static inline void prefix##_48(z80_device *z80); static inline void prefix##_49(z80_device *z80); static inline void prefix##_4a(z80_device *z80); static inline void prefix##_4b(z80_device *z80); \
	static inline void prefix##_4c(z80_device *z80); static inline void prefix##_4d(z80_device *z80); static inline void prefix##_4e(z80_device *z80); static inline void prefix##_4f(z80_device *z80); \
	static inline void prefix##_50(z80_device *z80); static inline void prefix##_51(z80_device *z80); static inline void prefix##_52(z80_device *z80); static inline void prefix##_53(z80_device *z80); \
	static inline void prefix##_54(z80_device *z80); static inline void prefix##_55(z80_device *z80); static inline void prefix##_56(z80_device *z80); static inline void prefix##_57(z80_device *z80); \
	static inline void prefix##_58(z80_device *z80); static inline void prefix##_59(z80_device *z80); static inline void prefix##_5a(z80_device *z80); static inline void prefix##_5b(z80_device *z80); \
	static inline void prefix##_5c(z80_device *z80); static inline void prefix##_5d(z80_device *z80); static inline void prefix##_5e(z80_device *z80); static inline void prefix##_5f(z80_device *z80); \
	static inline void prefix##_60(z80_device *z80); static inline void prefix##_61(z80_device *z80); static inline void prefix##_62(z80_device *z80); static inline void prefix##_63(z80_device *z80); \
	static inline void prefix##_64(z80_device *z80); static inline void prefix##_65(z80_device *z80); static inline void prefix##_66(z80_device *z80); static inline void prefix##_67(z80_device *z80); \
	static inline void prefix##_68(z80_device *z80); static inline void prefix##_69(z80_device *z80); static inline void prefix##_6a(z80_device *z80); static inline void prefix##_6b(z80_device *z80); \
	static inline void prefix##_6c(z80_device *z80); static inline void prefix##_6d(z80_device *z80); static inline void prefix##_6e(z80_device *z80); static inline void prefix##_6f(z80_device *z80); \
	static inline void prefix##_70(z80_device *z80); static inline void prefix##_71(z80_device *z80); static inline void prefix##_72(z80_device *z80); static inline void prefix##_73(z80_device *z80); \
	static inline void prefix##_74(z80_device *z80); static inline void prefix##_75(z80_device *z80); static inline void prefix##_76(z80_device *z80); static inline void prefix##_77(z80_device *z80); \
	static inline void prefix##_78(z80_device *z80); static inline void prefix##_79(z80_device *z80); static inline void prefix##_7a(z80_device *z80); static inline void prefix##_7b(z80_device *z80); \
	static inline void prefix##_7c(z80_device *z80); static inline void prefix##_7d(z80_device *z80); static inline void prefix##_7e(z80_device *z80); static inline void prefix##_7f(z80_device *z80); \
	static inline void prefix##_80(z80_device *z80); static inline void prefix##_81(z80_device *z80); static inline void prefix##_82(z80_device *z80); static inline void prefix##_83(z80_device *z80); \
	static inline void prefix##_84(z80_device *z80); static inline void prefix##_85(z80_device *z80); static inline void prefix##_86(z80_device *z80); static inline void prefix##_87(z80_device *z80); \
	static inline void prefix##_88(z80_device *z80); static inline void prefix##_89(z80_device *z80); static inline void prefix##_8a(z80_device *z80); static inline void prefix##_8b(z80_device *z80); \
	static inline void prefix##_8c(z80_device *z80); static inline void prefix##_8d(z80_device *z80); static inline void prefix##_8e(z80_device *z80); static inline void prefix##_8f(z80_device *z80); \
	static inline void prefix##_90(z80_device *z80); static inline void prefix##_91(z80_device *z80); static inline void prefix##_92(z80_device *z80); static inline void prefix##_93(z80_device *z80); \
	static inline void prefix##_94(z80_device *z80); static inline void prefix##_95(z80_device *z80); static inline void prefix##_96(z80_device *z80); static inline void prefix##_97(z80_device *z80); \
	static inline void prefix##_98(z80_device *z80); static inline void prefix##_99(z80_device *z80); static inline void prefix##_9a(z80_device *z80); static inline void prefix##_9b(z80_device *z80); \
	static inline void prefix##_9c(z80_device *z80); static inline void prefix##_9d(z80_device *z80); static inline void prefix##_9e(z80_device *z80); static inline void prefix##_9f(z80_device *z80); \
	static inline void prefix##_a0(z80_device *z80); static inline void prefix##_a1(z80_device *z80); static inline void prefix##_a2(z80_device *z80); static inline void prefix##_a3(z80_device *z80); \
	static inline void prefix##_a4(z80_device *z80); static inline void prefix##_a5(z80_device *z80); static inline void prefix##_a6(z80_device *z80); static inline void prefix##_a7(z80_device *z80); \
	static inline void prefix##_a8(z80_device *z80); static inline void prefix##_a9(z80_device *z80); static inline void prefix##_aa(z80_device *z80); static inline void prefix##_ab(z80_device *z80); \
	static inline void prefix##_ac(z80_device *z80); static inline void prefix##_ad(z80_device *z80); static inline void prefix##_ae(z80_device *z80); static inline void prefix##_af(z80_device *z80); \
	static inline void prefix##_b0(z80_device *z80); static inline void prefix##_b1(z80_device *z80); static inline void prefix##_b2(z80_device *z80); static inline void prefix##_b3(z80_device *z80); \
	static inline void prefix##_b4(z80_device *z80); static inline void prefix##_b5(z80_device *z80); static inline void prefix##_b6(z80_device *z80); static inline void prefix##_b7(z80_device *z80); \
	static inline void prefix##_b8(z80_device *z80); static inline void prefix##_b9(z80_device *z80); static inline void prefix##_ba(z80_device *z80); static inline void prefix##_bb(z80_device *z80); \
	static inline void prefix##_bc(z80_device *z80); static inline void prefix##_bd(z80_device *z80); static inline void prefix##_be(z80_device *z80); static inline void prefix##_bf(z80_device *z80); \
	static inline void prefix##_c0(z80_device *z80); static inline void prefix##_c1(z80_device *z80); static inline void prefix##_c2(z80_device *z80); static inline void prefix##_c3(z80_device *z80); \
	static inline void prefix##_c4(z80_device *z80); static inline void prefix##_c5(z80_device *z80); static inline void prefix##_c6(z80_device *z80); static inline void prefix##_c7(z80_device *z80); \
	static inline void prefix##_c8(z80_device *z80); static inline void prefix##_c9(z80_device *z80); static inline void prefix##_ca(z80_device *z80); static inline void prefix##_cb(z80_device *z80); \
	static inline void prefix##_cc(z80_device *z80); static inline void prefix##_cd(z80_device *z80); static inline void prefix##_ce(z80_device *z80); static inline void prefix##_cf(z80_device *z80); \
	static inline void prefix##_d0(z80_device *z80); static inline void prefix##_d1(z80_device *z80); static inline void prefix##_d2(z80_device *z80); static inline void prefix##_d3(z80_device *z80); \
	static inline void prefix##_d4(z80_device *z80); static inline void prefix##_d5(z80_device *z80); static inline void prefix##_d6(z80_device *z80); static inline void prefix##_d7(z80_device *z80); \
	static inline void prefix##_d8(z80_device *z80); static inline void prefix##_d9(z80_device *z80); static inline void prefix##_da(z80_device *z80); static inline void prefix##_db(z80_device *z80); \
	static inline void prefix##_dc(z80_device *z80); static inline void prefix##_dd(z80_device *z80); static inline void prefix##_de(z80_device *z80); static inline void prefix##_df(z80_device *z80); \
	static inline void prefix##_e0(z80_device *z80); static inline void prefix##_e1(z80_device *z80); static inline void prefix##_e2(z80_device *z80); static inline void prefix##_e3(z80_device *z80); \
	static inline void prefix##_e4(z80_device *z80); static inline void prefix##_e5(z80_device *z80); static inline void prefix##_e6(z80_device *z80); static inline void prefix##_e7(z80_device *z80); \
	static inline void prefix##_e8(z80_device *z80); static inline void prefix##_e9(z80_device *z80); static inline void prefix##_ea(z80_device *z80); static inline void prefix##_eb(z80_device *z80); \
	static inline void prefix##_ec(z80_device *z80); static inline void prefix##_ed(z80_device *z80); static inline void prefix##_ee(z80_device *z80); static inline void prefix##_ef(z80_device *z80); \
	static inline void prefix##_f0(z80_device *z80); static inline void prefix##_f1(z80_device *z80); static inline void prefix##_f2(z80_device *z80); static inline void prefix##_f3(z80_device *z80); \
	static inline void prefix##_f4(z80_device *z80); static inline void prefix##_f5(z80_device *z80); static inline void prefix##_f6(z80_device *z80); static inline void prefix##_f7(z80_device *z80); \
	static inline void prefix##_f8(z80_device *z80); static inline void prefix##_f9(z80_device *z80); static inline void prefix##_fa(z80_device *z80); static inline void prefix##_fb(z80_device *z80); \
	static inline void prefix##_fc(z80_device *z80); static inline void prefix##_fd(z80_device *z80); static inline void prefix##_fe(z80_device *z80); static inline void prefix##_ff(z80_device *z80);

PROTOTYPES(op)
PROTOTYPES(cb)
PROTOTYPES(dd)
PROTOTYPES(ed)
PROTOTYPES(fd)
PROTOTYPES(xycb)

/**********************************************************
 * opcodes with CB prefix
 * rotate, shift and bit operations
 **********************************************************/
OP(cb,00) { B = rlc(z80, B);             } /* RLC  B           */
OP(cb,01) { C = rlc(z80, C);             } /* RLC  C           */
OP(cb,02) { D = rlc(z80, D);             } /* RLC  D           */
OP(cb,03) { E = rlc(z80, E);             } /* RLC  E           */
OP(cb,04) { H = rlc(z80, H);             } /* RLC  H           */
OP(cb,05) { L = rlc(z80, L);             } /* RLC  L           */
OP(cb,06) { wm(z80, HL, rlc(z80, rm(z80, HL)));    } /* RLC  (HL)        */
OP(cb,07) { A = rlc(z80, A);             } /* RLC  A           */

OP(cb,08) { B = rrc(z80, B);             } /* RRC  B           */
OP(cb,09) { C = rrc(z80, C);             } /* RRC  C           */
OP(cb,0a) { D = rrc(z80, D);             } /* RRC  D           */
OP(cb,0b) { E = rrc(z80, E);             } /* RRC  E           */
OP(cb,0c) { H = rrc(z80, H);             } /* RRC  H           */
OP(cb,0d) { L = rrc(z80, L);             } /* RRC  L           */
OP(cb,0e) { wm(z80, HL, rrc(z80, rm(z80, HL)));    } /* RRC  (HL)        */
OP(cb,0f) { A = rrc(z80, A);             } /* RRC  A           */

OP(cb,10) { B = rl(z80, B);              } /* RL   B           */
OP(cb,11) { C = rl(z80, C);              } /* RL   C           */
OP(cb,12) { D = rl(z80, D);              } /* RL   D           */
OP(cb,13) { E = rl(z80, E);              } /* RL   E           */
OP(cb,14) { H = rl(z80, H);              } /* RL   H           */
OP(cb,15) { L = rl(z80, L);              } /* RL   L           */
OP(cb,16) { wm(z80, HL, rl(z80, rm(z80, HL)));     } /* RL   (HL)        */
OP(cb,17) { A = rl(z80, A);              } /* RL   A           */

OP(cb,18) { B = rr(z80, B);              } /* RR   B           */
OP(cb,19) { C = rr(z80, C);              } /* RR   C           */
OP(cb,1a) { D = rr(z80, D);              } /* RR   D           */
OP(cb,1b) { E = rr(z80, E);              } /* RR   E           */
OP(cb,1c) { H = rr(z80, H);              } /* RR   H           */
OP(cb,1d) { L = rr(z80, L);              } /* RR   L           */
OP(cb,1e) { wm(z80, HL, rr(z80, rm(z80, HL)));     } /* RR   (HL)        */
OP(cb,1f) { A = rr(z80, A);              } /* RR   A           */

OP(cb,20) { B = sla(z80, B);             } /* SLA  B           */
OP(cb,21) { C = sla(z80, C);             } /* SLA  C           */
OP(cb,22) { D = sla(z80, D);             } /* SLA  D           */
OP(cb,23) { E = sla(z80, E);             } /* SLA  E           */
OP(cb,24) { H = sla(z80, H);             } /* SLA  H           */
OP(cb,25) { L = sla(z80, L);             } /* SLA  L           */
OP(cb,26) { wm(z80, HL, sla(z80, rm(z80, HL)));    } /* SLA  (HL)        */
OP(cb,27) { A = sla(z80, A);             } /* SLA  A           */

OP(cb,28) { B = sra(z80, B);             } /* SRA  B           */
OP(cb,29) { C = sra(z80, C);             } /* SRA  C           */
OP(cb,2a) { D = sra(z80, D);             } /* SRA  D           */
OP(cb,2b) { E = sra(z80, E);             } /* SRA  E           */
OP(cb,2c) { H = sra(z80, H);             } /* SRA  H           */
OP(cb,2d) { L = sra(z80, L);             } /* SRA  L           */
OP(cb,2e) { wm(z80, HL, sra(z80, rm(z80, HL)));    } /* SRA  (HL)        */
OP(cb,2f) { A = sra(z80, A);             } /* SRA  A           */

OP(cb,30) { B = sll(z80, B);             } /* SLL  B           */
OP(cb,31) { C = sll(z80, C);             } /* SLL  C           */
OP(cb,32) { D = sll(z80, D);             } /* SLL  D           */
OP(cb,33) { E = sll(z80, E);             } /* SLL  E           */
OP(cb,34) { H = sll(z80, H);             } /* SLL  H           */
OP(cb,35) { L = sll(z80, L);             } /* SLL  L           */
OP(cb,36) { wm(z80, HL, sll(z80, rm(z80, HL)));    } /* SLL  (HL)        */
OP(cb,37) { A = sll(z80, A);             } /* SLL  A           */

OP(cb,38) { B = srl(z80, B);             } /* SRL  B           */
OP(cb,39) { C = srl(z80, C);             } /* SRL  C           */
OP(cb,3a) { D = srl(z80, D);             } /* SRL  D           */
OP(cb,3b) { E = srl(z80, E);             } /* SRL  E           */
OP(cb,3c) { H = srl(z80, H);             } /* SRL  H           */
OP(cb,3d) { L = srl(z80, L);             } /* SRL  L           */
OP(cb,3e) { wm(z80, HL, srl(z80, rm(z80, HL)));    } /* SRL  (HL)        */
OP(cb,3f) { A = srl(z80, A);             } /* SRL  A           */

OP(cb,40) { bit(z80, 0, B);              } /* BIT  0,B         */
OP(cb,41) { bit(z80, 0, C);              } /* BIT  0,C         */
OP(cb,42) { bit(z80, 0, D);              } /* BIT  0,D         */
OP(cb,43) { bit(z80, 0, E);              } /* BIT  0,E         */
OP(cb,44) { bit(z80, 0, H);              } /* BIT  0,H         */
OP(cb,45) { bit(z80, 0, L);              } /* BIT  0,L         */
OP(cb,46) { bit_hl(z80, 0, rm(z80, HL));      } /* BIT  0,(HL)      */
OP(cb,47) { bit(z80, 0, A);              } /* BIT  0,A         */

OP(cb,48) { bit(z80, 1, B);              } /* BIT  1,B         */
OP(cb,49) { bit(z80, 1, C);              } /* BIT  1,C         */
OP(cb,4a) { bit(z80, 1, D);              } /* BIT  1,D         */
OP(cb,4b) { bit(z80, 1, E);              } /* BIT  1,E         */
OP(cb,4c) { bit(z80, 1, H);              } /* BIT  1,H         */
OP(cb,4d) { bit(z80, 1, L);              } /* BIT  1,L         */
OP(cb,4e) { bit_hl(z80, 1, rm(z80, HL));      } /* BIT  1,(HL)      */
OP(cb,4f) { bit(z80, 1, A);              } /* BIT  1,A         */

OP(cb,50) { bit(z80, 2, B);              } /* BIT  2,B         */
OP(cb,51) { bit(z80, 2, C);              } /* BIT  2,C         */
OP(cb,52) { bit(z80, 2, D);              } /* BIT  2,D         */
OP(cb,53) { bit(z80, 2, E);              } /* BIT  2,E         */
OP(cb,54) { bit(z80, 2, H);              } /* BIT  2,H         */
OP(cb,55) { bit(z80, 2, L);              } /* BIT  2,L         */
OP(cb,56) { bit_hl(z80, 2, rm(z80, HL));      } /* BIT  2,(HL)      */
OP(cb,57) { bit(z80, 2, A);              } /* BIT  2,A         */

OP(cb,58) { bit(z80, 3, B);              } /* BIT  3,B         */
OP(cb,59) { bit(z80, 3, C);              } /* BIT  3,C         */
OP(cb,5a) { bit(z80, 3, D);              } /* BIT  3,D         */
OP(cb,5b) { bit(z80, 3, E);              } /* BIT  3,E         */
OP(cb,5c) { bit(z80, 3, H);              } /* BIT  3,H         */
OP(cb,5d) { bit(z80, 3, L);              } /* BIT  3,L         */
OP(cb,5e) { bit_hl(z80, 3, rm(z80, HL));      } /* BIT  3,(HL)      */
OP(cb,5f) { bit(z80, 3, A);              } /* BIT  3,A         */

OP(cb,60) { bit(z80, 4, B);              } /* BIT  4,B         */
OP(cb,61) { bit(z80, 4, C);              } /* BIT  4,C         */
OP(cb,62) { bit(z80, 4, D);              } /* BIT  4,D         */
OP(cb,63) { bit(z80, 4, E);              } /* BIT  4,E         */
OP(cb,64) { bit(z80, 4, H);              } /* BIT  4,H         */
OP(cb,65) { bit(z80, 4, L);              } /* BIT  4,L         */
OP(cb,66) { bit_hl(z80, 4, rm(z80, HL));      } /* BIT  4,(HL)      */
OP(cb,67) { bit(z80, 4, A);              } /* BIT  4,A         */

OP(cb,68) { bit(z80, 5, B);              } /* BIT  5,B         */
OP(cb,69) { bit(z80, 5, C);              } /* BIT  5,C         */
OP(cb,6a) { bit(z80, 5, D);              } /* BIT  5,D         */
OP(cb,6b) { bit(z80, 5, E);              } /* BIT  5,E         */
OP(cb,6c) { bit(z80, 5, H);              } /* BIT  5,H         */
OP(cb,6d) { bit(z80, 5, L);              } /* BIT  5,L         */
OP(cb,6e) { bit_hl(z80, 5, rm(z80, HL));      } /* BIT  5,(HL)      */
OP(cb,6f) { bit(z80, 5, A);              } /* BIT  5,A         */

OP(cb,70) { bit(z80, 6, B);              } /* BIT  6,B         */
OP(cb,71) { bit(z80, 6, C);              } /* BIT  6,C         */
OP(cb,72) { bit(z80, 6, D);              } /* BIT  6,D         */
OP(cb,73) { bit(z80, 6, E);              } /* BIT  6,E         */
OP(cb,74) { bit(z80, 6, H);              } /* BIT  6,H         */
OP(cb,75) { bit(z80, 6, L);              } /* BIT  6,L         */
OP(cb,76) { bit_hl(z80, 6, rm(z80, HL));      } /* BIT  6,(HL)      */
OP(cb,77) { bit(z80, 6, A);              } /* BIT  6,A         */

OP(cb,78) { bit(z80, 7, B);              } /* BIT  7,B         */
OP(cb,79) { bit(z80, 7, C);              } /* BIT  7,C         */
OP(cb,7a) { bit(z80, 7, D);              } /* BIT  7,D         */
OP(cb,7b) { bit(z80, 7, E);              } /* BIT  7,E         */
OP(cb,7c) { bit(z80, 7, H);              } /* BIT  7,H         */
OP(cb,7d) { bit(z80, 7, L);              } /* BIT  7,L         */
OP(cb,7e) { bit_hl(z80, 7, rm(z80, HL));      } /* BIT  7,(HL)      */
OP(cb,7f) { bit(z80, 7, A);              } /* BIT  7,A         */

OP(cb,80) { B = res(0, B);          } /* RES  0,B         */
OP(cb,81) { C = res(0, C);          } /* RES  0,C         */
OP(cb,82) { D = res(0, D);          } /* RES  0,D         */
OP(cb,83) { E = res(0, E);          } /* RES  0,E         */
OP(cb,84) { H = res(0, H);          } /* RES  0,H         */
OP(cb,85) { L = res(0, L);          } /* RES  0,L         */
OP(cb,86) { wm(z80, HL, res(0, rm(z80, HL))); } /* RES  0,(HL)      */
OP(cb,87) { A = res(0, A);          } /* RES  0,A         */

OP(cb,88) { B = res(1, B);          } /* RES  1,B         */
OP(cb,89) { C = res(1, C);          } /* RES  1,C         */
OP(cb,8a) { D = res(1, D);          } /* RES  1,D         */
OP(cb,8b) { E = res(1, E);          } /* RES  1,E         */
OP(cb,8c) { H = res(1, H);          } /* RES  1,H         */
OP(cb,8d) { L = res(1, L);          } /* RES  1,L         */
OP(cb,8e) { wm(z80, HL, res(1, rm(z80, HL))); } /* RES  1,(HL)      */
OP(cb,8f) { A = res(1, A);          } /* RES  1,A         */

OP(cb,90) { B = res(2, B);          } /* RES  2,B         */
OP(cb,91) { C = res(2, C);          } /* RES  2,C         */
OP(cb,92) { D = res(2, D);          } /* RES  2,D         */
OP(cb,93) { E = res(2, E);          } /* RES  2,E         */
OP(cb,94) { H = res(2, H);          } /* RES  2,H         */
OP(cb,95) { L = res(2, L);          } /* RES  2,L         */
OP(cb,96) { wm(z80, HL, res(2, rm(z80, HL))); } /* RES  2,(HL)      */
OP(cb,97) { A = res(2, A);          } /* RES  2,A         */

OP(cb,98) { B = res(3, B);          } /* RES  3,B         */
OP(cb,99) { C = res(3, C);          } /* RES  3,C         */
OP(cb,9a) { D = res(3, D);          } /* RES  3,D         */
OP(cb,9b) { E = res(3, E);          } /* RES  3,E         */
OP(cb,9c) { H = res(3, H);          } /* RES  3,H         */
OP(cb,9d) { L = res(3, L);          } /* RES  3,L         */
OP(cb,9e) { wm(z80, HL, res(3, rm(z80, HL))); } /* RES  3,(HL)      */
OP(cb,9f) { A = res(3, A);          } /* RES  3,A         */

OP(cb,a0) { B = res(4, B);          } /* RES  4,B         */
OP(cb,a1) { C = res(4, C);          } /* RES  4,C         */
OP(cb,a2) { D = res(4, D);          } /* RES  4,D         */
OP(cb,a3) { E = res(4, E);          } /* RES  4,E         */
OP(cb,a4) { H = res(4, H);          } /* RES  4,H         */
OP(cb,a5) { L = res(4, L);          } /* RES  4,L         */
OP(cb,a6) { wm(z80, HL, res(4, rm(z80, HL))); } /* RES  4,(HL)      */
OP(cb,a7) { A = res(4, A);          } /* RES  4,A         */

OP(cb,a8) { B = res(5, B);          } /* RES  5,B         */
OP(cb,a9) { C = res(5, C);          } /* RES  5,C         */
OP(cb,aa) { D = res(5, D);          } /* RES  5,D         */
OP(cb,ab) { E = res(5, E);          } /* RES  5,E         */
OP(cb,ac) { H = res(5, H);          } /* RES  5,H         */
OP(cb,ad) { L = res(5, L);          } /* RES  5,L         */
OP(cb,ae) { wm(z80, HL, res(5, rm(z80, HL))); } /* RES  5,(HL)      */
OP(cb,af) { A = res(5, A);          } /* RES  5,A         */

OP(cb,b0) { B = res(6, B);          } /* RES  6,B         */
OP(cb,b1) { C = res(6, C);          } /* RES  6,C         */
OP(cb,b2) { D = res(6, D);          } /* RES  6,D         */
OP(cb,b3) { E = res(6, E);          } /* RES  6,E         */
OP(cb,b4) { H = res(6, H);          } /* RES  6,H         */
OP(cb,b5) { L = res(6, L);          } /* RES  6,L         */
OP(cb,b6) { wm(z80, HL, res(6, rm(z80, HL))); } /* RES  6,(HL)      */
OP(cb,b7) { A = res(6, A);          } /* RES  6,A         */

OP(cb,b8) { B = res(7, B);          } /* RES  7,B         */
OP(cb,b9) { C = res(7, C);          } /* RES  7,C         */
OP(cb,ba) { D = res(7, D);          } /* RES  7,D         */
OP(cb,bb) { E = res(7, E);          } /* RES  7,E         */
OP(cb,bc) { H = res(7, H);          } /* RES  7,H         */
OP(cb,bd) { L = res(7, L);          } /* RES  7,L         */
OP(cb,be) { wm(z80, HL, res(7, rm(z80, HL))); } /* RES  7,(HL)      */
OP(cb,bf) { A = res(7, A);          } /* RES  7,A         */

OP(cb,c0) { B = set(0, B);          } /* SET  0,B         */
OP(cb,c1) { C = set(0, C);          } /* SET  0,C         */
OP(cb,c2) { D = set(0, D);          } /* SET  0,D         */
OP(cb,c3) { E = set(0, E);          } /* SET  0,E         */
OP(cb,c4) { H = set(0, H);          } /* SET  0,H         */
OP(cb,c5) { L = set(0, L);          } /* SET  0,L         */
OP(cb,c6) { wm(z80, HL, set(0, rm(z80, HL))); } /* SET  0,(HL)      */
OP(cb,c7) { A = set(0, A);          } /* SET  0,A         */

OP(cb,c8) { B = set(1, B);          } /* SET  1,B         */
OP(cb,c9) { C = set(1, C);          } /* SET  1,C         */
OP(cb,ca) { D = set(1, D);          } /* SET  1,D         */
OP(cb,cb) { E = set(1, E);          } /* SET  1,E         */
OP(cb,cc) { H = set(1, H);          } /* SET  1,H         */
OP(cb,cd) { L = set(1, L);          } /* SET  1,L         */
OP(cb,ce) { wm(z80, HL, set(1, rm(z80, HL))); } /* SET  1,(HL)      */
OP(cb,cf) { A = set(1, A);          } /* SET  1,A         */

OP(cb,d0) { B = set(2, B);          } /* SET  2,B         */
OP(cb,d1) { C = set(2, C);          } /* SET  2,C         */
OP(cb,d2) { D = set(2, D);          } /* SET  2,D         */
OP(cb,d3) { E = set(2, E);          } /* SET  2,E         */
OP(cb,d4) { H = set(2, H);          } /* SET  2,H         */
OP(cb,d5) { L = set(2, L);          } /* SET  2,L         */
OP(cb,d6) { wm(z80, HL, set(2, rm(z80, HL))); } /* SET  2,(HL)      */
OP(cb,d7) { A = set(2, A);          } /* SET  2,A         */

OP(cb,d8) { B = set(3, B);          } /* SET  3,B         */
OP(cb,d9) { C = set(3, C);          } /* SET  3,C         */
OP(cb,da) { D = set(3, D);          } /* SET  3,D         */
OP(cb,db) { E = set(3, E);          } /* SET  3,E         */
OP(cb,dc) { H = set(3, H);          } /* SET  3,H         */
OP(cb,dd) { L = set(3, L);          } /* SET  3,L         */
OP(cb,de) { wm(z80, HL, set(3, rm(z80, HL))); } /* SET  3,(HL)      */
OP(cb,df) { A = set(3, A);          } /* SET  3,A         */

OP(cb,e0) { B = set(4, B);          } /* SET  4,B         */
OP(cb,e1) { C = set(4, C);          } /* SET  4,C         */
OP(cb,e2) { D = set(4, D);          } /* SET  4,D         */
OP(cb,e3) { E = set(4, E);          } /* SET  4,E         */
OP(cb,e4) { H = set(4, H);          } /* SET  4,H         */
OP(cb,e5) { L = set(4, L);          } /* SET  4,L         */
OP(cb,e6) { wm(z80, HL, set(4, rm(z80, HL))); } /* SET  4,(HL)      */
OP(cb,e7) { A = set(4, A);          } /* SET  4,A         */

OP(cb,e8) { B = set(5, B);          } /* SET  5,B         */
OP(cb,e9) { C = set(5, C);          } /* SET  5,C         */
OP(cb,ea) { D = set(5, D);          } /* SET  5,D         */
OP(cb,eb) { E = set(5, E);          } /* SET  5,E         */
OP(cb,ec) { H = set(5, H);          } /* SET  5,H         */
OP(cb,ed) { L = set(5, L);          } /* SET  5,L         */
OP(cb,ee) { wm(z80, HL, set(5, rm(z80, HL))); } /* SET  5,(HL)      */
OP(cb,ef) { A = set(5, A);          } /* SET  5,A         */

OP(cb,f0) { B = set(6, B);          } /* SET  6,B         */
OP(cb,f1) { C = set(6, C);          } /* SET  6,C         */
OP(cb,f2) { D = set(6, D);          } /* SET  6,D         */
OP(cb,f3) { E = set(6, E);          } /* SET  6,E         */
OP(cb,f4) { H = set(6, H);          } /* SET  6,H         */
OP(cb,f5) { L = set(6, L);          } /* SET  6,L         */
OP(cb,f6) { wm(z80, HL, set(6, rm(z80, HL))); } /* SET  6,(HL)      */
OP(cb,f7) { A = set(6, A);          } /* SET  6,A         */

OP(cb,f8) { B = set(7, B);          } /* SET  7,B         */
OP(cb,f9) { C = set(7, C);          } /* SET  7,C         */
OP(cb,fa) { D = set(7, D);          } /* SET  7,D         */
OP(cb,fb) { E = set(7, E);          } /* SET  7,E         */
OP(cb,fc) { H = set(7, H);          } /* SET  7,H         */
OP(cb,fd) { L = set(7, L);          } /* SET  7,L         */
OP(cb,fe) { wm(z80, HL, set(7, rm(z80, HL))); } /* SET  7,(HL)      */
OP(cb,ff) { A = set(7, A);          } /* SET  7,A         */


/**********************************************************
* opcodes with DD/FD CB prefix
* rotate, shift and bit operations with (IX+o)
**********************************************************/
OP(xycb,00) { B = rlc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);    } /* RLC  B=(XY+o)    */
OP(xycb,01) { C = rlc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);    } /* RLC  C=(XY+o)    */
OP(xycb,02) { D = rlc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);    } /* RLC  D=(XY+o)    */
OP(xycb,03) { E = rlc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);    } /* RLC  E=(XY+o)    */
OP(xycb,04) { H = rlc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);    } /* RLC  H=(XY+o)    */
OP(xycb,05) { L = rlc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);    } /* RLC  L=(XY+o)    */
OP(xycb,06) { wm(z80, z80->m_ea, rlc(z80, rm(z80, z80->m_ea)));           } /* RLC  (XY+o)      */
OP(xycb,07) { A = rlc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);    } /* RLC  A=(XY+o)    */

OP(xycb,08) { B = rrc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);    } /* RRC  B=(XY+o)    */
OP(xycb,09) { C = rrc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);    } /* RRC  C=(XY+o)    */
OP(xycb,0a) { D = rrc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);    } /* RRC  D=(XY+o)    */
OP(xycb,0b) { E = rrc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);    } /* RRC  E=(XY+o)    */
OP(xycb,0c) { H = rrc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);    } /* RRC  H=(XY+o)    */
OP(xycb,0d) { L = rrc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);    } /* RRC  L=(XY+o)    */
OP(xycb,0e) { wm(z80, z80->m_ea,rrc(z80, rm(z80, z80->m_ea)));            } /* RRC  (XY+o)      */
OP(xycb,0f) { A = rrc(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);    } /* RRC  A=(XY+o)    */

OP(xycb,10) { B = rl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);     } /* RL   B=(XY+o)    */
OP(xycb,11) { C = rl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);     } /* RL   C=(XY+o)    */
OP(xycb,12) { D = rl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);     } /* RL   D=(XY+o)    */
OP(xycb,13) { E = rl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);     } /* RL   E=(XY+o)    */
OP(xycb,14) { H = rl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);     } /* RL   H=(XY+o)    */
OP(xycb,15) { L = rl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);     } /* RL   L=(XY+o)    */
OP(xycb,16) { wm(z80, z80->m_ea,rl(z80, rm(z80, z80->m_ea)));             } /* RL   (XY+o)      */
OP(xycb,17) { A = rl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);     } /* RL   A=(XY+o)    */

OP(xycb,18) { B = rr(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);     } /* RR   B=(XY+o)    */
OP(xycb,19) { C = rr(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);     } /* RR   C=(XY+o)    */
OP(xycb,1a) { D = rr(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);     } /* RR   D=(XY+o)    */
OP(xycb,1b) { E = rr(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);     } /* RR   E=(XY+o)    */
OP(xycb,1c) { H = rr(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);     } /* RR   H=(XY+o)    */
OP(xycb,1d) { L = rr(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);     } /* RR   L=(XY+o)    */
OP(xycb,1e) { wm(z80, z80->m_ea, rr(z80, rm(z80, z80->m_ea)));            } /* RR   (XY+o)      */
OP(xycb,1f) { A = rr(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);     } /* RR   A=(XY+o)    */

OP(xycb,20) { B = sla(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);    } /* SLA  B=(XY+o)    */
OP(xycb,21) { C = sla(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);    } /* SLA  C=(XY+o)    */
OP(xycb,22) { D = sla(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);    } /* SLA  D=(XY+o)    */
OP(xycb,23) { E = sla(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);    } /* SLA  E=(XY+o)    */
OP(xycb,24) { H = sla(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);    } /* SLA  H=(XY+o)    */
OP(xycb,25) { L = sla(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);    } /* SLA  L=(XY+o)    */
OP(xycb,26) { wm(z80, z80->m_ea, sla(z80, rm(z80, z80->m_ea)));           } /* SLA  (XY+o)      */
OP(xycb,27) { A = sla(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);    } /* SLA  A=(XY+o)    */

OP(xycb,28) { B = sra(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);    } /* SRA  B=(XY+o)    */
OP(xycb,29) { C = sra(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);    } /* SRA  C=(XY+o)    */
OP(xycb,2a) { D = sra(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);    } /* SRA  D=(XY+o)    */
OP(xycb,2b) { E = sra(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);    } /* SRA  E=(XY+o)    */
OP(xycb,2c) { H = sra(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);    } /* SRA  H=(XY+o)    */
OP(xycb,2d) { L = sra(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);    } /* SRA  L=(XY+o)    */
OP(xycb,2e) { wm(z80, z80->m_ea, sra(z80, rm(z80, z80->m_ea)));           } /* SRA  (XY+o)      */
OP(xycb,2f) { A = sra(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);    } /* SRA  A=(XY+o)    */

OP(xycb,30) { B = sll(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);    } /* SLL  B=(XY+o)    */
OP(xycb,31) { C = sll(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);    } /* SLL  C=(XY+o)    */
OP(xycb,32) { D = sll(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);    } /* SLL  D=(XY+o)    */
OP(xycb,33) { E = sll(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);    } /* SLL  E=(XY+o)    */
OP(xycb,34) { H = sll(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);    } /* SLL  H=(XY+o)    */
OP(xycb,35) { L = sll(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);    } /* SLL  L=(XY+o)    */
OP(xycb,36) { wm(z80, z80->m_ea, sll(z80, rm(z80, z80->m_ea)));           } /* SLL  (XY+o)      */
OP(xycb,37) { A = sll(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);    } /* SLL  A=(XY+o)    */

OP(xycb,38) { B = srl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B);    } /* SRL  B=(XY+o)    */
OP(xycb,39) { C = srl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C);    } /* SRL  C=(XY+o)    */
OP(xycb,3a) { D = srl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D);    } /* SRL  D=(XY+o)    */
OP(xycb,3b) { E = srl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E);    } /* SRL  E=(XY+o)    */
OP(xycb,3c) { H = srl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H);    } /* SRL  H=(XY+o)    */
OP(xycb,3d) { L = srl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L);    } /* SRL  L=(XY+o)    */
OP(xycb,3e) { wm(z80, z80->m_ea, srl(z80, rm(z80, z80->m_ea)));           } /* SRL  (XY+o)      */
OP(xycb,3f) { A = srl(z80, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A);    } /* SRL  A=(XY+o)    */

OP(xycb,40) { xycb_46(z80);                         } /* BIT  0,(XY+o)    */
OP(xycb,41) { xycb_46(z80);                         } /* BIT  0,(XY+o)    */
OP(xycb,42) { xycb_46(z80);                         } /* BIT  0,(XY+o)    */
OP(xycb,43) { xycb_46(z80);                         } /* BIT  0,(XY+o)    */
OP(xycb,44) { xycb_46(z80);                         } /* BIT  0,(XY+o)    */
OP(xycb,45) { xycb_46(z80);                         } /* BIT  0,(XY+o)    */
OP(xycb,46) { bit_xy(z80, 0, rm(z80, z80->m_ea));               } /* BIT  0,(XY+o)    */
OP(xycb,47) { xycb_46(z80);                         } /* BIT  0,(XY+o)    */

OP(xycb,48) { xycb_4e(z80);                         } /* BIT  1,(XY+o)    */
OP(xycb,49) { xycb_4e(z80);                         } /* BIT  1,(XY+o)    */
OP(xycb,4a) { xycb_4e(z80);                         } /* BIT  1,(XY+o)    */
OP(xycb,4b) { xycb_4e(z80);                         } /* BIT  1,(XY+o)    */
OP(xycb,4c) { xycb_4e(z80);                         } /* BIT  1,(XY+o)    */
OP(xycb,4d) { xycb_4e(z80);                         } /* BIT  1,(XY+o)    */
OP(xycb,4e) { bit_xy(z80, 1, rm(z80, z80->m_ea));               } /* BIT  1,(XY+o)    */
OP(xycb,4f) { xycb_4e(z80);                         } /* BIT  1,(XY+o)    */

OP(xycb,50) { xycb_56(z80);                         } /* BIT  2,(XY+o)    */
OP(xycb,51) { xycb_56(z80);                         } /* BIT  2,(XY+o)    */
OP(xycb,52) { xycb_56(z80);                         } /* BIT  2,(XY+o)    */
OP(xycb,53) { xycb_56(z80);                         } /* BIT  2,(XY+o)    */
OP(xycb,54) { xycb_56(z80);                         } /* BIT  2,(XY+o)    */
OP(xycb,55) { xycb_56(z80);                         } /* BIT  2,(XY+o)    */
OP(xycb,56) { bit_xy(z80, 2, rm(z80, z80->m_ea));               } /* BIT  2,(XY+o)    */
OP(xycb,57) { xycb_56(z80);                         } /* BIT  2,(XY+o)    */

OP(xycb,58) { xycb_5e(z80);                         } /* BIT  3,(XY+o)    */
OP(xycb,59) { xycb_5e(z80);                         } /* BIT  3,(XY+o)    */
OP(xycb,5a) { xycb_5e(z80);                         } /* BIT  3,(XY+o)    */
OP(xycb,5b) { xycb_5e(z80);                         } /* BIT  3,(XY+o)    */
OP(xycb,5c) { xycb_5e(z80);                         } /* BIT  3,(XY+o)    */
OP(xycb,5d) { xycb_5e(z80);                         } /* BIT  3,(XY+o)    */
OP(xycb,5e) { bit_xy(z80, 3, rm(z80, z80->m_ea));               } /* BIT  3,(XY+o)    */
OP(xycb,5f) { xycb_5e(z80);                         } /* BIT  3,(XY+o)    */

OP(xycb,60) { xycb_66(z80);                         } /* BIT  4,(XY+o)    */
OP(xycb,61) { xycb_66(z80);                         } /* BIT  4,(XY+o)    */
OP(xycb,62) { xycb_66(z80);                         } /* BIT  4,(XY+o)    */
OP(xycb,63) { xycb_66(z80);                         } /* BIT  4,(XY+o)    */
OP(xycb,64) { xycb_66(z80);                         } /* BIT  4,(XY+o)    */
OP(xycb,65) { xycb_66(z80);                         } /* BIT  4,(XY+o)    */
OP(xycb,66) { bit_xy(z80, 4, rm(z80, z80->m_ea));               } /* BIT  4,(XY+o)    */
OP(xycb,67) { xycb_66(z80);                         } /* BIT  4,(XY+o)    */

OP(xycb,68) { xycb_6e(z80);                         } /* BIT  5,(XY+o)    */
OP(xycb,69) { xycb_6e(z80);                         } /* BIT  5,(XY+o)    */
OP(xycb,6a) { xycb_6e(z80);                         } /* BIT  5,(XY+o)    */
OP(xycb,6b) { xycb_6e(z80);                         } /* BIT  5,(XY+o)    */
OP(xycb,6c) { xycb_6e(z80);                         } /* BIT  5,(XY+o)    */
OP(xycb,6d) { xycb_6e(z80);                         } /* BIT  5,(XY+o)    */
OP(xycb,6e) { bit_xy(z80, 5, rm(z80, z80->m_ea));               } /* BIT  5,(XY+o)    */
OP(xycb,6f) { xycb_6e(z80);                         } /* BIT  5,(XY+o)    */

OP(xycb,70) { xycb_76(z80);                         } /* BIT  6,(XY+o)    */
OP(xycb,71) { xycb_76(z80);                         } /* BIT  6,(XY+o)    */
OP(xycb,72) { xycb_76(z80);                         } /* BIT  6,(XY+o)    */
OP(xycb,73) { xycb_76(z80);                         } /* BIT  6,(XY+o)    */
OP(xycb,74) { xycb_76(z80);                         } /* BIT  6,(XY+o)    */
OP(xycb,75) { xycb_76(z80);                         } /* BIT  6,(XY+o)    */
OP(xycb,76) { bit_xy(z80, 6, rm(z80, z80->m_ea));               } /* BIT  6,(XY+o)    */
OP(xycb,77) { xycb_76(z80);                         } /* BIT  6,(XY+o)    */

OP(xycb,78) { xycb_7e(z80);                         } /* BIT  7,(XY+o)    */
OP(xycb,79) { xycb_7e(z80);                         } /* BIT  7,(XY+o)    */
OP(xycb,7a) { xycb_7e(z80);                         } /* BIT  7,(XY+o)    */
OP(xycb,7b) { xycb_7e(z80);                         } /* BIT  7,(XY+o)    */
OP(xycb,7c) { xycb_7e(z80);                         } /* BIT  7,(XY+o)    */
OP(xycb,7d) { xycb_7e(z80);                         } /* BIT  7,(XY+o)    */
OP(xycb,7e) { bit_xy(z80, 7, rm(z80, z80->m_ea));               } /* BIT  7,(XY+o)    */
OP(xycb,7f) { xycb_7e(z80);                         } /* BIT  7,(XY+o)    */

OP(xycb,80) { B = res(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  0,B=(XY+o)  */
OP(xycb,81) { C = res(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  0,C=(XY+o)  */
OP(xycb,82) { D = res(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  0,D=(XY+o)  */
OP(xycb,83) { E = res(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  0,E=(XY+o)  */
OP(xycb,84) { H = res(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  0,H=(XY+o)  */
OP(xycb,85) { L = res(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  0,L=(XY+o)  */
OP(xycb,86) { wm(z80, z80->m_ea, res(0, rm(z80, z80->m_ea)));        } /* RES  0,(XY+o)    */
OP(xycb,87) { A = res(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  0,A=(XY+o)  */

OP(xycb,88) { B = res(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  1,B=(XY+o)  */
OP(xycb,89) { C = res(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  1,C=(XY+o)  */
OP(xycb,8a) { D = res(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  1,D=(XY+o)  */
OP(xycb,8b) { E = res(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  1,E=(XY+o)  */
OP(xycb,8c) { H = res(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  1,H=(XY+o)  */
OP(xycb,8d) { L = res(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  1,L=(XY+o)  */
OP(xycb,8e) { wm(z80, z80->m_ea, res(1, rm(z80, z80->m_ea)));        } /* RES  1,(XY+o)    */
OP(xycb,8f) { A = res(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  1,A=(XY+o)  */

OP(xycb,90) { B = res(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  2,B=(XY+o)  */
OP(xycb,91) { C = res(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  2,C=(XY+o)  */
OP(xycb,92) { D = res(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  2,D=(XY+o)  */
OP(xycb,93) { E = res(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  2,E=(XY+o)  */
OP(xycb,94) { H = res(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  2,H=(XY+o)  */
OP(xycb,95) { L = res(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  2,L=(XY+o)  */
OP(xycb,96) { wm(z80, z80->m_ea, res(2, rm(z80, z80->m_ea)));        } /* RES  2,(XY+o)    */
OP(xycb,97) { A = res(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  2,A=(XY+o)  */

OP(xycb,98) { B = res(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  3,B=(XY+o)  */
OP(xycb,99) { C = res(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  3,C=(XY+o)  */
OP(xycb,9a) { D = res(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  3,D=(XY+o)  */
OP(xycb,9b) { E = res(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  3,E=(XY+o)  */
OP(xycb,9c) { H = res(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  3,H=(XY+o)  */
OP(xycb,9d) { L = res(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  3,L=(XY+o)  */
OP(xycb,9e) { wm(z80, z80->m_ea, res(3, rm(z80, z80->m_ea)));        } /* RES  3,(XY+o)    */
OP(xycb,9f) { A = res(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  3,A=(XY+o)  */

OP(xycb,a0) { B = res(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  4,B=(XY+o)  */
OP(xycb,a1) { C = res(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  4,C=(XY+o)  */
OP(xycb,a2) { D = res(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  4,D=(XY+o)  */
OP(xycb,a3) { E = res(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  4,E=(XY+o)  */
OP(xycb,a4) { H = res(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  4,H=(XY+o)  */
OP(xycb,a5) { L = res(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  4,L=(XY+o)  */
OP(xycb,a6) { wm(z80, z80->m_ea, res(4, rm(z80, z80->m_ea)));        } /* RES  4,(XY+o)    */
OP(xycb,a7) { A = res(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  4,A=(XY+o)  */

OP(xycb,a8) { B = res(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  5,B=(XY+o)  */
OP(xycb,a9) { C = res(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  5,C=(XY+o)  */
OP(xycb,aa) { D = res(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  5,D=(XY+o)  */
OP(xycb,ab) { E = res(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  5,E=(XY+o)  */
OP(xycb,ac) { H = res(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  5,H=(XY+o)  */
OP(xycb,ad) { L = res(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  5,L=(XY+o)  */
OP(xycb,ae) { wm(z80, z80->m_ea, res(5, rm(z80, z80->m_ea)));        } /* RES  5,(XY+o)    */
OP(xycb,af) { A = res(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  5,A=(XY+o)  */

OP(xycb,b0) { B = res(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  6,B=(XY+o)  */
OP(xycb,b1) { C = res(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  6,C=(XY+o)  */
OP(xycb,b2) { D = res(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  6,D=(XY+o)  */
OP(xycb,b3) { E = res(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  6,E=(XY+o)  */
OP(xycb,b4) { H = res(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  6,H=(XY+o)  */
OP(xycb,b5) { L = res(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  6,L=(XY+o)  */
OP(xycb,b6) { wm(z80, z80->m_ea, res(6, rm(z80, z80->m_ea)));        } /* RES  6,(XY+o)    */
OP(xycb,b7) { A = res(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  6,A=(XY+o)  */

OP(xycb,b8) { B = res(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* RES  7,B=(XY+o)  */
OP(xycb,b9) { C = res(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* RES  7,C=(XY+o)  */
OP(xycb,ba) { D = res(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* RES  7,D=(XY+o)  */
OP(xycb,bb) { E = res(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* RES  7,E=(XY+o)  */
OP(xycb,bc) { H = res(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* RES  7,H=(XY+o)  */
OP(xycb,bd) { L = res(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* RES  7,L=(XY+o)  */
OP(xycb,be) { wm(z80, z80->m_ea, res(7, rm(z80, z80->m_ea)));        } /* RES  7,(XY+o)    */
OP(xycb,bf) { A = res(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* RES  7,A=(XY+o)  */

OP(xycb,c0) { B = set(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  0,B=(XY+o)  */
OP(xycb,c1) { C = set(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  0,C=(XY+o)  */
OP(xycb,c2) { D = set(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  0,D=(XY+o)  */
OP(xycb,c3) { E = set(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  0,E=(XY+o)  */
OP(xycb,c4) { H = set(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  0,H=(XY+o)  */
OP(xycb,c5) { L = set(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  0,L=(XY+o)  */
OP(xycb,c6) { wm(z80, z80->m_ea, set(0, rm(z80, z80->m_ea)));        } /* SET  0,(XY+o)    */
OP(xycb,c7) { A = set(0, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  0,A=(XY+o)  */

OP(xycb,c8) { B = set(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  1,B=(XY+o)  */
OP(xycb,c9) { C = set(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  1,C=(XY+o)  */
OP(xycb,ca) { D = set(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  1,D=(XY+o)  */
OP(xycb,cb) { E = set(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  1,E=(XY+o)  */
OP(xycb,cc) { H = set(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  1,H=(XY+o)  */
OP(xycb,cd) { L = set(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  1,L=(XY+o)  */
OP(xycb,ce) { wm(z80, z80->m_ea, set(1, rm(z80, z80->m_ea)));        } /* SET  1,(XY+o)    */
OP(xycb,cf) { A = set(1, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  1,A=(XY+o)  */

OP(xycb,d0) { B = set(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  2,B=(XY+o)  */
OP(xycb,d1) { C = set(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  2,C=(XY+o)  */
OP(xycb,d2) { D = set(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  2,D=(XY+o)  */
OP(xycb,d3) { E = set(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  2,E=(XY+o)  */
OP(xycb,d4) { H = set(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  2,H=(XY+o)  */
OP(xycb,d5) { L = set(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  2,L=(XY+o)  */
OP(xycb,d6) { wm(z80, z80->m_ea, set(2, rm(z80, z80->m_ea)));        } /* SET  2,(XY+o)    */
OP(xycb,d7) { A = set(2, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  2,A=(XY+o)  */

OP(xycb,d8) { B = set(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  3,B=(XY+o)  */
OP(xycb,d9) { C = set(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  3,C=(XY+o)  */
OP(xycb,da) { D = set(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  3,D=(XY+o)  */
OP(xycb,db) { E = set(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  3,E=(XY+o)  */
OP(xycb,dc) { H = set(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  3,H=(XY+o)  */
OP(xycb,dd) { L = set(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  3,L=(XY+o)  */
OP(xycb,de) { wm(z80, z80->m_ea, set(3, rm(z80, z80->m_ea)));        } /* SET  3,(XY+o)    */
OP(xycb,df) { A = set(3, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  3,A=(XY+o)  */

OP(xycb,e0) { B = set(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  4,B=(XY+o)  */
OP(xycb,e1) { C = set(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  4,C=(XY+o)  */
OP(xycb,e2) { D = set(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  4,D=(XY+o)  */
OP(xycb,e3) { E = set(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  4,E=(XY+o)  */
OP(xycb,e4) { H = set(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  4,H=(XY+o)  */
OP(xycb,e5) { L = set(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  4,L=(XY+o)  */
OP(xycb,e6) { wm(z80, z80->m_ea, set(4, rm(z80, z80->m_ea)));        } /* SET  4,(XY+o)    */
OP(xycb,e7) { A = set(4, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  4,A=(XY+o)  */

OP(xycb,e8) { B = set(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  5,B=(XY+o)  */
OP(xycb,e9) { C = set(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  5,C=(XY+o)  */
OP(xycb,ea) { D = set(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  5,D=(XY+o)  */
OP(xycb,eb) { E = set(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  5,E=(XY+o)  */
OP(xycb,ec) { H = set(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  5,H=(XY+o)  */
OP(xycb,ed) { L = set(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  5,L=(XY+o)  */
OP(xycb,ee) { wm(z80, z80->m_ea, set(5, rm(z80, z80->m_ea)));        } /* SET  5,(XY+o)    */
OP(xycb,ef) { A = set(5, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  5,A=(XY+o)  */

OP(xycb,f0) { B = set(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  6,B=(XY+o)  */
OP(xycb,f1) { C = set(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  6,C=(XY+o)  */
OP(xycb,f2) { D = set(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  6,D=(XY+o)  */
OP(xycb,f3) { E = set(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  6,E=(XY+o)  */
OP(xycb,f4) { H = set(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  6,H=(XY+o)  */
OP(xycb,f5) { L = set(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  6,L=(XY+o)  */
OP(xycb,f6) { wm(z80, z80->m_ea, set(6, rm(z80, z80->m_ea)));        } /* SET  6,(XY+o)    */
OP(xycb,f7) { A = set(6, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  6,A=(XY+o)  */

OP(xycb,f8) { B = set(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, B); } /* SET  7,B=(XY+o)  */
OP(xycb,f9) { C = set(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, C); } /* SET  7,C=(XY+o)  */
OP(xycb,fa) { D = set(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, D); } /* SET  7,D=(XY+o)  */
OP(xycb,fb) { E = set(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, E); } /* SET  7,E=(XY+o)  */
OP(xycb,fc) { H = set(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, H); } /* SET  7,H=(XY+o)  */
OP(xycb,fd) { L = set(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, L); } /* SET  7,L=(XY+o)  */
OP(xycb,fe) { wm(z80, z80->m_ea, set(7, rm(z80, z80->m_ea)));        } /* SET  7,(XY+o)    */
OP(xycb,ff) { A = set(7, rm(z80, z80->m_ea)); wm(z80, z80->m_ea, A); } /* SET  7,A=(XY+o)  */

OP(illegal,1) {
	warning("Z80 ill. opcode $%02x $%02x ($%04x)\n",
			rm(z80, (PCD-1)&0xffff), rm(z80, PCD), PCD-1);
}

/**********************************************************
 * IX register related opcodes (DD prefix)
 **********************************************************/
OP(dd,00) { illegal_1(z80); op_00(z80);                            } /* DB   DD          */
OP(dd,01) { illegal_1(z80); op_01(z80);                            } /* DB   DD          */
OP(dd,02) { illegal_1(z80); op_02(z80);                            } /* DB   DD          */
OP(dd,03) { illegal_1(z80); op_03(z80);                            } /* DB   DD          */
OP(dd,04) { illegal_1(z80); op_04(z80);                            } /* DB   DD          */
OP(dd,05) { illegal_1(z80); op_05(z80);                            } /* DB   DD          */
OP(dd,06) { illegal_1(z80); op_06(z80);                            } /* DB   DD          */
OP(dd,07) { illegal_1(z80); op_07(z80);                            } /* DB   DD          */

OP(dd,08) { illegal_1(z80); op_08(z80);                            } /* DB   DD          */
OP(dd,09) { add16(z80, &z80->m_ix, &z80->m_bc);                               } /* ADD  IX,BC       */
OP(dd,0a) { illegal_1(z80); op_0a(z80);                            } /* DB   DD          */
OP(dd,0b) { illegal_1(z80); op_0b(z80);                            } /* DB   DD          */
OP(dd,0c) { illegal_1(z80); op_0c(z80);                            } /* DB   DD          */
OP(dd,0d) { illegal_1(z80); op_0d(z80);                            } /* DB   DD          */
OP(dd,0e) { illegal_1(z80); op_0e(z80);                            } /* DB   DD          */
OP(dd,0f) { illegal_1(z80); op_0f(z80);                            } /* DB   DD          */

OP(dd,10) { illegal_1(z80); op_10(z80);                            } /* DB   DD          */
OP(dd,11) { illegal_1(z80); op_11(z80);                            } /* DB   DD          */
OP(dd,12) { illegal_1(z80); op_12(z80);                            } /* DB   DD          */
OP(dd,13) { illegal_1(z80); op_13(z80);                            } /* DB   DD          */
OP(dd,14) { illegal_1(z80); op_14(z80);                            } /* DB   DD          */
OP(dd,15) { illegal_1(z80); op_15(z80);                            } /* DB   DD          */
OP(dd,16) { illegal_1(z80); op_16(z80);                            } /* DB   DD          */
OP(dd,17) { illegal_1(z80); op_17(z80);                            } /* DB   DD          */

OP(dd,18) { illegal_1(z80); op_18(z80);                            } /* DB   DD          */
OP(dd,19) { add16(z80, &z80->m_ix, &z80->m_de);                               } /* ADD  IX,DE       */
OP(dd,1a) { illegal_1(z80); op_1a(z80);                            } /* DB   DD          */
OP(dd,1b) { illegal_1(z80); op_1b(z80);                            } /* DB   DD          */
OP(dd,1c) { illegal_1(z80); op_1c(z80);                            } /* DB   DD          */
OP(dd,1d) { illegal_1(z80); op_1d(z80);                            } /* DB   DD          */
OP(dd,1e) { illegal_1(z80); op_1e(z80);                            } /* DB   DD          */
OP(dd,1f) { illegal_1(z80); op_1f(z80);                            } /* DB   DD          */

OP(dd,20) { illegal_1(z80); op_20(z80);                            } /* DB   DD          */
OP(dd,21) { IX = arg16(z80);                                    } /* LD   IX,w        */
OP(dd,22) { z80->m_ea = arg16(z80); wm16(z80, z80->m_ea, &z80->m_ix); WZ = z80->m_ea + 1; } /* LD   (w),IX      */
OP(dd,23) { IX++;                                            } /* INC  IX          */
OP(dd,24) { HX = inc(z80, HX);                                    } /* INC  HX          */
OP(dd,25) { HX = dec(z80, HX);                                    } /* DEC  HX          */
OP(dd,26) { HX = arg(z80);                                      } /* LD   HX,n        */
OP(dd,27) { illegal_1(z80); op_27(z80);                            } /* DB   DD          */

OP(dd,28) { illegal_1(z80); op_28(z80);                            } /* DB   DD          */
OP(dd,29) { add16(z80, &z80->m_ix, &z80->m_ix);                               } /* ADD  IX,IX       */
OP(dd,2a) { z80->m_ea = arg16(z80); rm16(z80, z80->m_ea, &z80->m_ix); WZ = z80->m_ea + 1; } /* LD   IX,(w)      */
OP(dd,2b) { IX--;                                            } /* DEC  IX          */
OP(dd,2c) { LX = inc(z80, LX);                                    } /* INC  LX          */
OP(dd,2d) { LX = dec(z80, LX);                                    } /* DEC  LX          */
OP(dd,2e) { LX = arg(z80);                                      } /* LD   LX,n        */
OP(dd,2f) { illegal_1(z80); op_2f(z80);                            } /* DB   DD          */

OP(dd,30) { illegal_1(z80); op_30(z80);                            } /* DB   DD          */
OP(dd,31) { illegal_1(z80); op_31(z80);                            } /* DB   DD          */
OP(dd,32) { illegal_1(z80); op_32(z80);                            } /* DB   DD          */
OP(dd,33) { illegal_1(z80); op_33(z80);                            } /* DB   DD          */
OP(dd,34) { eax(z80); wm(z80, z80->m_ea, inc(z80, rm(z80, z80->m_ea)));                  } /* INC  (IX+o)      */
OP(dd,35) { eax(z80); wm(z80, z80->m_ea, dec(z80, rm(z80, z80->m_ea)));                  } /* DEC  (IX+o)      */
OP(dd,36) { eax(z80); wm(z80, z80->m_ea, arg(z80));                          } /* LD   (IX+o),n    */
OP(dd,37) { illegal_1(z80); op_37(z80);                            } /* DB   DD          */

OP(dd,38) { illegal_1(z80); op_38(z80);                            } /* DB   DD          */
OP(dd,39) { add16(z80, &z80->m_ix, &z80->m_sp);                               } /* ADD  IX,SP       */
OP(dd,3a) { illegal_1(z80); op_3a(z80);                            } /* DB   DD          */
OP(dd,3b) { illegal_1(z80); op_3b(z80);                            } /* DB   DD          */
OP(dd,3c) { illegal_1(z80); op_3c(z80);                            } /* DB   DD          */
OP(dd,3d) { illegal_1(z80); op_3d(z80);                            } /* DB   DD          */
OP(dd,3e) { illegal_1(z80); op_3e(z80);                            } /* DB   DD          */
OP(dd,3f) { illegal_1(z80); op_3f(z80);                            } /* DB   DD          */

OP(dd,40) { illegal_1(z80); op_40(z80);                            } /* DB   DD          */
OP(dd,41) { illegal_1(z80); op_41(z80);                            } /* DB   DD          */
OP(dd,42) { illegal_1(z80); op_42(z80);                            } /* DB   DD          */
OP(dd,43) { illegal_1(z80); op_43(z80);                            } /* DB   DD          */
OP(dd,44) { B = HX;                                          } /* LD   B,HX        */
OP(dd,45) { B = LX;                                          } /* LD   B,LX        */
OP(dd,46) { eax(z80); B = rm(z80, z80->m_ea);                             } /* LD   B,(IX+o)    */
OP(dd,47) { illegal_1(z80); op_47(z80);                            } /* DB   DD          */

OP(dd,48) { illegal_1(z80); op_48(z80);                            } /* DB   DD          */
OP(dd,49) { illegal_1(z80); op_49(z80);                            } /* DB   DD          */
OP(dd,4a) { illegal_1(z80); op_4a(z80);                            } /* DB   DD          */
OP(dd,4b) { illegal_1(z80); op_4b(z80);                            } /* DB   DD          */
OP(dd,4c) { C = HX;                                          } /* LD   C,HX        */
OP(dd,4d) { C = LX;                                          } /* LD   C,LX        */
OP(dd,4e) { eax(z80); C = rm(z80, z80->m_ea);                             } /* LD   C,(IX+o)    */
OP(dd,4f) { illegal_1(z80); op_4f(z80);                            } /* DB   DD          */

OP(dd,50) { illegal_1(z80); op_50(z80);                            } /* DB   DD          */
OP(dd,51) { illegal_1(z80); op_51(z80);                            } /* DB   DD          */
OP(dd,52) { illegal_1(z80); op_52(z80);                            } /* DB   DD          */
OP(dd,53) { illegal_1(z80); op_53(z80);                            } /* DB   DD          */
OP(dd,54) { D = HX;                                          } /* LD   D,HX        */
OP(dd,55) { D = LX;                                          } /* LD   D,LX        */
OP(dd,56) { eax(z80); D = rm(z80, z80->m_ea);                             } /* LD   D,(IX+o)    */
OP(dd,57) { illegal_1(z80); op_57(z80);                            } /* DB   DD          */

OP(dd,58) { illegal_1(z80); op_58(z80);                            } /* DB   DD          */
OP(dd,59) { illegal_1(z80); op_59(z80);                            } /* DB   DD          */
OP(dd,5a) { illegal_1(z80); op_5a(z80);                            } /* DB   DD          */
OP(dd,5b) { illegal_1(z80); op_5b(z80);                            } /* DB   DD          */
OP(dd,5c) { E = HX;                                          } /* LD   E,HX        */
OP(dd,5d) { E = LX;                                          } /* LD   E,LX        */
OP(dd,5e) { eax(z80); E = rm(z80, z80->m_ea);                             } /* LD   E,(IX+o)    */
OP(dd,5f) { illegal_1(z80); op_5f(z80);                            } /* DB   DD          */

OP(dd,60) { HX = B;                                          } /* LD   HX,B        */
OP(dd,61) { HX = C;                                          } /* LD   HX,C        */
OP(dd,62) { HX = D;                                          } /* LD   HX,D        */
OP(dd,63) { HX = E;                                          } /* LD   HX,E        */
OP(dd,64) {                                                  } /* LD   HX,HX       */
OP(dd,65) { HX = LX;                                         } /* LD   HX,LX       */
OP(dd,66) { eax(z80); H = rm(z80, z80->m_ea);                             } /* LD   H,(IX+o)    */
OP(dd,67) { HX = A;                                          } /* LD   HX,A        */

OP(dd,68) { LX = B;                                          } /* LD   LX,B        */
OP(dd,69) { LX = C;                                          } /* LD   LX,C        */
OP(dd,6a) { LX = D;                                          } /* LD   LX,D        */
OP(dd,6b) { LX = E;                                          } /* LD   LX,E        */
OP(dd,6c) { LX = HX;                                         } /* LD   LX,HX       */
OP(dd,6d) {                                                  } /* LD   LX,LX       */
OP(dd,6e) { eax(z80); L = rm(z80, z80->m_ea);                             } /* LD   L,(IX+o)    */
OP(dd,6f) { LX = A;                                          } /* LD   LX,A        */

OP(dd,70) { eax(z80); wm(z80, z80->m_ea, B);                              } /* LD   (IX+o),B    */
OP(dd,71) { eax(z80); wm(z80, z80->m_ea, C);                              } /* LD   (IX+o),C    */
OP(dd,72) { eax(z80); wm(z80, z80->m_ea, D);                              } /* LD   (IX+o),D    */
OP(dd,73) { eax(z80); wm(z80, z80->m_ea, E);                              } /* LD   (IX+o),E    */
OP(dd,74) { eax(z80); wm(z80, z80->m_ea, H);                              } /* LD   (IX+o),H    */
OP(dd,75) { eax(z80); wm(z80, z80->m_ea, L);                              } /* LD   (IX+o),L    */
OP(dd,76) { illegal_1(z80); op_76(z80);                            } /* DB   DD          */
OP(dd,77) { eax(z80); wm(z80, z80->m_ea, A);                              } /* LD   (IX+o),A    */

OP(dd,78) { illegal_1(z80); op_78(z80);                            } /* DB   DD          */
OP(dd,79) { illegal_1(z80); op_79(z80);                            } /* DB   DD          */
OP(dd,7a) { illegal_1(z80); op_7a(z80);                            } /* DB   DD          */
OP(dd,7b) { illegal_1(z80); op_7b(z80);                            } /* DB   DD          */
OP(dd,7c) { A = HX;                                          } /* LD   A,HX        */
OP(dd,7d) { A = LX;                                          } /* LD   A,LX        */
OP(dd,7e) { eax(z80); A = rm(z80, z80->m_ea);                             } /* LD   A,(IX+o)    */
OP(dd,7f) { illegal_1(z80); op_7f(z80);                            } /* DB   DD          */

OP(dd,80) { illegal_1(z80); op_80(z80);                            } /* DB   DD          */
OP(dd,81) { illegal_1(z80); op_81(z80);                            } /* DB   DD          */
OP(dd,82) { illegal_1(z80); op_82(z80);                            } /* DB   DD          */
OP(dd,83) { illegal_1(z80); op_83(z80);                            } /* DB   DD          */
OP(dd,84) { add_a(z80, HX);                                       } /* ADD  A,HX        */
OP(dd,85) { add_a(z80, LX);                                       } /* ADD  A,LX        */
OP(dd,86) { eax(z80); add_a(z80, rm(z80, z80->m_ea));                          } /* ADD  A,(IX+o)    */
OP(dd,87) { illegal_1(z80); op_87(z80);                            } /* DB   DD          */

OP(dd,88) { illegal_1(z80); op_88(z80);                            } /* DB   DD          */
OP(dd,89) { illegal_1(z80); op_89(z80);                            } /* DB   DD          */
OP(dd,8a) { illegal_1(z80); op_8a(z80);                            } /* DB   DD          */
OP(dd,8b) { illegal_1(z80); op_8b(z80);                            } /* DB   DD          */
OP(dd,8c) { adc_a(z80, HX);                                       } /* ADC  A,HX        */
OP(dd,8d) { adc_a(z80, LX);                                       } /* ADC  A,LX        */
OP(dd,8e) { eax(z80); adc_a(z80, rm(z80, z80->m_ea));                          } /* ADC  A,(IX+o)    */
OP(dd,8f) { illegal_1(z80); op_8f(z80);                            } /* DB   DD          */

OP(dd,90) { illegal_1(z80); op_90(z80);                            } /* DB   DD          */
OP(dd,91) { illegal_1(z80); op_91(z80);                            } /* DB   DD          */
OP(dd,92) { illegal_1(z80); op_92(z80);                            } /* DB   DD          */
OP(dd,93) { illegal_1(z80); op_93(z80);                            } /* DB   DD          */
OP(dd,94) { sub(z80, HX);                                         } /* SUB  HX          */
OP(dd,95) { sub(z80, LX);                                         } /* SUB  LX          */
OP(dd,96) { eax(z80); sub(z80, rm(z80, z80->m_ea));                            } /* SUB  (IX+o)      */
OP(dd,97) { illegal_1(z80); op_97(z80);                            } /* DB   DD          */

OP(dd,98) { illegal_1(z80); op_98(z80);                            } /* DB   DD          */
OP(dd,99) { illegal_1(z80); op_99(z80);                            } /* DB   DD          */
OP(dd,9a) { illegal_1(z80); op_9a(z80);                            } /* DB   DD          */
OP(dd,9b) { illegal_1(z80); op_9b(z80);                            } /* DB   DD          */
OP(dd,9c) { sbc_a(z80, HX);                                       } /* SBC  A,HX        */
OP(dd,9d) { sbc_a(z80, LX);                                       } /* SBC  A,LX        */
OP(dd,9e) { eax(z80); sbc_a(z80, rm(z80, z80->m_ea));                          } /* SBC  A,(IX+o)    */
OP(dd,9f) { illegal_1(z80); op_9f(z80);                            } /* DB   DD          */

OP(dd,a0) { illegal_1(z80); op_a0(z80);                            } /* DB   DD          */
OP(dd,a1) { illegal_1(z80); op_a1(z80);                            } /* DB   DD          */
OP(dd,a2) { illegal_1(z80); op_a2(z80);                            } /* DB   DD          */
OP(dd,a3) { illegal_1(z80); op_a3(z80);                            } /* DB   DD          */
OP(dd,a4) { and_a(z80, HX);                                       } /* AND  HX          */
OP(dd,a5) { and_a(z80, LX);                                       } /* AND  LX          */
OP(dd,a6) { eax(z80); and_a(z80, rm(z80, z80->m_ea));                          } /* AND  (IX+o)      */
OP(dd,a7) { illegal_1(z80); op_a7(z80);                            } /* DB   DD          */

OP(dd,a8) { illegal_1(z80); op_a8(z80);                            } /* DB   DD          */
OP(dd,a9) { illegal_1(z80); op_a9(z80);                            } /* DB   DD          */
OP(dd,aa) { illegal_1(z80); op_aa(z80);                            } /* DB   DD          */
OP(dd,ab) { illegal_1(z80); op_ab(z80);                            } /* DB   DD          */
OP(dd,ac) { xor_a(z80, HX);                                       } /* XOR  HX          */
OP(dd,ad) { xor_a(z80, LX);                                       } /* XOR  LX          */
OP(dd,ae) { eax(z80); xor_a(z80, rm(z80, z80->m_ea));                          } /* XOR  (IX+o)      */
OP(dd,af) { illegal_1(z80); op_af(z80);                            } /* DB   DD          */

OP(dd,b0) { illegal_1(z80); op_b0(z80);                            } /* DB   DD          */
OP(dd,b1) { illegal_1(z80); op_b1(z80);                            } /* DB   DD          */
OP(dd,b2) { illegal_1(z80); op_b2(z80);                            } /* DB   DD          */
OP(dd,b3) { illegal_1(z80); op_b3(z80);                            } /* DB   DD          */
OP(dd,b4) { or_a(z80, HX);                                        } /* OR   HX          */
OP(dd,b5) { or_a(z80, LX);                                        } /* OR   LX          */
OP(dd,b6) { eax(z80); or_a(z80, rm(z80, z80->m_ea));                           } /* OR   (IX+o)      */
OP(dd,b7) { illegal_1(z80); op_b7(z80);                            } /* DB   DD          */

OP(dd,b8) { illegal_1(z80); op_b8(z80);                            } /* DB   DD          */
OP(dd,b9) { illegal_1(z80); op_b9(z80);                            } /* DB   DD          */
OP(dd,ba) { illegal_1(z80); op_ba(z80);                            } /* DB   DD          */
OP(dd,bb) { illegal_1(z80); op_bb(z80);                            } /* DB   DD          */
OP(dd,bc) { cp(z80, HX);                                          } /* CP   HX          */
OP(dd,bd) { cp(z80, LX);                                          } /* CP   LX          */
OP(dd,be) { eax(z80); cp(z80, rm(z80, z80->m_ea));                             } /* CP   (IX+o)      */
OP(dd,bf) { illegal_1(z80); op_bf(z80);                            } /* DB   DD          */

OP(dd,c0) { illegal_1(z80); op_c0(z80);                            } /* DB   DD          */
OP(dd,c1) { illegal_1(z80); op_c1(z80);                            } /* DB   DD          */
OP(dd,c2) { illegal_1(z80); op_c2(z80);                            } /* DB   DD          */
OP(dd,c3) { illegal_1(z80); op_c3(z80);                            } /* DB   DD          */
OP(dd,c4) { illegal_1(z80); op_c4(z80);                            } /* DB   DD          */
OP(dd,c5) { illegal_1(z80); op_c5(z80);                            } /* DB   DD          */
OP(dd,c6) { illegal_1(z80); op_c6(z80);                            } /* DB   DD          */
OP(dd,c7) { illegal_1(z80); op_c7(z80);                            } /* DB   DD          */

OP(dd,c8) { illegal_1(z80); op_c8(z80);                            } /* DB   DD          */
OP(dd,c9) { illegal_1(z80); op_c9(z80);                            } /* DB   DD          */
OP(dd,ca) { illegal_1(z80); op_ca(z80);                            } /* DB   DD          */
OP(dd,cb) { eax(z80); EXEC(xycb,arg(z80));                         } /* **   DD CB xx    */
OP(dd,cc) { illegal_1(z80); op_cc(z80);                            } /* DB   DD          */
OP(dd,cd) { illegal_1(z80); op_cd(z80);                            } /* DB   DD          */
OP(dd,ce) { illegal_1(z80); op_ce(z80);                            } /* DB   DD          */
OP(dd,cf) { illegal_1(z80); op_cf(z80);                            } /* DB   DD          */

OP(dd,d0) { illegal_1(z80); op_d0(z80);                            } /* DB   DD          */
OP(dd,d1) { illegal_1(z80); op_d1(z80);                            } /* DB   DD          */
OP(dd,d2) { illegal_1(z80); op_d2(z80);                            } /* DB   DD          */
OP(dd,d3) { illegal_1(z80); op_d3(z80);                            } /* DB   DD          */
OP(dd,d4) { illegal_1(z80); op_d4(z80);                            } /* DB   DD          */
OP(dd,d5) { illegal_1(z80); op_d5(z80);                            } /* DB   DD          */
OP(dd,d6) { illegal_1(z80); op_d6(z80);                            } /* DB   DD          */
OP(dd,d7) { illegal_1(z80); op_d7(z80);                            } /* DB   DD          */

OP(dd,d8) { illegal_1(z80); op_d8(z80);                            } /* DB   DD          */
OP(dd,d9) { illegal_1(z80); op_d9(z80);                            } /* DB   DD          */
OP(dd,da) { illegal_1(z80); op_da(z80);                            } /* DB   DD          */
OP(dd,db) { illegal_1(z80); op_db(z80);                            } /* DB   DD          */
OP(dd,dc) { illegal_1(z80); op_dc(z80);                            } /* DB   DD          */
OP(dd,dd) { illegal_1(z80); op_dd(z80);                            } /* DB   DD          */
OP(dd,de) { illegal_1(z80); op_de(z80);                            } /* DB   DD          */
OP(dd,df) { illegal_1(z80); op_df(z80);                            } /* DB   DD          */

OP(dd,e0) { illegal_1(z80); op_e0(z80);                            } /* DB   DD          */
OP(dd,e1) { pop(z80, &z80->m_ix);                                       } /* POP  IX          */
OP(dd,e2) { illegal_1(z80); op_e2(z80);                            } /* DB   DD          */
OP(dd,e3) { ex_sp(z80, &z80->m_ix);                                     } /* EX   (SP),IX     */
OP(dd,e4) { illegal_1(z80); op_e4(z80);                            } /* DB   DD          */
OP(dd,e5) { push(z80, &z80->m_ix);                                      } /* PUSH IX          */
OP(dd,e6) { illegal_1(z80); op_e6(z80);                            } /* DB   DD          */
OP(dd,e7) { illegal_1(z80); op_e7(z80);                            } /* DB   DD          */

OP(dd,e8) { illegal_1(z80); op_e8(z80);                            } /* DB   DD          */
OP(dd,e9) { PC = IX;                                         } /* JP   (IX)        */
OP(dd,ea) { illegal_1(z80); op_ea(z80);                            } /* DB   DD          */
OP(dd,eb) { illegal_1(z80); op_eb(z80);                            } /* DB   DD          */
OP(dd,ec) { illegal_1(z80); op_ec(z80);                            } /* DB   DD          */
OP(dd,ed) { illegal_1(z80); op_ed(z80);                            } /* DB   DD          */
OP(dd,ee) { illegal_1(z80); op_ee(z80);                            } /* DB   DD          */
OP(dd,ef) { illegal_1(z80); op_ef(z80);                            } /* DB   DD          */

OP(dd,f0) { illegal_1(z80); op_f0(z80);                            } /* DB   DD          */
OP(dd,f1) { illegal_1(z80); op_f1(z80);                            } /* DB   DD          */
OP(dd,f2) { illegal_1(z80); op_f2(z80);                            } /* DB   DD          */
OP(dd,f3) { illegal_1(z80); op_f3(z80);                            } /* DB   DD          */
OP(dd,f4) { illegal_1(z80); op_f4(z80);                            } /* DB   DD          */
OP(dd,f5) { illegal_1(z80); op_f5(z80);                            } /* DB   DD          */
OP(dd,f6) { illegal_1(z80); op_f6(z80);                            } /* DB   DD          */
OP(dd,f7) { illegal_1(z80); op_f7(z80);                            } /* DB   DD          */

OP(dd,f8) { illegal_1(z80); op_f8(z80);                            } /* DB   DD          */
OP(dd,f9) { SP = IX;                                         } /* LD   SP,IX       */
OP(dd,fa) { illegal_1(z80); op_fa(z80);                            } /* DB   DD          */
OP(dd,fb) { illegal_1(z80); op_fb(z80);                            } /* DB   DD          */
OP(dd,fc) { illegal_1(z80); op_fc(z80);                            } /* DB   DD          */
OP(dd,fd) { illegal_1(z80); op_fd(z80);                            } /* DB   DD          */
OP(dd,fe) { illegal_1(z80); op_fe(z80);                            } /* DB   DD          */
OP(dd,ff) { illegal_1(z80); op_ff(z80);                            } /* DB   DD          */

/**********************************************************
 * IY register related opcodes (FD prefix)
 **********************************************************/
OP(fd,00) { illegal_1(z80); op_00(z80);                            } /* DB   FD          */
OP(fd,01) { illegal_1(z80); op_01(z80);                            } /* DB   FD          */
OP(fd,02) { illegal_1(z80); op_02(z80);                            } /* DB   FD          */
OP(fd,03) { illegal_1(z80); op_03(z80);                            } /* DB   FD          */
OP(fd,04) { illegal_1(z80); op_04(z80);                            } /* DB   FD          */
OP(fd,05) { illegal_1(z80); op_05(z80);                            } /* DB   FD          */
OP(fd,06) { illegal_1(z80); op_06(z80);                            } /* DB   FD          */
OP(fd,07) { illegal_1(z80); op_07(z80);                            } /* DB   FD          */

OP(fd,08) { illegal_1(z80); op_08(z80);                            } /* DB   FD          */
OP(fd,09) { add16(z80, &z80->m_iy, &z80->m_bc);                               } /* ADD  IY,BC       */
OP(fd,0a) { illegal_1(z80); op_0a(z80);                            } /* DB   FD          */
OP(fd,0b) { illegal_1(z80); op_0b(z80);                            } /* DB   FD          */
OP(fd,0c) { illegal_1(z80); op_0c(z80);                            } /* DB   FD          */
OP(fd,0d) { illegal_1(z80); op_0d(z80);                            } /* DB   FD          */
OP(fd,0e) { illegal_1(z80); op_0e(z80);                            } /* DB   FD          */
OP(fd,0f) { illegal_1(z80); op_0f(z80);                            } /* DB   FD          */

OP(fd,10) { illegal_1(z80); op_10(z80);                            } /* DB   FD          */
OP(fd,11) { illegal_1(z80); op_11(z80);                            } /* DB   FD          */
OP(fd,12) { illegal_1(z80); op_12(z80);                            } /* DB   FD          */
OP(fd,13) { illegal_1(z80); op_13(z80);                            } /* DB   FD          */
OP(fd,14) { illegal_1(z80); op_14(z80);                            } /* DB   FD          */
OP(fd,15) { illegal_1(z80); op_15(z80);                            } /* DB   FD          */
OP(fd,16) { illegal_1(z80); op_16(z80);                            } /* DB   FD          */
OP(fd,17) { illegal_1(z80); op_17(z80);                            } /* DB   FD          */

OP(fd,18) { illegal_1(z80); op_18(z80);                            } /* DB   FD          */
OP(fd,19) { add16(z80, &z80->m_iy, &z80->m_de);                               } /* ADD  IY,DE       */
OP(fd,1a) { illegal_1(z80); op_1a(z80);                            } /* DB   FD          */
OP(fd,1b) { illegal_1(z80); op_1b(z80);                            } /* DB   FD          */
OP(fd,1c) { illegal_1(z80); op_1c(z80);                            } /* DB   FD          */
OP(fd,1d) { illegal_1(z80); op_1d(z80);                            } /* DB   FD          */
OP(fd,1e) { illegal_1(z80); op_1e(z80);                            } /* DB   FD          */
OP(fd,1f) { illegal_1(z80); op_1f(z80);                            } /* DB   FD          */

OP(fd,20) { illegal_1(z80); op_20(z80);                            } /* DB   FD          */
OP(fd,21) { IY = arg16(z80);                                    } /* LD   IY,w        */
OP(fd,22) { z80->m_ea = arg16(z80); wm16(z80, z80->m_ea, &z80->m_iy); WZ = z80->m_ea + 1; } /* LD   (w),IY      */
OP(fd,23) { IY++;                                            } /* INC  IY          */
OP(fd,24) { HY = inc(z80, HY);                                    } /* INC  HY          */
OP(fd,25) { HY = dec(z80, HY);                                    } /* DEC  HY          */
OP(fd,26) { HY = arg(z80);                                      } /* LD   HY,n        */
OP(fd,27) { illegal_1(z80); op_27(z80);                            } /* DB   FD          */

OP(fd,28) { illegal_1(z80); op_28(z80);                            } /* DB   FD          */
OP(fd,29) { add16(z80, &z80->m_iy, &z80->m_iy);                               } /* ADD  IY,IY       */
OP(fd,2a) { z80->m_ea = arg16(z80); rm16(z80, z80->m_ea, &z80->m_iy); WZ = z80->m_ea + 1; } /* LD   IY,(w)      */
OP(fd,2b) { IY--;                                            } /* DEC  IY          */
OP(fd,2c) { LY = inc(z80, LY);                                    } /* INC  LY          */
OP(fd,2d) { LY = dec(z80, LY);                                    } /* DEC  LY          */
OP(fd,2e) { LY = arg(z80);                                      } /* LD   LY,n        */
OP(fd,2f) { illegal_1(z80); op_2f(z80);                            } /* DB   FD          */

OP(fd,30) { illegal_1(z80); op_30(z80);                            } /* DB   FD          */
OP(fd,31) { illegal_1(z80); op_31(z80);                            } /* DB   FD          */
OP(fd,32) { illegal_1(z80); op_32(z80);                            } /* DB   FD          */
OP(fd,33) { illegal_1(z80); op_33(z80);                            } /* DB   FD          */
OP(fd,34) { eay(z80); wm(z80, z80->m_ea, inc(z80, rm(z80, z80->m_ea)));                  } /* INC  (IY+o)      */
OP(fd,35) { eay(z80); wm(z80, z80->m_ea, dec(z80, rm(z80, z80->m_ea)));                  } /* DEC  (IY+o)      */
OP(fd,36) { eay(z80); wm(z80, z80->m_ea, arg(z80));                          } /* LD   (IY+o),n    */
OP(fd,37) { illegal_1(z80); op_37(z80);                            } /* DB   FD          */

OP(fd,38) { illegal_1(z80); op_38(z80);                            } /* DB   FD          */
OP(fd,39) { add16(z80, &z80->m_iy, &z80->m_sp);                               } /* ADD  IY,SP       */
OP(fd,3a) { illegal_1(z80); op_3a(z80);                            } /* DB   FD          */
OP(fd,3b) { illegal_1(z80); op_3b(z80);                            } /* DB   FD          */
OP(fd,3c) { illegal_1(z80); op_3c(z80);                            } /* DB   FD          */
OP(fd,3d) { illegal_1(z80); op_3d(z80);                            } /* DB   FD          */
OP(fd,3e) { illegal_1(z80); op_3e(z80);                            } /* DB   FD          */
OP(fd,3f) { illegal_1(z80); op_3f(z80);                            } /* DB   FD          */

OP(fd,40) { illegal_1(z80); op_40(z80);                            } /* DB   FD          */
OP(fd,41) { illegal_1(z80); op_41(z80);                            } /* DB   FD          */
OP(fd,42) { illegal_1(z80); op_42(z80);                            } /* DB   FD          */
OP(fd,43) { illegal_1(z80); op_43(z80);                            } /* DB   FD          */
OP(fd,44) { B = HY;                                          } /* LD   B,HY        */
OP(fd,45) { B = LY;                                          } /* LD   B,LY        */
OP(fd,46) { eay(z80); B = rm(z80, z80->m_ea);                             } /* LD   B,(IY+o)    */
OP(fd,47) { illegal_1(z80); op_47(z80);                            } /* DB   FD          */

OP(fd,48) { illegal_1(z80); op_48(z80);                            } /* DB   FD          */
OP(fd,49) { illegal_1(z80); op_49(z80);                            } /* DB   FD          */
OP(fd,4a) { illegal_1(z80); op_4a(z80);                            } /* DB   FD          */
OP(fd,4b) { illegal_1(z80); op_4b(z80);                            } /* DB   FD          */
OP(fd,4c) { C = HY;                                          } /* LD   C,HY        */
OP(fd,4d) { C = LY;                                          } /* LD   C,LY        */
OP(fd,4e) { eay(z80); C = rm(z80, z80->m_ea);                             } /* LD   C,(IY+o)    */
OP(fd,4f) { illegal_1(z80); op_4f(z80);                            } /* DB   FD          */

OP(fd,50) { illegal_1(z80); op_50(z80);                            } /* DB   FD          */
OP(fd,51) { illegal_1(z80); op_51(z80);                            } /* DB   FD          */
OP(fd,52) { illegal_1(z80); op_52(z80);                            } /* DB   FD          */
OP(fd,53) { illegal_1(z80); op_53(z80);                            } /* DB   FD          */
OP(fd,54) { D = HY;                                          } /* LD   D,HY        */
OP(fd,55) { D = LY;                                          } /* LD   D,LY        */
OP(fd,56) { eay(z80); D = rm(z80, z80->m_ea);                             } /* LD   D,(IY+o)    */
OP(fd,57) { illegal_1(z80); op_57(z80);                            } /* DB   FD          */

OP(fd,58) { illegal_1(z80); op_58(z80);                            } /* DB   FD          */
OP(fd,59) { illegal_1(z80); op_59(z80);                            } /* DB   FD          */
OP(fd,5a) { illegal_1(z80); op_5a(z80);                            } /* DB   FD          */
OP(fd,5b) { illegal_1(z80); op_5b(z80);                            } /* DB   FD          */
OP(fd,5c) { E = HY;                                          } /* LD   E,HY        */
OP(fd,5d) { E = LY;                                          } /* LD   E,LY        */
OP(fd,5e) { eay(z80); E = rm(z80, z80->m_ea);                             } /* LD   E,(IY+o)    */
OP(fd,5f) { illegal_1(z80); op_5f(z80);                            } /* DB   FD          */

OP(fd,60) { HY = B;                                          } /* LD   HY,B        */
OP(fd,61) { HY = C;                                          } /* LD   HY,C        */
OP(fd,62) { HY = D;                                          } /* LD   HY,D        */
OP(fd,63) { HY = E;                                          } /* LD   HY,E        */
OP(fd,64) {                                                  } /* LD   HY,HY       */
OP(fd,65) { HY = LY;                                         } /* LD   HY,LY       */
OP(fd,66) { eay(z80); H = rm(z80, z80->m_ea);                             } /* LD   H,(IY+o)    */
OP(fd,67) { HY = A;                                          } /* LD   HY,A        */

OP(fd,68) { LY = B;                                          } /* LD   LY,B        */
OP(fd,69) { LY = C;                                          } /* LD   LY,C        */
OP(fd,6a) { LY = D;                                          } /* LD   LY,D        */
OP(fd,6b) { LY = E;                                          } /* LD   LY,E        */
OP(fd,6c) { LY = HY;                                         } /* LD   LY,HY       */
OP(fd,6d) {                                                  } /* LD   LY,LY       */
OP(fd,6e) { eay(z80); L = rm(z80, z80->m_ea);                             } /* LD   L,(IY+o)    */
OP(fd,6f) { LY = A;                                          } /* LD   LY,A        */

OP(fd,70) { eay(z80); wm(z80, z80->m_ea, B);                              } /* LD   (IY+o),B    */
OP(fd,71) { eay(z80); wm(z80, z80->m_ea, C);                              } /* LD   (IY+o),C    */
OP(fd,72) { eay(z80); wm(z80, z80->m_ea, D);                              } /* LD   (IY+o),D    */
OP(fd,73) { eay(z80); wm(z80, z80->m_ea, E);                              } /* LD   (IY+o),E    */
OP(fd,74) { eay(z80); wm(z80, z80->m_ea, H);                              } /* LD   (IY+o),H    */
OP(fd,75) { eay(z80); wm(z80, z80->m_ea, L);                              } /* LD   (IY+o),L    */
OP(fd,76) { illegal_1(z80); op_76(z80);                            } /* DB   FD          */
OP(fd,77) { eay(z80); wm(z80, z80->m_ea, A);                              } /* LD   (IY+o),A    */

OP(fd,78) { illegal_1(z80); op_78(z80);                            } /* DB   FD          */
OP(fd,79) { illegal_1(z80); op_79(z80);                            } /* DB   FD          */
OP(fd,7a) { illegal_1(z80); op_7a(z80);                            } /* DB   FD          */
OP(fd,7b) { illegal_1(z80); op_7b(z80);                            } /* DB   FD          */
OP(fd,7c) { A = HY;                                          } /* LD   A,HY        */
OP(fd,7d) { A = LY;                                          } /* LD   A,LY        */
OP(fd,7e) { eay(z80); A = rm(z80, z80->m_ea);                             } /* LD   A,(IY+o)    */
OP(fd,7f) { illegal_1(z80); op_7f(z80);                            } /* DB   FD          */

OP(fd,80) { illegal_1(z80); op_80(z80);                            } /* DB   FD          */
OP(fd,81) { illegal_1(z80); op_81(z80);                            } /* DB   FD          */
OP(fd,82) { illegal_1(z80); op_82(z80);                            } /* DB   FD          */
OP(fd,83) { illegal_1(z80); op_83(z80);                            } /* DB   FD          */
OP(fd,84) { add_a(z80, HY);                                       } /* ADD  A,HY        */
OP(fd,85) { add_a(z80, LY);                                       } /* ADD  A,LY        */
OP(fd,86) { eay(z80); add_a(z80, rm(z80, z80->m_ea));                          } /* ADD  A,(IY+o)    */
OP(fd,87) { illegal_1(z80); op_87(z80);                            } /* DB   FD          */

OP(fd,88) { illegal_1(z80); op_88(z80);                            } /* DB   FD          */
OP(fd,89) { illegal_1(z80); op_89(z80);                            } /* DB   FD          */
OP(fd,8a) { illegal_1(z80); op_8a(z80);                            } /* DB   FD          */
OP(fd,8b) { illegal_1(z80); op_8b(z80);                            } /* DB   FD          */
OP(fd,8c) { adc_a(z80, HY);                                       } /* ADC  A,HY        */
OP(fd,8d) { adc_a(z80, LY);                                       } /* ADC  A,LY        */
OP(fd,8e) { eay(z80); adc_a(z80, rm(z80, z80->m_ea));                          } /* ADC  A,(IY+o)    */
OP(fd,8f) { illegal_1(z80); op_8f(z80);                            } /* DB   FD          */

OP(fd,90) { illegal_1(z80); op_90(z80);                            } /* DB   FD          */
OP(fd,91) { illegal_1(z80); op_91(z80);                            } /* DB   FD          */
OP(fd,92) { illegal_1(z80); op_92(z80);                            } /* DB   FD          */
OP(fd,93) { illegal_1(z80); op_93(z80);                            } /* DB   FD          */
OP(fd,94) { sub(z80, HY);                                         } /* SUB  HY          */
OP(fd,95) { sub(z80, LY);                                         } /* SUB  LY          */
OP(fd,96) { eay(z80); sub(z80, rm(z80, z80->m_ea));                            } /* SUB  (IY+o)      */
OP(fd,97) { illegal_1(z80); op_97(z80);                            } /* DB   FD          */

OP(fd,98) { illegal_1(z80); op_98(z80);                            } /* DB   FD          */
OP(fd,99) { illegal_1(z80); op_99(z80);                            } /* DB   FD          */
OP(fd,9a) { illegal_1(z80); op_9a(z80);                            } /* DB   FD          */
OP(fd,9b) { illegal_1(z80); op_9b(z80);                            } /* DB   FD          */
OP(fd,9c) { sbc_a(z80, HY);                                       } /* SBC  A,HY        */
OP(fd,9d) { sbc_a(z80, LY);                                       } /* SBC  A,LY        */
OP(fd,9e) { eay(z80); sbc_a(z80, rm(z80, z80->m_ea));                          } /* SBC  A,(IY+o)    */
OP(fd,9f) { illegal_1(z80); op_9f(z80);                            } /* DB   FD          */

OP(fd,a0) { illegal_1(z80); op_a0(z80);                            } /* DB   FD          */
OP(fd,a1) { illegal_1(z80); op_a1(z80);                            } /* DB   FD          */
OP(fd,a2) { illegal_1(z80); op_a2(z80);                            } /* DB   FD          */
OP(fd,a3) { illegal_1(z80); op_a3(z80);                            } /* DB   FD          */
OP(fd,a4) { and_a(z80, HY);                                       } /* AND  HY          */
OP(fd,a5) { and_a(z80, LY);                                       } /* AND  LY          */
OP(fd,a6) { eay(z80); and_a(z80, rm(z80, z80->m_ea));                          } /* AND  (IY+o)      */
OP(fd,a7) { illegal_1(z80); op_a7(z80);                            } /* DB   FD          */

OP(fd,a8) { illegal_1(z80); op_a8(z80);                            } /* DB   FD          */
OP(fd,a9) { illegal_1(z80); op_a9(z80);                            } /* DB   FD          */
OP(fd,aa) { illegal_1(z80); op_aa(z80);                            } /* DB   FD          */
OP(fd,ab) { illegal_1(z80); op_ab(z80);                            } /* DB   FD          */
OP(fd,ac) { xor_a(z80, HY);                                       } /* XOR  HY          */
OP(fd,ad) { xor_a(z80, LY);                                       } /* XOR  LY          */
OP(fd,ae) { eay(z80); xor_a(z80, rm(z80, z80->m_ea));                          } /* XOR  (IY+o)      */
OP(fd,af) { illegal_1(z80); op_af(z80);                            } /* DB   FD          */

OP(fd,b0) { illegal_1(z80); op_b0(z80);                            } /* DB   FD          */
OP(fd,b1) { illegal_1(z80); op_b1(z80);                            } /* DB   FD          */
OP(fd,b2) { illegal_1(z80); op_b2(z80);                            } /* DB   FD          */
OP(fd,b3) { illegal_1(z80); op_b3(z80);                            } /* DB   FD          */
OP(fd,b4) { or_a(z80, HY);                                        } /* OR   HY          */
OP(fd,b5) { or_a(z80, LY);                                        } /* OR   LY          */
OP(fd,b6) { eay(z80); or_a(z80, rm(z80, z80->m_ea));                           } /* OR   (IY+o)      */
OP(fd,b7) { illegal_1(z80); op_b7(z80);                            } /* DB   FD          */

OP(fd,b8) { illegal_1(z80); op_b8(z80);                            } /* DB   FD          */
OP(fd,b9) { illegal_1(z80); op_b9(z80);                            } /* DB   FD          */
OP(fd,ba) { illegal_1(z80); op_ba(z80);                            } /* DB   FD          */
OP(fd,bb) { illegal_1(z80); op_bb(z80);                            } /* DB   FD          */
OP(fd,bc) { cp(z80, HY);                                          } /* CP   HY          */
OP(fd,bd) { cp(z80, LY);                                          } /* CP   LY          */
OP(fd,be) { eay(z80); cp(z80, rm(z80, z80->m_ea));                             } /* CP   (IY+o)      */
OP(fd,bf) { illegal_1(z80); op_bf(z80);                            } /* DB   FD          */

OP(fd,c0) { illegal_1(z80); op_c0(z80);                            } /* DB   FD          */
OP(fd,c1) { illegal_1(z80); op_c1(z80);                            } /* DB   FD          */
OP(fd,c2) { illegal_1(z80); op_c2(z80);                            } /* DB   FD          */
OP(fd,c3) { illegal_1(z80); op_c3(z80);                            } /* DB   FD          */
OP(fd,c4) { illegal_1(z80); op_c4(z80);                            } /* DB   FD          */
OP(fd,c5) { illegal_1(z80); op_c5(z80);                            } /* DB   FD          */
OP(fd,c6) { illegal_1(z80); op_c6(z80);                            } /* DB   FD          */
OP(fd,c7) { illegal_1(z80); op_c7(z80);                            } /* DB   FD          */

OP(fd,c8) { illegal_1(z80); op_c8(z80);                            } /* DB   FD          */
OP(fd,c9) { illegal_1(z80); op_c9(z80);                            } /* DB   FD          */
OP(fd,ca) { illegal_1(z80); op_ca(z80);                            } /* DB   FD          */
OP(fd,cb) { eay(z80); EXEC(xycb,arg(z80));                         } /* **   FD CB xx    */
OP(fd,cc) { illegal_1(z80); op_cc(z80);                            } /* DB   FD          */
OP(fd,cd) { illegal_1(z80); op_cd(z80);                            } /* DB   FD          */
OP(fd,ce) { illegal_1(z80); op_ce(z80);                            } /* DB   FD          */
OP(fd,cf) { illegal_1(z80); op_cf(z80);                            } /* DB   FD          */

OP(fd,d0) { illegal_1(z80); op_d0(z80);                            } /* DB   FD          */
OP(fd,d1) { illegal_1(z80); op_d1(z80);                            } /* DB   FD          */
OP(fd,d2) { illegal_1(z80); op_d2(z80);                            } /* DB   FD          */
OP(fd,d3) { illegal_1(z80); op_d3(z80);                            } /* DB   FD          */
OP(fd,d4) { illegal_1(z80); op_d4(z80);                            } /* DB   FD          */
OP(fd,d5) { illegal_1(z80); op_d5(z80);                            } /* DB   FD          */
OP(fd,d6) { illegal_1(z80); op_d6(z80);                            } /* DB   FD          */
OP(fd,d7) { illegal_1(z80); op_d7(z80);                            } /* DB   FD          */

OP(fd,d8) { illegal_1(z80); op_d8(z80);                            } /* DB   FD          */
OP(fd,d9) { illegal_1(z80); op_d9(z80);                            } /* DB   FD          */
OP(fd,da) { illegal_1(z80); op_da(z80);                            } /* DB   FD          */
OP(fd,db) { illegal_1(z80); op_db(z80);                            } /* DB   FD          */
OP(fd,dc) { illegal_1(z80); op_dc(z80);                            } /* DB   FD          */
OP(fd,dd) { illegal_1(z80); op_dd(z80);                            } /* DB   FD          */
OP(fd,de) { illegal_1(z80); op_de(z80);                            } /* DB   FD          */
OP(fd,df) { illegal_1(z80); op_df(z80);                            } /* DB   FD          */

OP(fd,e0) { illegal_1(z80); op_e0(z80);                            } /* DB   FD          */
OP(fd,e1) { pop(z80, &z80->m_iy);                                       } /* POP  IY          */
OP(fd,e2) { illegal_1(z80); op_e2(z80);                            } /* DB   FD          */
OP(fd,e3) { ex_sp(z80, &z80->m_iy);                                     } /* EX   (SP),IY     */
OP(fd,e4) { illegal_1(z80); op_e4(z80);                            } /* DB   FD          */
OP(fd,e5) { push(z80, &z80->m_iy);                                      } /* PUSH IY          */
OP(fd,e6) { illegal_1(z80); op_e6(z80);                            } /* DB   FD          */
OP(fd,e7) { illegal_1(z80); op_e7(z80);                            } /* DB   FD          */

OP(fd,e8) { illegal_1(z80); op_e8(z80);                            } /* DB   FD          */
OP(fd,e9) { PC = IY;                                         } /* JP   (IY)        */
OP(fd,ea) { illegal_1(z80); op_ea(z80);                            } /* DB   FD          */
OP(fd,eb) { illegal_1(z80); op_eb(z80);                            } /* DB   FD          */
OP(fd,ec) { illegal_1(z80); op_ec(z80);                            } /* DB   FD          */
OP(fd,ed) { illegal_1(z80); op_ed(z80);                            } /* DB   FD          */
OP(fd,ee) { illegal_1(z80); op_ee(z80);                            } /* DB   FD          */
OP(fd,ef) { illegal_1(z80); op_ef(z80);                            } /* DB   FD          */

OP(fd,f0) { illegal_1(z80); op_f0(z80);                            } /* DB   FD          */
OP(fd,f1) { illegal_1(z80); op_f1(z80);                            } /* DB   FD          */
OP(fd,f2) { illegal_1(z80); op_f2(z80);                            } /* DB   FD          */
OP(fd,f3) { illegal_1(z80); op_f3(z80);                            } /* DB   FD          */
OP(fd,f4) { illegal_1(z80); op_f4(z80);                            } /* DB   FD          */
OP(fd,f5) { illegal_1(z80); op_f5(z80);                            } /* DB   FD          */
OP(fd,f6) { illegal_1(z80); op_f6(z80);                            } /* DB   FD          */
OP(fd,f7) { illegal_1(z80); op_f7(z80);                            } /* DB   FD          */

OP(fd,f8) { illegal_1(z80); op_f8(z80);                            } /* DB   FD          */
OP(fd,f9) { SP = IY;                                         } /* LD   SP,IY       */
OP(fd,fa) { illegal_1(z80); op_fa(z80);                            } /* DB   FD          */
OP(fd,fb) { illegal_1(z80); op_fb(z80);                            } /* DB   FD          */
OP(fd,fc) { illegal_1(z80); op_fc(z80);                            } /* DB   FD          */
OP(fd,fd) { illegal_1(z80); op_fd(z80);                            } /* DB   FD          */
OP(fd,fe) { illegal_1(z80); op_fe(z80);                            } /* DB   FD          */
OP(fd,ff) { illegal_1(z80); op_ff(z80);                            } /* DB   FD          */

OP(illegal,2)
{
	warning("Z80 ill. opcode $ed $%02x\n",
			rm(z80, (PCD-1)&0xffff));
}

/**********************************************************
 * special opcodes (ED prefix)
 **********************************************************/
OP(ed,00) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,01) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,02) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,03) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,04) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,05) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,06) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,07) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,08) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,09) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,0a) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,0b) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,0c) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,0d) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,0e) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,0f) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,10) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,11) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,12) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,13) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,14) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,15) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,16) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,17) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,18) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,19) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,1a) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,1b) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,1c) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,1d) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,1e) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,1f) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,20) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,21) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,22) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,23) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,24) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,25) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,26) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,27) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,28) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,29) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,2a) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,2b) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,2c) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,2d) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,2e) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,2f) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,30) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,31) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,32) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,33) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,34) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,35) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,36) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,37) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,38) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,39) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,3a) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,3b) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,3c) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,3d) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,3e) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,3f) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,40) { B = in(z80, BC); F = (F & CF) | SZP[B];               } /* IN   B,(C)       */
OP(ed,41) { out(z80, BC, B);                                      } /* OUT  (C),B       */
OP(ed,42) { sbc_hl(z80, &z80->m_bc);                                    } /* SBC  HL,BC       */
OP(ed,43) { z80->m_ea = arg16(z80); wm16(z80, z80->m_ea, &z80->m_bc); WZ = z80->m_ea + 1; } /* LD   (w),BC      */
OP(ed,44) { neg(z80);                                           } /* NEG              */
OP(ed,45) { retn(z80);                                          } /* RETN             */
OP(ed,46) { z80->m_im = 0;                                        } /* IM   0           */
OP(ed,47) { ld_i_a(z80);                                        } /* LD   i,A         */

OP(ed,48) { C = in(z80, BC); F = (F & CF) | SZP[C];               } /* IN   C,(C)       */
OP(ed,49) { out(z80, BC, C);                                      } /* OUT  (C),C       */
OP(ed,4a) { adc_hl(z80, &z80->m_bc);                                    } /* ADC  HL,BC       */
OP(ed,4b) { z80->m_ea = arg16(z80); rm16(z80, z80->m_ea, &z80->m_bc); WZ = z80->m_ea + 1; } /* LD   BC,(w)      */
OP(ed,4c) { neg(z80);                                           } /* NEG              */
OP(ed,4d) { reti(z80);                                          } /* RETI             */
OP(ed,4e) { z80->m_im = 0;                                        } /* IM   0           */
OP(ed,4f) { ld_r_a(z80);                                        } /* LD   r,A         */

OP(ed,50) { D = in(z80, BC); F = (F & CF) | SZP[D];               } /* IN   D,(C)       */
OP(ed,51) { out(z80, BC, D);                                      } /* OUT  (C),D       */
OP(ed,52) { sbc_hl(z80, &z80->m_de);                                    } /* SBC  HL,DE       */
OP(ed,53) { z80->m_ea = arg16(z80); wm16(z80, z80->m_ea, &z80->m_de); WZ = z80->m_ea + 1; } /* LD   (w),DE      */
OP(ed,54) { neg(z80);                                           } /* NEG              */
OP(ed,55) { retn(z80);                                          } /* RETN             */
OP(ed,56) { z80->m_im = 1;                                        } /* IM   1           */
OP(ed,57) { ld_a_i(z80);                                        } /* LD   A,i         */

OP(ed,58) { E = in(z80, BC); F = (F & CF) | SZP[E];               } /* IN   E,(C)       */
OP(ed,59) { out(z80, BC, E);                                      } /* OUT  (C),E       */
OP(ed,5a) { adc_hl(z80, &z80->m_de);                                    } /* ADC  HL,DE       */
OP(ed,5b) { z80->m_ea = arg16(z80); rm16(z80, z80->m_ea, &z80->m_de); WZ = z80->m_ea + 1; } /* LD   DE,(w)      */
OP(ed,5c) { neg(z80);                                           } /* NEG              */
OP(ed,5d) { reti(z80);                                          } /* RETI             */
OP(ed,5e) { z80->m_im = 2;                                        } /* IM   2           */
OP(ed,5f) { ld_a_r(z80);                                        } /* LD   A,r         */

OP(ed,60) { H = in(z80, BC); F = (F & CF) | SZP[H];               } /* IN   H,(C)       */
OP(ed,61) { out(z80, BC, H);                                      } /* OUT  (C),H       */
OP(ed,62) { sbc_hl(z80, &z80->m_hl);                                    } /* SBC  HL,HL       */
OP(ed,63) { z80->m_ea = arg16(z80); wm16(z80, z80->m_ea, &z80->m_hl); WZ = z80->m_ea + 1; } /* LD   (w),HL      */
OP(ed,64) { neg(z80);                                           } /* NEG              */
OP(ed,65) { retn(z80);                                          } /* RETN             */
OP(ed,66) { z80->m_im = 0;                                        } /* IM   0           */
OP(ed,67) { rrd(z80);                                           } /* RRD  (HL)        */

OP(ed,68) { L = in(z80, BC); F = (F & CF) | SZP[L];               } /* IN   L,(C)       */
OP(ed,69) { out(z80, BC, L);                                      } /* OUT  (C),L       */
OP(ed,6a) { adc_hl(z80, &z80->m_hl);                                    } /* ADC  HL,HL       */
OP(ed,6b) { z80->m_ea = arg16(z80); rm16(z80, z80->m_ea, &z80->m_hl); WZ = z80->m_ea + 1; } /* LD   HL,(w)      */
OP(ed,6c) { neg(z80);                                           } /* NEG              */
OP(ed,6d) { reti(z80);                                          } /* RETI             */
OP(ed,6e) { z80->m_im = 0;                                        } /* IM   0           */
OP(ed,6f) { rld(z80);                                           } /* RLD  (HL)        */

OP(ed,70) { uint8_t res = in(z80, BC); F = (F & CF) | SZP[res];     } /* IN   0,(C)       */
OP(ed,71) { out(z80, BC, 0);                                      } /* OUT  (C),0       */
OP(ed,72) { sbc_hl(z80, &z80->m_sp);                                    } /* SBC  HL,SP       */
OP(ed,73) { z80->m_ea = arg16(z80); wm16(z80, z80->m_ea, &z80->m_sp); WZ = z80->m_ea + 1; } /* LD   (w),SP      */
OP(ed,74) { neg(z80);                                           } /* NEG              */
OP(ed,75) { retn(z80);                                          } /* RETN             */
OP(ed,76) { z80->m_im = 1;                                        } /* IM   1           */
OP(ed,77) { illegal_2(z80);                                     } /* DB   ED,77       */

OP(ed,78) { A = in(z80, BC); F = (F & CF) | SZP[A]; WZ = BC + 1;  } /* IN   A,(C)       */
OP(ed,79) { out(z80, BC, A);  WZ = BC + 1;                        } /* OUT  (C),A       */
OP(ed,7a) { adc_hl(z80, &z80->m_sp);                                    } /* ADC  HL,SP       */
OP(ed,7b) { z80->m_ea = arg16(z80); rm16(z80, z80->m_ea, &z80->m_sp); WZ = z80->m_ea + 1; } /* LD   SP,(w)      */
OP(ed,7c) { neg(z80);                                           } /* NEG              */
OP(ed,7d) { reti(z80);                                          } /* RETI             */
OP(ed,7e) { z80->m_im = 2;                                        } /* IM   2           */
OP(ed,7f) { illegal_2(z80);                                     } /* DB   ED,7F       */

OP(ed,80) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,81) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,82) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,83) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,84) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,85) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,86) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,87) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,88) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,89) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,8a) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,8b) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,8c) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,8d) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,8e) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,8f) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,90) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,91) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,92) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,93) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,94) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,95) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,96) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,97) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,98) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,99) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,9a) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,9b) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,9c) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,9d) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,9e) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,9f) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,a0) { ldi(z80);                                           } /* LDI              */
OP(ed,a1) { cpi(z80);                                           } /* CPI              */
OP(ed,a2) { ini(z80);                                           } /* INI              */
OP(ed,a3) { outi(z80);                                          } /* OUTI             */
OP(ed,a4) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,a5) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,a6) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,a7) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,a8) { ldd(z80);                                           } /* LDD              */
OP(ed,a9) { cpd(z80);                                           } /* CPD              */
OP(ed,aa) { ind(z80);                                           } /* IND              */
OP(ed,ab) { outd(z80);                                          } /* OUTD             */
OP(ed,ac) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ad) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ae) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,af) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,b0) { ldir(z80);                                          } /* LDIR             */
OP(ed,b1) { cpir(z80);                                          } /* CPIR             */
OP(ed,b2) { inir(z80);                                          } /* INIR             */
OP(ed,b3) { otir(z80);                                          } /* OTIR             */
OP(ed,b4) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,b5) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,b6) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,b7) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,b8) { lddr(z80);                                          } /* LDDR             */
OP(ed,b9) { cpdr(z80);                                          } /* CPDR             */
OP(ed,ba) { indr(z80);                                          } /* INDR             */
OP(ed,bb) { otdr(z80);                                          } /* OTDR             */
OP(ed,bc) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,bd) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,be) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,bf) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,c0) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c1) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c2) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c3) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c4) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c5) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c6) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c7) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,c8) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,c9) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ca) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,cb) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,cc) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,cd) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ce) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,cf) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,d0) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d1) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d2) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d3) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d4) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d5) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d6) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d7) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,d8) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,d9) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,da) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,db) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,dc) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,dd) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,de) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,df) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,e0) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e1) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e2) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e3) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e4) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e5) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e6) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e7) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,e8) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,e9) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ea) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,eb) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ec) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ed) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ee) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ef) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,f0) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f1) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f2) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f3) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f4) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f5) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f6) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f7) { illegal_2(z80);                                     } /* DB   ED          */

OP(ed,f8) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,f9) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,fa) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,fb) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,fc) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,fd) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,fe) { illegal_2(z80);                                     } /* DB   ED          */
OP(ed,ff) { illegal_2(z80);                                     } /* DB   ED          */


/**********************************************************
 * main opcodes
 **********************************************************/
OP(op,00) {                                                                       } /* NOP              */
OP(op,01) { BC = arg16(z80);                                                         } /* LD   BC,w        */
OP(op,02) { wm(z80, BC,A); WZ_L = (BC + 1) & 0xFF;  WZ_H = A;                          } /* LD (BC),A */
OP(op,03) { BC++;                                                                 } /* INC  BC          */
OP(op,04) { B = inc(z80, B);                                                           } /* INC  B           */
OP(op,05) { B = dec(z80, B);                                                           } /* DEC  B           */
OP(op,06) { B = arg(z80);                                                            } /* LD   B,n         */
OP(op,07) { rlca(z80);                                                               } /* RLCA             */

OP(op,08) { ex_af(z80);                                                              } /* EX   AF,AF'      */
OP(op,09) { add16(z80, &z80->m_hl, &z80->m_bc);                                                    } /* ADD  HL,BC       */
OP(op,0a) { A = rm(z80, BC);  WZ=BC+1;                                                 } /* LD   A,(BC)      */
OP(op,0b) { BC--;                                                                 } /* DEC  BC          */
OP(op,0c) { C = inc(z80, C);                                                           } /* INC  C           */
OP(op,0d) { C = dec(z80, C);                                                           } /* DEC  C           */
OP(op,0e) { C = arg(z80);                                                            } /* LD   C,n         */
OP(op,0f) { rrca(z80);                                                               } /* RRCA             */

OP(op,10) { B--; jr_cond(z80, B, 0x10);                                                } /* DJNZ o           */
OP(op,11) { DE = arg16(z80);                                                         } /* LD   DE,w        */
OP(op,12) { wm(z80, DE,A); WZ_L = (DE + 1) & 0xFF;  WZ_H = A;                          } /* LD (DE),A */
OP(op,13) { DE++;                                                                 } /* INC  DE          */
OP(op,14) { D = inc(z80, D);                                                           } /* INC  D           */
OP(op,15) { D = dec(z80, D);                                                           } /* DEC  D           */
OP(op,16) { D = arg(z80);                                                            } /* LD   D,n         */
OP(op,17) { rla(z80);                                                                } /* RLA              */

OP(op,18) { jr(z80);                                                                 } /* JR   o           */
OP(op,19) { add16(z80, &z80->m_hl, &z80->m_de);                                                    } /* ADD  HL,DE       */
OP(op,1a) { A = rm(z80, DE); WZ = DE + 1;                                              } /* LD   A,(DE)      */
OP(op,1b) { DE--;                                                                 } /* DEC  DE          */
OP(op,1c) { E = inc(z80, E);                                                           } /* INC  E           */
OP(op,1d) { E = dec(z80, E);                                                           } /* DEC  E           */
OP(op,1e) { E = arg(z80);                                                            } /* LD   E,n         */
OP(op,1f) { rra(z80);                                                                } /* RRA              */

OP(op,20) { jr_cond(z80, !(F & ZF), 0x20);                                             } /* JR   NZ,o        */
OP(op,21) { HL = arg16(z80);                                                         } /* LD   HL,w        */
OP(op,22) { z80->m_ea = arg16(z80); wm16(z80, z80->m_ea, &z80->m_hl); WZ = z80->m_ea + 1;                      } /* LD   (w),HL      */
OP(op,23) { HL++;                                                                 } /* INC  HL          */
OP(op,24) { H = inc(z80, H);                                                           } /* INC  H           */
OP(op,25) { H = dec(z80, H);                                                           } /* DEC  H           */
OP(op,26) { H = arg(z80);                                                            } /* LD   H,n         */
OP(op,27) { daa(z80);                                                                } /* DAA              */

OP(op,28) { jr_cond(z80, F & ZF, 0x28);                                                } /* JR   Z,o         */
OP(op,29) { add16(z80, &z80->m_hl, &z80->m_hl);                                                    } /* ADD  HL,HL       */
OP(op,2a) { z80->m_ea = arg16(z80); rm16(z80, z80->m_ea, &z80->m_hl); WZ = z80->m_ea+1;                        } /* LD   HL,(w)      */
OP(op,2b) { HL--;                                                                 } /* DEC  HL          */
OP(op,2c) { L = inc(z80, L);                                                           } /* INC  L           */
OP(op,2d) { L = dec(z80, L);                                                           } /* DEC  L           */
OP(op,2e) { L = arg(z80);                                                            } /* LD   L,n         */
OP(op,2f) { A ^= 0xff; F = (F & (SF | ZF | PF | CF)) | HF | NF | (A & (YF | XF)); } /* CPL              */

OP(op,30) { jr_cond(z80, !(F & CF), 0x30);                                             } /* JR   NC,o        */
OP(op,31) { SP = arg16(z80);                                                         } /* LD   SP,w        */
OP(op,32) { z80->m_ea = arg16(z80); wm(z80, z80->m_ea, A); WZ_L = (z80->m_ea + 1) & 0xFF; WZ_H = A;      } /* LD   (w),A       */
OP(op,33) { SP++;                                                                 } /* INC  SP          */
OP(op,34) { wm(z80, HL, inc(z80, rm(z80, HL)));                                                  } /* INC  (HL)        */
OP(op,35) { wm(z80, HL, dec(z80, rm(z80, HL)));                                                  } /* DEC  (HL)        */
OP(op,36) { wm(z80, HL, arg(z80));                                                        } /* LD   (HL),n      */
OP(op,37) { F = (F & (SF | ZF | YF | XF | PF)) | CF | (A & (YF | XF));            } /* SCF              */

OP(op,38) { jr_cond(z80, F & CF, 0x38);                                                } /* JR   C,o         */
OP(op,39) { add16(z80, &z80->m_hl, &z80->m_sp);                                                    } /* ADD  HL,SP       */
OP(op,3a) { z80->m_ea = arg16(z80); A = rm(z80, z80->m_ea); WZ = z80->m_ea + 1;                          } /* LD   A,(w)       */
OP(op,3b) { SP--;                                                                 } /* DEC  SP          */
OP(op,3c) { A = inc(z80, A);                                                           } /* INC  A           */
OP(op,3d) { A = dec(z80, A);                                                           } /* DEC  A           */
OP(op,3e) { A = arg(z80);                                                            } /* LD   A,n         */
OP(op,3f) { F = ((F&(SF|ZF|YF|XF|PF|CF))|((F&CF)<<4)|(A&(YF|XF)))^CF;             } /* CCF        */

OP(op,40) {                                                                       } /* LD   B,B         */
OP(op,41) { B = C;                                                                } /* LD   B,C         */
OP(op,42) { B = D;                                                                } /* LD   B,D         */
OP(op,43) { B = E;                                                                } /* LD   B,E         */
OP(op,44) { B = H;                                                                } /* LD   B,H         */
OP(op,45) { B = L;                                                                } /* LD   B,L         */
OP(op,46) { B = rm(z80, HL);                                                           } /* LD   B,(HL)      */
OP(op,47) { B = A;                                                                } /* LD   B,A         */

OP(op,48) { C = B;                                                                } /* LD   C,B         */
OP(op,49) {                                                                       } /* LD   C,C         */
OP(op,4a) { C = D;                                                                } /* LD   C,D         */
OP(op,4b) { C = E;                                                                } /* LD   C,E         */
OP(op,4c) { C = H;                                                                } /* LD   C,H         */
OP(op,4d) { C = L;                                                                } /* LD   C,L         */
OP(op,4e) { C = rm(z80, HL);                                                           } /* LD   C,(HL)      */
OP(op,4f) { C = A;                                                                } /* LD   C,A         */

OP(op,50) { D = B;                                                                } /* LD   D,B         */
OP(op,51) { D = C;                                                                } /* LD   D,C         */
OP(op,52) {                                                                       } /* LD   D,D         */
OP(op,53) { D = E;                                                                } /* LD   D,E         */
OP(op,54) { D = H;                                                                } /* LD   D,H         */
OP(op,55) { D = L;                                                                } /* LD   D,L         */
OP(op,56) { D = rm(z80, HL);                                                           } /* LD   D,(HL)      */
OP(op,57) { D = A;                                                                } /* LD   D,A         */

OP(op,58) { E = B;                                                                } /* LD   E,B         */
OP(op,59) { E = C;                                                                } /* LD   E,C         */
OP(op,5a) { E = D;                                                                } /* LD   E,D         */
OP(op,5b) {                                                                       } /* LD   E,E         */
OP(op,5c) { E = H;                                                                } /* LD   E,H         */
OP(op,5d) { E = L;                                                                } /* LD   E,L         */
OP(op,5e) { E = rm(z80, HL);                                                           } /* LD   E,(HL)      */
OP(op,5f) { E = A;                                                                } /* LD   E,A         */

OP(op,60) { H = B;                                                                } /* LD   H,B         */
OP(op,61) { H = C;                                                                } /* LD   H,C         */
OP(op,62) { H = D;                                                                } /* LD   H,D         */
OP(op,63) { H = E;                                                                } /* LD   H,E         */
OP(op,64) {                                                                       } /* LD   H,H         */
OP(op,65) { H = L;                                                                } /* LD   H,L         */
OP(op,66) { H = rm(z80, HL);                                                           } /* LD   H,(HL)      */
OP(op,67) { H = A;                                                                } /* LD   H,A         */

OP(op,68) { L = B;                                                                } /* LD   L,B         */
OP(op,69) { L = C;                                                                } /* LD   L,C         */
OP(op,6a) { L = D;                                                                } /* LD   L,D         */
OP(op,6b) { L = E;                                                                } /* LD   L,E         */
OP(op,6c) { L = H;                                                                } /* LD   L,H         */
OP(op,6d) {                                                                       } /* LD   L,L         */
OP(op,6e) { L = rm(z80, HL);                                                           } /* LD   L,(HL)      */
OP(op,6f) { L = A;                                                                } /* LD   L,A         */

OP(op,70) { wm(z80, HL, B);                                                            } /* LD   (HL),B      */
OP(op,71) { wm(z80, HL, C);                                                            } /* LD   (HL),C      */
OP(op,72) { wm(z80, HL, D);                                                            } /* LD   (HL),D      */
OP(op,73) { wm(z80, HL, E);                                                            } /* LD   (HL),E      */
OP(op,74) { wm(z80, HL, H);                                                            } /* LD   (HL),H      */
OP(op,75) { wm(z80, HL, L);                                                            } /* LD   (HL),L      */
OP(op,76) { halt(z80);                                                               } /* halt             */
OP(op,77) { wm(z80, HL, A);                                                            } /* LD   (HL),A      */

OP(op,78) { A = B;                                                                } /* LD   A,B         */
OP(op,79) { A = C;                                                                } /* LD   A,C         */
OP(op,7a) { A = D;                                                                } /* LD   A,D         */
OP(op,7b) { A = E;                                                                } /* LD   A,E         */
OP(op,7c) { A = H;                                                                } /* LD   A,H         */
OP(op,7d) { A = L;                                                                } /* LD   A,L         */
OP(op,7e) { A = rm(z80, HL);                                                           } /* LD   A,(HL)      */
OP(op,7f) {                                                                       } /* LD   A,A         */

OP(op,80) { add_a(z80, B);                                                             } /* ADD  A,B         */
OP(op,81) { add_a(z80, C);                                                             } /* ADD  A,C         */
OP(op,82) { add_a(z80, D);                                                             } /* ADD  A,D         */
OP(op,83) { add_a(z80, E);                                                             } /* ADD  A,E         */
OP(op,84) { add_a(z80, H);                                                             } /* ADD  A,H         */
OP(op,85) { add_a(z80, L);                                                             } /* ADD  A,L         */
OP(op,86) { add_a(z80, rm(z80, HL));                                                        } /* ADD  A,(HL)      */
OP(op,87) { add_a(z80, A);                                                             } /* ADD  A,A         */

OP(op,88) { adc_a(z80, B);                                                             } /* ADC  A,B         */
OP(op,89) { adc_a(z80, C);                                                             } /* ADC  A,C         */
OP(op,8a) { adc_a(z80, D);                                                             } /* ADC  A,D         */
OP(op,8b) { adc_a(z80, E);                                                             } /* ADC  A,E         */
OP(op,8c) { adc_a(z80, H);                                                             } /* ADC  A,H         */
OP(op,8d) { adc_a(z80, L);                                                             } /* ADC  A,L         */
OP(op,8e) { adc_a(z80, rm(z80, HL));                                                        } /* ADC  A,(HL)      */
OP(op,8f) { adc_a(z80, A);                                                             } /* ADC  A,A         */

OP(op,90) { sub(z80, B);                                                               } /* SUB  B           */
OP(op,91) { sub(z80, C);                                                               } /* SUB  C           */
OP(op,92) { sub(z80, D);                                                               } /* SUB  D           */
OP(op,93) { sub(z80, E);                                                               } /* SUB  E           */
OP(op,94) { sub(z80, H);                                                               } /* SUB  H           */
OP(op,95) { sub(z80, L);                                                               } /* SUB  L           */
OP(op,96) { sub(z80, rm(z80, HL));                                                          } /* SUB  (HL)        */
OP(op,97) { sub(z80, A);                                                               } /* SUB  A           */

OP(op,98) { sbc_a(z80, B);                                                             } /* SBC  A,B         */
OP(op,99) { sbc_a(z80, C);                                                             } /* SBC  A,C         */
OP(op,9a) { sbc_a(z80, D);                                                             } /* SBC  A,D         */
OP(op,9b) { sbc_a(z80, E);                                                             } /* SBC  A,E         */
OP(op,9c) { sbc_a(z80, H);                                                             } /* SBC  A,H         */
OP(op,9d) { sbc_a(z80, L);                                                             } /* SBC  A,L         */
OP(op,9e) { sbc_a(z80, rm(z80, HL));                                                        } /* SBC  A,(HL)      */
OP(op,9f) { sbc_a(z80, A);                                                             } /* SBC  A,A         */

OP(op,a0) { and_a(z80, B);                                                             } /* AND  B           */
OP(op,a1) { and_a(z80, C);                                                             } /* AND  C           */
OP(op,a2) { and_a(z80, D);                                                             } /* AND  D           */
OP(op,a3) { and_a(z80, E);                                                             } /* AND  E           */
OP(op,a4) { and_a(z80, H);                                                             } /* AND  H           */
OP(op,a5) { and_a(z80, L);                                                             } /* AND  L           */
OP(op,a6) { and_a(z80, rm(z80, HL));                                                        } /* AND  (HL)        */
OP(op,a7) { and_a(z80, A);                                                             } /* AND  A           */

OP(op,a8) { xor_a(z80, B);                                                             } /* XOR  B           */
OP(op,a9) { xor_a(z80, C);                                                             } /* XOR  C           */
OP(op,aa) { xor_a(z80, D);                                                             } /* XOR  D           */
OP(op,ab) { xor_a(z80, E);                                                             } /* XOR  E           */
OP(op,ac) { xor_a(z80, H);                                                             } /* XOR  H           */
OP(op,ad) { xor_a(z80, L);                                                             } /* XOR  L           */
OP(op,ae) { xor_a(z80, rm(z80, HL));                                                        } /* XOR  (HL)        */
OP(op,af) { xor_a(z80, A);                                                             } /* XOR  A           */

OP(op,b0) { or_a(z80, B);                                                              } /* OR   B           */
OP(op,b1) { or_a(z80, C);                                                              } /* OR   C           */
OP(op,b2) { or_a(z80, D);                                                              } /* OR   D           */
OP(op,b3) { or_a(z80, E);                                                              } /* OR   E           */
OP(op,b4) { or_a(z80, H);                                                              } /* OR   H           */
OP(op,b5) { or_a(z80, L);                                                              } /* OR   L           */
OP(op,b6) { or_a(z80, rm(z80, HL));                                                         } /* OR   (HL)        */
OP(op,b7) { or_a(z80, A);                                                              } /* OR   A           */

OP(op,b8) { cp(z80, B);                                                                } /* CP   B           */
OP(op,b9) { cp(z80, C);                                                                } /* CP   C           */
OP(op,ba) { cp(z80, D);                                                                } /* CP   D           */
OP(op,bb) { cp(z80, E);                                                                } /* CP   E           */
OP(op,bc) { cp(z80, H);                                                                } /* CP   H           */
OP(op,bd) { cp(z80, L);                                                                } /* CP   L           */
OP(op,be) { cp(z80, rm(z80, HL));                                                           } /* CP   (HL)        */
OP(op,bf) { cp(z80, A);                                                                } /* CP   A           */

OP(op,c0) { ret_cond(z80, !(F & ZF), 0xc0);                                            } /* RET  NZ          */
OP(op,c1) { pop(z80, &z80->m_bc);                                                            } /* POP  BC          */
OP(op,c2) { jp_cond(z80, !(F & ZF));                                                   } /* JP   NZ,a        */
OP(op,c3) { jp(z80);                                                                 } /* JP   a           */
OP(op,c4) { call_cond(z80, !(F & ZF), 0xc4);                                           } /* CALL NZ,a        */
OP(op,c5) { push(z80, &z80->m_bc);                                                           } /* PUSH BC          */
OP(op,c6) { add_a(z80, arg(z80));                                                         } /* ADD  A,n         */
OP(op,c7) { rst(z80, 0x00);                                                            } /* RST  0           */

OP(op,c8) { ret_cond(z80, F & ZF, 0xc8);                                               } /* RET  Z           */
OP(op,c9) { pop(z80, &z80->m_pc); WZ = PCD;                                                  } /* RET              */
OP(op,ca) { jp_cond(z80, F & ZF);                                                      } /* JP   Z,a         */
OP(op,cb) { z80->m_r++; EXEC(cb,rop(z80));                                                } /* **** CB xx       */
OP(op,cc) { call_cond(z80, F & ZF, 0xcc);                                              } /* CALL Z,a         */
OP(op,cd) { call(z80);                                                               } /* CALL a           */
OP(op,ce) { adc_a(z80, arg(z80));                                                         } /* ADC  A,n         */
OP(op,cf) { rst(z80, 0x08);                                                            } /* RST  1           */

OP(op,d0) { ret_cond(z80, !(F & CF), 0xd0);                                            } /* RET  NC          */
OP(op,d1) { pop(z80, &z80->m_de);                                                            } /* POP  DE          */
OP(op,d2) { jp_cond(z80, !(F & CF));                                                   } /* JP   NC,a        */
OP(op,d3) { unsigned n = arg(z80) | (A << 8); out(z80, n, A); WZ_L = ((n & 0xff) + 1) & 0xff;  WZ_H = A;   } /* OUT  (n),A       */
OP(op,d4) { call_cond(z80, !(F & CF), 0xd4);                                           } /* CALL NC,a        */
OP(op,d5) { push(z80, &z80->m_de);                                                           } /* PUSH DE          */
OP(op,d6) { sub(z80, arg(z80));                                                           } /* SUB  n           */
OP(op,d7) { rst(z80, 0x10);                                                            } /* RST  2           */

OP(op,d8) { ret_cond(z80, F & CF, 0xd8);                                               } /* RET  C           */
OP(op,d9) { exx(z80);                                                                } /* EXX              */
OP(op,da) { jp_cond(z80, F & CF);                                                      } /* JP   C,a         */
OP(op,db) { unsigned n = arg(z80) | (A << 8); A = in(z80, n); WZ = n + 1;                 } /* IN   A,(n)  */
OP(op,dc) { call_cond(z80, F & CF, 0xdc);                                              } /* CALL C,a         */
OP(op,dd) { z80->m_r++; EXEC(dd,rop(z80));                                                } /* **** DD xx       */
OP(op,de) { sbc_a(z80, arg(z80));                                                         } /* SBC  A,n         */
OP(op,df) { rst(z80, 0x18);                                                            } /* RST  3           */

OP(op,e0) { ret_cond(z80, !(F & PF), 0xe0);                                            } /* RET  PO          */
OP(op,e1) { pop(z80, &z80->m_hl);                                                            } /* POP  HL          */
OP(op,e2) { jp_cond(z80, !(F & PF));                                                   } /* JP   PO,a        */
OP(op,e3) { ex_sp(z80, &z80->m_hl);                                                          } /* EX   HL,(SP)     */
OP(op,e4) { call_cond(z80, !(F & PF), 0xe4);                                           } /* CALL PO,a        */
OP(op,e5) { push(z80, &z80->m_hl);                                                           } /* PUSH HL          */
OP(op,e6) { and_a(z80, arg(z80));                                                         } /* AND  n           */
OP(op,e7) { rst(z80, 0x20);                                                            } /* RST  4           */

OP(op,e8) { ret_cond(z80, F & PF, 0xe8);                                               } /* RET  PE          */
OP(op,e9) { PC = HL;                                                              } /* JP   (HL)        */
OP(op,ea) { jp_cond(z80, F & PF);                                                      } /* JP   PE,a        */
OP(op,eb) { ex_de_hl(z80);                                                           } /* EX   DE,HL       */
OP(op,ec) { call_cond(z80, F & PF, 0xec);                                              } /* CALL PE,a        */
OP(op,ed) { z80->m_r++; EXEC(ed,rop(z80));                                                } /* **** ED xx       */
OP(op,ee) { xor_a(z80, arg(z80));                                                         } /* XOR  n           */
OP(op,ef) { rst(z80, 0x28);                                                            } /* RST  5           */

OP(op,f0) { ret_cond(z80, !(F & SF), 0xf0);                                            } /* RET  P           */
OP(op,f1) { pop(z80, &z80->m_af);                                                            } /* POP  AF          */
OP(op,f2) { jp_cond(z80, !(F & SF));                                                   } /* JP   P,a         */
OP(op,f3) { z80->m_iff1 = z80->m_iff2 = 0;                                                  } /* DI               */
OP(op,f4) { call_cond(z80, !(F & SF), 0xf4);                                           } /* CALL P,a         */
OP(op,f5) { push(z80, &z80->m_af);                                                           } /* PUSH AF          */
OP(op,f6) { or_a(z80, arg(z80));                                                          } /* OR   n           */
OP(op,f7) { rst(z80, 0x30);                                                            } /* RST  6           */

OP(op,f8) { ret_cond(z80, F & SF, 0xf8);                                               } /* RET  M           */
OP(op,f9) { SP = HL;                                                              } /* LD   SP,HL       */
OP(op,fa) { jp_cond(z80, F & SF);                                                      } /* JP   M,a         */
OP(op,fb) { ei(z80);                                                                 } /* EI               */
OP(op,fc) { call_cond(z80, F & SF, 0xfc);                                              } /* CALL M,a         */
OP(op,fd) { z80->m_r++; EXEC(fd,rop(z80));                                                } /* **** FD xx       */
OP(op,fe) { cp(z80, arg(z80));                                                            } /* CP   n           */
OP(op,ff) { rst(z80, 0x38);                                                            } /* RST  7           */

/*
void z80_device::take_nmi()
{
	PRVPC = 0xffff; // HACK: segag80r protection kludge

	// Check if processor was halted
	leave_halt(z80);

#if HAS_LDAIR_QUIRK
	// reset parity flag after LD A,I or LD A,R 
	if (m_after_ldair) F &= ~PF;
#endif

	m_iff1 = 0;
	push(m_pc);
	PCD = 0x0066;
	WZ=PCD;
	m_icount -= 11;
	m_nmi_pending = false;
}*/

static void take_interrupt(z80_device *z80)
{
	PRVPC = 0xffff; // HACK: segag80r protection kludge

	// check if processor was halted
	leave_halt(z80);

	// clear both interrupt flip flops
	z80->m_iff1 = z80->m_iff2 = 0;


	// fetch the IRQ vector
	int irq_vector = z80->im2_vector;

	// Interrupt mode 2. Call [i:databyte] 
	if( z80->m_im == 2 )
	{
		// Zilog's datasheet claims that "the least-significant bit must be a zero."
		// However, experiments have confirmed that IM 2 vectors do not have to be
		// even, and all 8 bits will be used; even $FF is handled normally.
		irq_vector = (irq_vector & 0xff) | (z80->m_i << 8);
		push(z80, &z80->m_pc);
		rm16(z80, irq_vector, &z80->m_pc);
		//LOG(("Z80 '%s' IM2 [$%04x] = $%04x\n", tag(), irq_vector, PCD));
		// CALL opcode timing + 'interrupt latency' cycles
		z80->m_icount -= z80->m_cc_op[0xcd] + z80->m_cc_ex[0xff];
	}
	else
	// Interrupt mode 1. RST 38h 
	if( z80->m_im == 1 )
	{
		//LOG(("Z80 '%s' IM1 $0038\n", tag()));
		push(z80, &z80->m_pc);
		PCD = 0x0038;
		// RST $38 + 'interrupt latency' cycles
		z80->m_icount -= z80->m_cc_op[0xff] + cc_ex[0xff];
	}
	else
	{
		// Interrupt mode 0. We check for CALL and JP instructions, 
		// if neither of these were found we assume a 1 byte opcode 
		// was placed on the databus                                
		//LOG(("Z80 '%s' IM0 $%04x\n", tag(), irq_vector));

		// check for nop 
		if (irq_vector != 0x00)
		{
			switch (irq_vector & 0xff0000)
			{
				case 0xcd0000:  // call
					push(z80, &z80->m_pc);
					PCD = irq_vector & 0xffff;
						// CALL $xxxx cycles
					z80->m_icount -= z80->m_cc_op[0xcd];
					break;
				case 0xc30000:  // jump 
					PCD = irq_vector & 0xffff;
					// JP $xxxx cycles 
					z80->m_icount -= z80->m_cc_op[0xc3];
					break;
				default:        // rst (or other opcodes?) 
					push(z80, &z80->m_pc);
					PCD = irq_vector & 0x0038;
					// RST $xx cycles 
					z80->m_icount -= z80->m_cc_op[0xff];
					break;
			}
		}

		// 'interrupt latency' cycles 
		z80->m_icount -= z80->m_cc_ex[0xff];
	}
	WZ=PCD;

#if HAS_LDAIR_QUIRK
	// reset parity flag after LD A,I or LD A,R 
	if (m_after_ldair) F &= ~PF;
#endif
}

void init_z80_opts(z80_options * options, memmap_chunk const * chunks, uint32_t num_chunks, memmap_chunk const * io_chunks, uint32_t num_io_chunks, uint32_t clock_divider, uint32_t io_address_mask)
{
	memset(options, 0, sizeof(*options));
	options->gen.clock_divider = clock_divider;
	options->gen.memmap = chunks;
	options->gen.memmap_chunks = num_chunks;
	options->gen.address_mask = 0xFFFF;
	options->iomap = io_chunks;
	options->io_chunks = num_io_chunks;
	options->io_address_mask = io_address_mask;
}

/****************************************************************************
 * Processor initialization
 ****************************************************************************/
z80_context *init_z80_context(z80_options *opts)
{
	z80_context *z80 = calloc(1, sizeof(z80_context));
	if( !tables_initialised )
	{
		uint8_t *padd = &SZHVC_add[  0*256];
		uint8_t *padc = &SZHVC_add[256*256];
		uint8_t *psub = &SZHVC_sub[  0*256];
		uint8_t *psbc = &SZHVC_sub[256*256];
		for (int oldval = 0; oldval < 256; oldval++)
		{
			for (int newval = 0; newval < 256; newval++)
			{
				/* add or adc w/o carry set */
				int val = newval - oldval;
				*padd = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
				*padd |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if( (newval & 0x0f) < (oldval & 0x0f) ) *padd |= HF;
				if( newval < oldval ) *padd |= CF;
				if( (val^oldval^0x80) & (val^newval) & 0x80 ) *padd |= VF;
				padd++;

				/* adc with carry set */
				val = newval - oldval - 1;
				*padc = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
				*padc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if( (newval & 0x0f) <= (oldval & 0x0f) ) *padc |= HF;
				if( newval <= oldval ) *padc |= CF;
				if( (val^oldval^0x80) & (val^newval) & 0x80 ) *padc |= VF;
				padc++;

				/* cp, sub or sbc w/o carry set */
				val = oldval - newval;
				*psub = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psub |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if( (newval & 0x0f) > (oldval & 0x0f) ) *psub |= HF;
				if( newval > oldval ) *psub |= CF;
				if( (val^oldval) & (oldval^newval) & 0x80 ) *psub |= VF;
				psub++;

				/* sbc with carry set */
				val = oldval - newval - 1;
				*psbc = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psbc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if( (newval & 0x0f) >= (oldval & 0x0f) ) *psbc |= HF;
				if( newval >= oldval ) *psbc |= CF;
				if( (val^oldval) & (oldval^newval) & 0x80 ) *psbc |= VF;
				psbc++;
			}
		}

		for (int i = 0; i < 256; i++)
		{
			int p = 0;
			if( i&0x01 ) ++p;
			if( i&0x02 ) ++p;
			if( i&0x04 ) ++p;
			if( i&0x08 ) ++p;
			if( i&0x10 ) ++p;
			if( i&0x20 ) ++p;
			if( i&0x40 ) ++p;
			if( i&0x80 ) ++p;
			SZ[i] = i ? i & SF : ZF;
			SZ[i] |= (i & (YF | XF));       /* undocumented flag bits 5+3 */
			SZ_BIT[i] = i ? i & SF : ZF | PF;
			SZ_BIT[i] |= (i & (YF | XF));   /* undocumented flag bits 5+3 */
			SZP[i] = SZ[i] | ((p & 1) ? 0 : PF);
			SZHV_inc[i] = SZ[i];
			if( i == 0x80 ) SZHV_inc[i] |= VF;
			if( (i & 0x0f) == 0x00 ) SZHV_inc[i] |= HF;
			SZHV_dec[i] = SZ[i] | NF;
			if( i == 0x7f ) SZHV_dec[i] |= VF;
			if( (i & 0x0f) == 0x0f ) SZHV_dec[i] |= HF;
		}

		tables_initialised = 1;
	}
	z80->options = opts;

	/* Reset registers to their initial values */
	PRVPC = 0;
	PCD = 0;
	SPD = 0;
	AFD = 0;
	BCD = 0;
	DED = 0;
	HLD = 0;
	IXD = 0;
	IYD = 0;
	WZ = 0;
	z80->m_af2.d = 0;
	z80->m_bc2.d = 0;
	z80->m_de2.d = 0;
	z80->m_hl2.d = 0;
	z80->m_r = 0;
	z80->m_r2 = 0;
	z80->m_iff1 = 0;
	z80->m_iff2 = 0;
	z80->m_halt = 0;
	z80->m_im = 0;
	z80->m_i = 0;
	z80->m_nmi_state = 0;
	z80->m_nmi_pending = 0;
	z80->m_irq_state = 0;
	z80->m_wait_state = 0;
	z80->busreq = 0;
	z80->m_after_ei = 0;
	z80->m_after_ldair = 0;
	z80->m_ea = 0;

	IX = IY = 0xffff; /* IX and IY are FFFF after a reset! */
	F = ZF;            /* Zero flag is set */

	/* setup cycle tables */
	z80->m_cc_op = cc_op;
	z80->m_cc_cb = cc_cb;
	z80->m_cc_ed = cc_ed;
	z80->m_cc_xy = cc_xy;
	z80->m_cc_xycb = cc_xycb;
	z80->m_cc_ex = cc_ex;
	
	for (uint32_t address = 0; address < (64*1024); address += 8*1024)
	{
		z80->read_pointers[address >> 13] = NULL;
		z80->write_pointers[address >> 13] = NULL;
		memmap_chunk const *chunk = find_map_chunk(address, &z80->options->gen, 0, NULL);
		if (!chunk || chunk->end < (address + 8*1024) || (chunk->flags & MMAP_PTR_IDX) || !chunk->buffer) {
			continue;
		}
		void *ptr = get_native_pointer(address, (void **)z80->mem_pointers, &z80->options->gen);
		if (!ptr) {
			continue;
		}
		if (chunk->flags & MMAP_READ) {
			z80->read_pointers[address >> 13] = ptr;
		}
		if (chunk->flags & MMAP_WRITE) {
			z80->write_pointers[address >> 13] = ptr;
		}
	}
	
	return z80;
}

/****************************************************************************
 * Do a reset
 ****************************************************************************/
void z80_assert_reset(z80_context *z80, uint32_t cycle)
{
	z80_run(z80, cycle);
	z80->reset = 1;
}
void z80_clear_reset(z80_context *z80, uint32_t cycle)
{
	if (!z80->reset) {
		return;
	}
	z80_run(z80, cycle);
	PC = 0x0000;
	z80->m_i = 0;
	z80->m_r = 0;
	z80->m_r2 = 0;
	//m_nmi_pending = false;
	z80->m_after_ei = 0;
	z80->m_after_ldair = 0;
	z80->m_iff1 = 0;
	z80->m_iff2 = 0;
	z80->reset = 0;

	WZ=PCD;
}

void z80_assert_busreq(z80_context *z80, uint32_t cycle)
{
	z80->busreq = 1;
}

void z80_clear_busreq(z80_context *z80, uint32_t cycle)
{
	z80->busreq = 0;
	z80->busack = 0;
}

uint8_t z80_get_busack(z80_context * context, uint32_t cycle)
{
	z80_run(context, cycle);
	return context->busack;
}

/****************************************************************************
 * Execute 'cycles' T-states.
 ****************************************************************************/
void z80_run(z80_context *z80, uint32_t target_cycle)
{
	if (z80->busack || z80->reset) {
		z80->current_cycle = target_cycle;
		return;
	}
	if (z80->current_cycle >= target_cycle) {
		return;
	}
	uint32_t sync_cycle = target_cycle;
	if (z80->next_int_pulse && (z80->int_pulse_end < z80->current_cycle || z80->int_pulse_end == CYCLE_NEVER)) {
		z80->next_int_pulse(z80);
	}
	z80->m_icount = ((target_cycle - z80->current_cycle) + z80->options->gen.clock_divider - 1) / z80->options->gen.clock_divider;
	int32_t int_icount = INT_MIN;
	if (z80->int_pulse_start < target_cycle) {
		int_icount = (z80->int_pulse_start < z80->current_cycle) ? z80->m_icount
			: ((z80->int_pulse_start - z80->current_cycle) + z80->options->gen.clock_divider - 1) / z80->options->gen.clock_divider;
	}
	do
	{
		// check for interrupts before each instruction
		/*
		//TODO: Interrupts
		if (m_nmi_pending)
			take_nmi(z80);
		else */
		if (z80->m_icount <= int_icount && z80->m_iff1 && !z80->m_after_ei) {
			take_interrupt(z80);
			z80->current_cycle = target_cycle - z80->m_icount * z80->options->gen.clock_divider;
			z80->next_int_pulse(z80);
			if (z80->int_pulse_start < target_cycle) {
				int_icount = (z80->int_pulse_start < z80->current_cycle) ? z80->m_icount
					: ((z80->int_pulse_start - z80->current_cycle) + z80->options->gen.clock_divider - 1) / z80->options->gen.clock_divider;
			}
		}

		z80->m_after_ei = 0;
		z80->m_after_ldair = 0;

		PRVPC = PCD;
		//debugger_instruction_hook(this, PCD);
		z80->m_r++;
		EXEC(op,rop(z80));
		if (z80->busreq) {
			z80->busack = 1;
			z80->m_icount = 0;
		}
	} while (z80->m_icount > 0);
	z80->current_cycle = target_cycle - z80->m_icount * z80->options->gen.clock_divider;
}

/**************************************************************************
 * Generic set_info
 **************************************************************************/

void z80_set_cycle_tables(z80_device *z80, const uint8_t *op, const uint8_t *cb, const uint8_t *ed, const uint8_t *xy, const uint8_t *xycb, const uint8_t *ex)
{
	z80->m_cc_op = (op != NULL) ? op : cc_op;
	z80->m_cc_cb = (cb != NULL) ? cb : cc_cb;
	z80->m_cc_ed = (ed != NULL) ? ed : cc_ed;
	z80->m_cc_xy = (xy != NULL) ? xy : cc_xy;
	z80->m_cc_xycb = (xycb != NULL) ? xycb : cc_xycb;
	z80->m_cc_ex = (ex != NULL) ? ex : cc_ex;
}

void z80_serialize(z80_context *context, serialize_buffer *buf)
{
}
void z80_deserialize(deserialize_buffer *buf, void *vcontext)
{
}

void z80_options_free(z80_options *opts)
{
	free(opts);
}

void z80_assert_nmi(z80_context *context, uint32_t cycle)
{
	context->nmi_start = cycle;
	//check_nmi(context);
}

void z80_adjust_cycles(z80_context * context, uint32_t deduction)
{
	if (context->current_cycle < deduction) {
		fprintf(stderr, "WARNING: Deduction of %u cycles when Z80 cycle counter is only %u\n", deduction, context->current_cycle);
		context->current_cycle = 0;
	} else {
		context->current_cycle -= deduction;
	}
	/*if (context->int_enable_cycle != CYCLE_NEVER) {
		if (context->int_enable_cycle < deduction) {
			context->int_enable_cycle = 0;
		} else {
			context->int_enable_cycle -= deduction;
		}
	}*/
	if (context->int_pulse_start != CYCLE_NEVER) {
		if (context->int_pulse_end < deduction) {
			context->int_pulse_start = context->int_pulse_end = CYCLE_NEVER;
		} else {
			if (context->int_pulse_end != CYCLE_NEVER) {
				context->int_pulse_end -= deduction;
			}
			if (context->int_pulse_start < deduction) {
				context->int_pulse_start = 0;
			} else {
				context->int_pulse_start -= deduction;
			}
		}
	}
}