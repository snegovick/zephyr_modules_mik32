#ifndef __MIKRON_MIK32_SOC_H
#define __MIKRON_MIK32_SOC_H

#include <zephyr/soc/mik32_memory_map.h>

/* bit operations */
#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#ifndef BIT
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
#endif

#ifndef __ASSEMBLER__

#ifdef __GNUC__

#define read_fpu(reg) ({ unsigned long __tmp; \
  __asm__ volatile ("fmv.x.w %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_fpu(reg, val) ({ \
  if (__builtin_constant_p(val) && (unsigned long)(val) < 32) \
    __asm__ volatile ("fmv.w.x " #reg ", %0" :: "i"(val)); \
  else \
    __asm__ volatile ("fmv.w.x " #reg ", %0" :: "r"(val)); })

#define read_csr(reg) ({ unsigned long __tmp; \
  __asm__ volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  if (__builtin_constant_p(val) && (unsigned long)(val) < 32) \
    __asm__ volatile ("csrw " #reg ", %0" :: "i"(val)); \
  else \
    __asm__ volatile ("csrw " #reg ", %0" :: "r"(val)); })

#define swap_csr(reg, val) ({ unsigned long __tmp; \
  if (__builtin_constant_p(val) && (unsigned long)(val) < 32) \
    __asm__ volatile ("csrrw %0, " #reg ", %1" : "=r"(__tmp) : "i"(val)); \
  else \
    __asm__ volatile ("csrrw %0, " #reg ", %1" : "=r"(__tmp) : "r"(val)); \
  __tmp; })

#define set_csr(reg, bit) ({ unsigned long __tmp; \
  if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32) \
    __asm__ volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "i"(bit)); \
  else \
    __asm__ volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "r"(bit)); \
  __tmp; })

#define clear_csr(reg, bit) ({ unsigned long __tmp; \
  if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32) \
    __asm__ volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "i"(bit)); \
  else \
    __asm__ volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "r"(bit)); \
  __tmp; })

#define rdtime() read_csr(time)
#define rdcycle() read_csr(cycle)
#define rdinstret() read_csr(instret)
#endif

#endif

#define MIE_MEIE MIP_MEIP
#define MIE_MTIE MIP_MTIP

#endif /*__MIKRON_MIK32_SOC_H*/
