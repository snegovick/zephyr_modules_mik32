#include <hal/mik32/peripherals/Include/mik32_hal_irq.h>
#include <hal/mik32/shared/include/mik32_memory_map.h>
#include <hal/mik32/shared/periphery/epic.h>
#include <hal/mik32/shared/periphery/scr1_timer.h>

void scr1_timer_isr();

void __soc_handle_all_irqs(void) {
	unsigned long mcause = read_csr(mcause);
	if ( (mcause & 0xF) == 7 && (mcause & (1<<31)) )
	{
		scr1_timer_isr();
	} else {
		EPIC->CLEAR = 0xfffffffful;
		while (1) {
		}
	}
}
