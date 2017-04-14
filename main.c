#include <stdbool.h>

#include "grbllib.h"
#include "driver.h"

#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"

int main(void) {

    FPUEnable();
    FPULazyStackingEnable();

	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	grbl_enter();

}
