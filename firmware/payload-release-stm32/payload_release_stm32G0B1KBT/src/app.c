#include "app.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_cortex.h"

int app_init() {
    LL_SYSTICK_EnableIT();
}

int app_update() {

}
