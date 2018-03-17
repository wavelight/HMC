#ifndef _TIMER_H_
#define _TIMER_H_

#include "definition.h"

extern time time_get(void);
extern time time_elapsed(time old, time new);
extern time time_elapsed_now(time old);

#endif