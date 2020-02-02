#ifndef _WWDG_H
#define _WWDG_H

extern uint16_t wwdg_flag;

void WWDG_Init(void);
int SuperiviseTask(void);
void ShootSupervise(void);

#endif
