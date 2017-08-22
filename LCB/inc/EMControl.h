
#define   EM_OFF_DCBUS_MIN          25 
#define   SP_CONSOLE_POWER_ON_RPM   25 
#define   SP_DCBUS_LIMIT            40 

void EMControl_Initial(void);
void EMControl_Running(void);
unsigned short EMControl_GetPWM(void);
//unsigned char EMControl_Check(void);

