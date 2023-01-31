/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "mb.h"
#include "mb_m.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbutils.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/* -----------------------Slave Defines -------------------------------------*/
#define S_DISCRETE_INPUT_START                      0
#define S_DISCRETE_INPUT_NDISCRETES                 16
#define S_COIL_START                                0
#define S_COIL_NCOILS                               64
#define S_REG_INPUT_START                           1000        //0
#define S_REG_INPUT_NREGS                           100
#define S_REG_HOLDING_START                         1000        //0
#define S_REG_HOLDING_NREGS                         100
/* salve mode: holding register's all address */
#define S_HD_RESERVE                                0
#define S_HD_CPU_USAGE_MAJOR                        1
#define S_HD_CPU_USAGE_MINOR                        2
/* salve mode: input register's all address */
#define S_IN_RESERVE                                0
/* salve mode: coil's all address */
#define S_CO_RESERVE                                0
/* salve mode: discrete's all address */
#define S_DI_RESERVE                                0
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/


