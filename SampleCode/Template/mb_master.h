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

/* -----------------------Master Defines -------------------------------------*/
#define M_DISCRETE_INPUT_START                      0
#define M_DISCRETE_INPUT_NDISCRETES                 16
#define M_COIL_START                                0
#define M_COIL_NCOILS                               64
#define M_REG_INPUT_START                           1000    //0
#define M_REG_INPUT_NREGS                           100
#define M_REG_HOLDING_START                         1000    //0
#define M_REG_HOLDING_NREGS                         100
/* master mode: holding register's all address */
#define M_HD_RESERVE                                0
/* master mode: input register's all address */
#define M_IN_RESERVE                                0
/* master mode: coil's all address */
#define M_CO_RESERVE                                0
/* master mode: discrete's all address */
#define M_DI_RESERVE                                0

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/


