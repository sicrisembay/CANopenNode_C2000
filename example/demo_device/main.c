/*
 *  ======== main.c ========
 */

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include "DSP2833x_Device.h"

#include "CANopen.h"
#include "OD.h"

/* default values for CO_CANopenInit() */
#define NMT_CONTROL     CO_NMT_STARTUP_TO_OPERATIONAL \
                      | CO_NMT_ERR_ON_ERR_REG         \
                      | CO_ERR_REG_GENERIC_ERR        \
                      | CO_ERR_REG_COMMUNICATION

#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL

static Task_Handle taskHdl_CO;
static Task_Handle taskHdl_CO_timer;
static CO_t * CO = NULL;
static void * CANptr = NULL; /* CAN module address */

#pragma CODE_SECTION(Device_reset, "ramfuncs");
void Device_reset(void)
{
    /* Tickle dog */
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;

    /* Enable watchdog */
    EALLOW;
    SysCtrlRegs.WDCR = 0x0000;  /* writing value other than b101 to WDCHK will immediately resets the device */
    EDIS;

    /* Should not reach here */
    while(1);
}

Void taskCO_timer(UArg a0, UArg a1)
{
    while(1) {
        Task_sleep(1);
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;
            /* get time difference since last function call */
            uint32_t timeDifference_us = 1000;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

            /* Further I/O or nonblocking application code may go here. */
        }
    }
}

Void taskCO_main(UArg a0, UArg a1)
{
    Error_Block eb;
    Task_Params tskParam;

    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    uint8_t pendingNodeId = 10; /* read from dip switches or nonvolatile memory, configurable by LSS slave */
    uint8_t activeNodeId = 10; /* Copied from CO_pendingNodeId in the communication reset section */
    uint16_t pendingBitRate = 250;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */

    /* Allocate CANopen object */
    CO = CO_new(NULL, &heapMemoryUsed);
    if(CO == NULL) {
        System_printf("Error: Can't allocate memory\n");
        BIOS_exit(0);
    } else {
        System_printf("Allocated %d bytes for CANopen objects\n", (int)heapMemoryUsed);
    }

    while(reset != CO_RESET_APP) {
        /* CANopen communication reset - initialize CANopen objects *******************/
        System_printf("CANopenNode - Reset communication...\n");

        /* Wait rt_thread. */
        CO->CANmodule->CANnormal = false;

        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode(CANptr);

        /* Initialize CANopen */
        err = CO_CANinit(CO, CANptr, pendingBitRate);
        if (err != CO_ERROR_NO) {
            System_printf("Error: CAN initialization failed: %d\n", err);
            BIOS_exit(0);
        }

        CO_LSS_address_t lssAddress = {
            .identity = {
                 .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                 .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                 .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                 .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
            }
        };

        err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
        if(err != CO_ERROR_NO) {
            System_printf("Error: LSS slave initialization failed: %d\n", err);
            BIOS_exit(0);
        }

        activeNodeId = pendingNodeId;
        uint32_t errInfo = 0;

        err = CO_CANopenInit(CO,                /* CANopen object */
                             NULL,              /* alternate NMT */
                             NULL,              /* alternate em */
                             OD,                /* Object dictionary */
                             OD_STATUS_BITS,    /* Optional OD_statusBits */
                             NMT_CONTROL,       /* CO_NMT_control_t */
                             FIRST_HB_TIME,     /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                             activeNodeId,
                             &errInfo);

        if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                System_printf("Error: Object Dictionary entry 0x%x\n", errInfo);
            }
            else {
                System_printf("Error: CANopen initialization failed: %d\n", err);
            }
            BIOS_exit(0);
        }

        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if(err != CO_ERROR_NO) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                System_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            }
            else {
                System_printf("Error: PDO initialization failed: %d\n", err);
            }
            BIOS_exit(0);
        }

        /*
         * Configure Timer Task for execution every 1 millisecond
         * Note: taskCO_timer priority must be higher than taskCO_main priority
         */
        Error_init(&eb);
        Task_Params_init(&tskParam);
        tskParam.priority = 2;
        tskParam.stackSize = 512;
        tskParam.instance->name = "taskCO_timer";
        taskHdl_CO_timer = Task_create(taskCO_timer, &tskParam, &eb);
        if (taskHdl_CO_timer == NULL) {
            System_printf("Task_create() CO_timer failed!\n");
            BIOS_exit(0);
        }


        /* Configure CAN transmit and receive interrupt */


        /* Configure CANopen callbacks, etc */


        /* start CAN */
        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        System_printf("CANopenNode - Running...\n");

        while(reset == CO_RESET_NOT) {
            Task_sleep(1);

            /* loop for normal program execution ******************************************/
            /* get time difference since last function call */
            uint32_t timeDifference_us = 1000;

            /* CANopen process */
            reset = CO_process(CO, false, timeDifference_us, NULL);

            /* Nonblocking application code may go here. */


            /* Process automatic storage */


            /* optional sleep for short time */


        }

    };

    /* Complete Device Reset */
    Device_reset();

    /* Must not reach here */
    while(1);
}


extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;

Int main()
{
    Error_Block eb;
    Task_Params tskParam;
    union CANTIOC_REG shadow_cantioc;
    union CANRIOC_REG shadow_canrioc;

    /*
     * Copy ramfuncs section
     */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
           &RamfuncsLoadEnd - &RamfuncsLoadStart);

    System_printf("enter main()\n");

    /*
     * Initialize CAN GPIO
     */
    /* CAN-A pin setting */
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     //Enable pull-up for GPIO31 (CANTXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA
    /* Enable CAN-A clock */
    SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 1;

    CANptr = (void *)(&ECanaRegs);

    shadow_cantioc.all = ECanaRegs.CANTIOC.all;
    shadow_cantioc.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = shadow_cantioc.all;

    shadow_canrioc.all = ECanaRegs.CANRIOC.all;
    shadow_canrioc.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = shadow_canrioc.all;

    EDIS;

    Error_init(&eb);
    Task_Params_init(&tskParam);
    tskParam.priority = 1;
    tskParam.stackSize = 1024;
    tskParam.instance->name = "taskCO_main";
    taskHdl_CO = Task_create(taskCO_main, &tskParam, &eb);
    if (taskHdl_CO == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    BIOS_start();    /* does not return */
    return(0);
}
