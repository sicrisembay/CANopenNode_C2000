/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <xdc/std.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/gates/GateHwi.h>
#include <ti/sysbios/family/c28/Hwi.h>
#include "DSP2833x_Device.h"

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef xdc_UChar uint8_t;
typedef xdc_Char  int8_t;

/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#define CO_CONFIG_LEDS  0

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t            bool_t;
typedef float                   float32_t;
typedef double                  float64_t;


/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg)  \
            ((uint16_t)(((CO_CANrxMsg_t *)msg)->ident))

#define CO_CANrxMsg_readDLC(msg)    \
            ((uint8_t)(((CO_CANrxMsg_t *)msg)->DLC))

#define CO_CANrxMsg_readData(msg)   \
            ((uint8_t *)&(((CO_CANrxMsg_t *)msg)->data[0]))

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

/* Transmit message object */
typedef struct {
    uint32_t mailboxIdx;
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
    volatile bool_t bufferInhibitFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t firstCANtxMessage;
    uint32_t errOld;
    GateHwi_Handle gateHwi_can_send;
    IArg key_can_send;
    GateMutex_Handle mtxHdl_can_emcy;
    IArg keyMtx_can_emcy;
    GateMutex_Handle mtxHdl_can_od;
    IArg keyMtx_can_od;
} CO_CANmodule_t;


/* Data storage object for one entry */
typedef struct {
    void *addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    /* Additional variables (target specific) */
    void *addrNV;
} CO_storage_entry_t;


/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)    \
                CAN_MODULE->key_can_send = GateHwi_enter(CAN_MODULE->gateHwi_can_send)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)  \
                GateHwi_leave(CAN_MODULE->gateHwi_can_send, CAN_MODULE->key_can_send)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)        \
                CAN_MODULE->keyMtx_can_emcy = GateMutex_enter(CAN_MODULE->mtxHdl_can_emcy)
#define CO_UNLOCK_EMCY(CAN_MODULE)      \
        GateMutex_leave(CAN_MODULE->mtxHdl_can_emcy, CAN_MODULE->keyMtx_can_emcy)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)          \
        CAN_MODULE->keyMtx_can_od = GateMutex_enter(CAN_MODULE->mtxHdl_can_od)
#define CO_UNLOCK_OD(CAN_MODULE)        \
        GateMutex_leave(CAN_MODULE->mtxHdl_can_od, CAN_MODULE->keyMtx_can_od)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) {CO_MemoryBarrier(); rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew) {CO_MemoryBarrier(); rxNew = NULL;}


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
