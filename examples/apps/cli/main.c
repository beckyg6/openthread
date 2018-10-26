/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <string.h>

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/cli.h>
#include <openthread/dataset_ftd.h>
#include <openthread/diag.h>
#include <openthread/instance.h>
#include <openthread/message.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/udp.h>

#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/logging.h>

#include "openthread-system.h"

#include "common/code_utils.hpp"

#if OPENTHREAD_EXAMPLES_POSIX
#include <setjmp.h>
#include <unistd.h>

jmp_buf gResetJump;

void __gcov_flush();
#endif

#define UDP_BLINKS 4

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
void *otPlatCAlloc(size_t aNum, size_t aSize)
{
    return calloc(aNum, aSize);
}

void otPlatFree(void *aPtr)
{
    free(aPtr);
}
#endif

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}

void handleGpioInterrupt1(otInstance *aInstance);

void setNetworkConfiguration(otInstance *aInstance);

void OTCALL handleNetifStateChanged(uint32_t aFlags, void *aContext);

void initUdp(otInstance *aInstance);
void closeUdp();
void sendUdp(otInstance *aInstance, const char *aDestAddr, const char *aUdpMessage);
void handleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo);

otUdpSocket mUdpSocket;
uint16_t mUdpPort = 1212;

uint16_t swap16(uint16_t v);

int main(int argc, char *argv[])
{
    otInstance *sInstance;

#if OPENTHREAD_EXAMPLES_POSIX
    if (setjmp(gResetJump))
    {
        alarm(0);
#if OPENTHREAD_ENABLE_COVERAGE
        __gcov_flush();
#endif
        execvp(argv[0], argv);
    }
#endif

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    size_t   otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer       = NULL;
#endif

pseudo_reset:

    otSysInit(argc, argv);

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);

    // Initialize OpenThread with the buffer
    sInstance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    sInstance = otInstanceInitSingle();
#endif
    assert(sInstance);

    otCliUartInit(sInstance);

#if OPENTHREAD_ENABLE_DIAG
    otDiagInit(sInstance);
#endif

    /* Register thread state change handler */
    otSetStateChangedCallback(sInstance, handleNetifStateChanged, sInstance);

    /* Register GPIO button handlers for first 2 buttons */
    otSysGpioRegisterCallback(BUTTON_GPIO_PORT, BUTTON_1_PIN, handleGpioInterrupt1, sInstance);
    otSysGpioIntEnable(BUTTON_GPIO_PORT, BUTTON_1_PIN);

    /* Override default values such as Pan ID so the boards all join the same network */
    setNetworkConfiguration(sInstance);

    /* start thread network interface (CLI cmd >ifconfig up) */
    otIp6SetEnabled(sInstance, true);

    /* start thread stack (CLI cmd >thread start) */
    otThreadSetEnabled(sInstance, true);

    initUdp(sInstance);

    while (!otSysPseudoResetWasRequested())
    {
        otTaskletsProcess(sInstance);
        otSysProcessDrivers(sInstance);
    }

    closeUdp();

    otInstanceFinalize(sInstance);
#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    free(otInstanceBuffer);
#endif

    goto pseudo_reset;

    return 0;
}

/**
 * Override default network settings, such as panid, so the devices can join a network
 */
void setNetworkConfiguration(otInstance *aInstance)
{
    /* Set the panid so the nodes join the same network (CLI cmd >panid 0x1234) */
    /* Using otOperationalDataset instead of:
     * otLinkSetPanId(sInstance, (otPanId)0x1234);
     *
     * Fields that can be configured in otOperationDataset to override defaults:
     *     Network Name, Mesh Local Prefix, Extended PAN ID, PAN ID, Delay Timer,
     *     Channel, Channel Mask Page 0, Network Master Key, PSKc, Security Policy
     */
    otOperationalDataset aDataset;

    static char aNetworkName[] = "OTCodelab";

    memset(&aDataset, 0, sizeof(otOperationalDataset));

    uint32_t aTimestamp                            = otPlatAlarmMilliGetNow();
    aDataset.mActiveTimestamp                      = aTimestamp;
    aDataset.mComponents.mIsActiveTimestampPresent = true;

    /* Set Channel to 15 */
    aDataset.mChannel                      = 15;
    aDataset.mComponents.mIsChannelPresent = true;

    /* Set Pan ID to 2222 */
    aDataset.mPanId                      = (otPanId)0x2222;
    aDataset.mComponents.mIsPanIdPresent = true;

    /* Set Extended Pan ID to C0DE1AB5C0DE1AB5 */
    uint8_t extPanId[OT_EXT_PAN_ID_SIZE] = {0xC0, 0xDE, 0x1A, 0xB5, 0xC0, 0xDE, 0x1A, 0xB5};
    memcpy(aDataset.mExtendedPanId.m8, extPanId, sizeof(aDataset.mExtendedPanId));
    aDataset.mComponents.mIsExtendedPanIdPresent = true;

    /* Set master key to 1234C0DE1AB51234C0DE1AB51234C0DE */
    uint8_t key[OT_MASTER_KEY_SIZE] = {0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5, 0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5};
    memcpy(aDataset.mMasterKey.m8, key, sizeof(aDataset.mMasterKey));
    aDataset.mComponents.mIsMasterKeyPresent = true;

    /* Set Network Name to OTCodelab */
    size_t length = strlen(aNetworkName);
    if (length <= OT_NETWORK_NAME_MAX_SIZE)
    {
        memset(&aDataset.mNetworkName, 0, sizeof(aDataset.mNetworkName));
        memcpy(aDataset.mNetworkName.m8, aNetworkName, length);
        aDataset.mComponents.mIsNetworkNamePresent = true;
    }

    otDatasetSetActive(aInstance, &aDataset);

    /* Set the router selection jitter to override the 2 minute default.
       CLI cmd >routerselectionjitter 20
       Warning: For demo purposes only - not to be used in a real product */
    uint8_t jitterValue = 10;
    otThreadSetRouterSelectionJitter(aInstance, jitterValue);
}

/**
 * Function to handle state changes in OpenThread device -
 * here it is only checking for role changes.
 */
void OTCALL handleNetifStateChanged(uint32_t aFlags, void *aContext)
{
    otDeviceRole changedRole;

    if ((aFlags & OT_CHANGED_THREAD_ROLE) != 0)
    {
        /* Clear all the role pins first then set the
           pin based on the changed role */
        otSysGpioOutClear(LED_GPIO_PORT, LED_1_PIN);
        otSysGpioOutClear(LED_GPIO_PORT, LED_2_PIN);
        otSysGpioOutClear(LED_GPIO_PORT, LED_3_PIN);

        changedRole = otThreadGetDeviceRole(aContext);
        switch (changedRole)
        {
        case OT_DEVICE_ROLE_LEADER:
            otSysGpioOutSet(LED_GPIO_PORT, LED_1_PIN);
            break;

        case OT_DEVICE_ROLE_ROUTER:
            otSysGpioOutSet(LED_GPIO_PORT, LED_2_PIN);
            break;

        case OT_DEVICE_ROLE_CHILD:
            otSysGpioOutSet(LED_GPIO_PORT, LED_3_PIN);
            break;

        case OT_DEVICE_ROLE_DETACHED:
        case OT_DEVICE_ROLE_DISABLED:
            /* Clear led 4 also if the board doesn't have thread started */
            otSysGpioOutClear(LED_GPIO_PORT, LED_4_PIN);
            break;
        }
    }
}

/**
 * Function to handle button 1 push event
 */
void handleGpioInterrupt1(otInstance *aInstance)
{
    static char nodeMulticastAddr[] = "ff03::1";
    static char udpMessage[]        = "Hello World!";

    /* Send multicast UDP -> the receiving boards should blink LED #4 */
    sendUdp(aInstance, nodeMulticastAddr, udpMessage);
}

/**
 * Initialize UDP socket
 */
void initUdp(otInstance *aInstance)
{
    static char anyUdpAddr[] = "::";

    otSockAddr  listenSockAddr;

    memset(&mUdpSocket, 0, sizeof(mUdpSocket));
    memset(&listenSockAddr, 0, sizeof(listenSockAddr));

    otIp6AddressFromString(anyUdpAddr, &listenSockAddr.mAddress);

    listenSockAddr.mPort = mUdpPort;

    listenSockAddr.mScopeId = OT_NETIF_INTERFACE_ID_THREAD;

    otUdpOpen(aInstance, &mUdpSocket, handleUdpReceive, aInstance);

    otUdpBind(&mUdpSocket, &listenSockAddr);
}

/**
 * Close UDP socket
 */
void closeUdp()
{
    otUdpClose(&mUdpSocket);
}

/**
 * Send a UDP datagram
 */
void sendUdp(otInstance *aInstance, const char *aDestAddr, const char *aUdpMessage)
{
    /* Send UDP datagram to the multicast address */
    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "Send UDP Datagram");

    otError       error = OT_ERROR_NONE;
    otMessage *   message;
    otMessageInfo messageInfo;
    otIp6Address  destinationAddr;

    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString(aDestAddr, &destinationAddr);
    messageInfo.mPeerAddr = destinationAddr;
    messageInfo.mPeerPort = mUdpPort;

    messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;

    message = otUdpNewMessage(aInstance, true);
    VerifyOrExit(message != NULL, error = OT_ERROR_NO_BUFS);

    SuccessOrExit(error = otMessageAppend(message, aUdpMessage, strlen(aUdpMessage)));

    error = otUdpSend(&mUdpSocket, message, &messageInfo);

exit:
    if (error != OT_ERROR_NONE && message != NULL)
    {
        otMessageFree(message);
    }
}

/**
 * Function to handle UDP datagrams received on the socket set up for listening
 */
void handleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    (void)aContext;

    /* Blink LED 4 a few times then turn it back off */
    otSysGpioOutBlink(LED_GPIO_PORT, LED_4_PIN, UDP_BLINKS);

    uint8_t buf[1500];
    int     length;

    length      = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);
    buf[length] = '\0';

    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "UDP message: %s", buf);
    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "%d bytes from %x:%x:%x:%x:%x:%x:%x:%x %d \r\n",
              otMessageGetLength(aMessage) - otMessageGetOffset(aMessage),
              swap16(aMessageInfo->mPeerAddr.mFields.m16[0]), swap16(aMessageInfo->mPeerAddr.mFields.m16[1]),
              swap16(aMessageInfo->mPeerAddr.mFields.m16[2]), swap16(aMessageInfo->mPeerAddr.mFields.m16[3]),
              swap16(aMessageInfo->mPeerAddr.mFields.m16[4]), swap16(aMessageInfo->mPeerAddr.mFields.m16[5]),
              swap16(aMessageInfo->mPeerAddr.mFields.m16[6]), swap16(aMessageInfo->mPeerAddr.mFields.m16[7]),
              aMessageInfo->mPeerPort);
}

/*
 * Provide, if required an "otPlatLog()" function
 */
#if OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_APP
void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);
    OT_UNUSED_VARIABLE(aFormat);

    va_list ap;
    va_start(ap, aFormat);
    otCliPlatLogv(aLogLevel, aLogRegion, aFormat, ap);
    va_end(ap);
}
#endif

uint16_t swap16(uint16_t v)
{
#if BYTE_ORDER_BIG_ENDIAN
    return v;
#else /* BYTE_ORDER_LITTLE_ENDIAN */
    return (((v & 0x00ffU) << 8) & 0xff00) | (((v & 0xff00U) >> 8) & 0x00ff);
#endif
}
