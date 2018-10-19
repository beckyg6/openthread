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
#include <openthread/icmp6.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
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

#define ICMP_BLINKS 2
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

void setNetworkConfiguration(otInstance *aInstance);

void OTCALL handleNetifStateChanged(uint32_t aFlags, void *aContext);

const char  nodeMulticastAddr[] = "ff03::1";
const char  anyUdpAddr[]        = "::";
const char  udpMessage[]        = "Hello UDP world";
uint16_t    udpPort             = 1212;
otSockAddr  listenSockAddr;
otUdpSocket mSocket;

void initUdp(otInstance *aInstance);
void closeUdp();
void sendUdp(otInstance *aInstance);
void handleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo);

void sendPing(otInstance *aInstance);
void handlePingReceive(void *               aContext,
                       otMessage *          aMessage,
                       const otMessageInfo *aMessageInfo,
                       const otIcmp6Header *aIcmpHeader);

void handleIp6DatagramReceive(otMessage *aMessage, void *aContext);

void handleGpioInterrupt1(otInstance *aInstance);
void handleGpioInterrupt2(otInstance *aInstance);

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

    otSysGpioRegisterCallback(BUTTON_GPIO_PORT, BUTTON_2_PIN, handleGpioInterrupt2, sInstance);
    otSysGpioIntEnable(BUTTON_GPIO_PORT, BUTTON_2_PIN);

    /* Disable ICMP Echo handling so we can process it from the ip6 handler.
       NOTE: this will bypass the functionality for the ping command on the CLI (no
       response is seen in the CLI) */
    otIcmp6SetEchoMode(sInstance, OT_ICMP6_ECHO_HANDLER_DISABLED);

    /* Register an IPv6 handler to receive ping messages from other boards */
    otIp6SetReceiveCallback(sInstance, handleIp6DatagramReceive, sInstance);

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

/*
 * Override default network settings
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

    const char aNetworkName[] = "OTCodelab";

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

/* Button 1 was pushed - send ICMP Ping */
void handleGpioInterrupt1(otInstance *aInstance)
{
    /* Send multicast ping -> the receiving boards should blink LED #4 */
    sendPing(aInstance);
}

/* Button 2 was pushed - send UDP datagram */
void handleGpioInterrupt2(otInstance *aInstance)
{
    /* Send multicast UDP -> the receiving boards should blink LED #4 */
    sendUdp(aInstance);
}

void initUdp(otInstance *aInstance)
{
    memset(&mSocket, 0, sizeof(mSocket));
    memset(&listenSockAddr, 0, sizeof(listenSockAddr));

    otIp6AddressFromString(anyUdpAddr, &listenSockAddr.mAddress);

    listenSockAddr.mPort = udpPort;

    listenSockAddr.mScopeId = OT_NETIF_INTERFACE_ID_THREAD;

    otUdpOpen(aInstance, &mSocket, handleUdpReceive, aInstance);

    otUdpBind(&mSocket, &listenSockAddr);
}

void closeUdp()
{
    otUdpClose(&mSocket);
}

uint16_t Swap16(uint16_t v)
{
#if BYTE_ORDER_BIG_ENDIAN
    return v;
#else /* BYTE_ORDER_LITTLE_ENDIAN */
    return (((v & 0x00ffU) << 8) & 0xff00) | (((v & 0xff00U) >> 8) & 0x00ff);
#endif
}

void sendUdp(otInstance *aInstance)
{
    (void)aInstance;

    /* Send UDP datagram to the multicast address */
    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "Send UDP Datagram");

    otError       error = OT_ERROR_NONE;
    otMessage *   message;
    otMessageInfo messageInfo;
    otIp6Address  pingDestinationAddr;

    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString(nodeMulticastAddr, &pingDestinationAddr);
    messageInfo.mPeerAddr = pingDestinationAddr;
    messageInfo.mPeerPort = udpPort;

    messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;

    message = otUdpNewMessage(aInstance, true);
    VerifyOrExit(message != NULL, error = OT_ERROR_NO_BUFS);

    SuccessOrExit(error = otMessageAppend(message, &udpMessage, strlen(udpMessage)));

    error = otUdpSend(&mSocket, message, &messageInfo);

exit:
    if (error != OT_ERROR_NONE && message != NULL)
    {
        otMessageFree(message);
    }
}

void handleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    (void)aContext;
    (void)aMessage;
    (void)aMessageInfo;

    /* Blink LED 4 a few times then turn it back off */
    otSysGpioOutBlink(LED_GPIO_PORT, LED_4_PIN, UDP_BLINKS);

    uint8_t buf[1500];
    int     length;

    length      = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);
    buf[length] = '\0';

    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "UDP message: %s", buf);
    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "%d bytes from %x:%x:%x:%x:%x:%x:%x:%x %d \r\n",
              otMessageGetLength(aMessage) - otMessageGetOffset(aMessage),
              Swap16(aMessageInfo->mPeerAddr.mFields.m16[0]), Swap16(aMessageInfo->mPeerAddr.mFields.m16[1]),
              Swap16(aMessageInfo->mPeerAddr.mFields.m16[2]), Swap16(aMessageInfo->mPeerAddr.mFields.m16[3]),
              Swap16(aMessageInfo->mPeerAddr.mFields.m16[4]), Swap16(aMessageInfo->mPeerAddr.mFields.m16[5]),
              Swap16(aMessageInfo->mPeerAddr.mFields.m16[6]), Swap16(aMessageInfo->mPeerAddr.mFields.m16[7]),
              aMessageInfo->mPeerPort);
}

void sendPing(otInstance *aInstance)
{
    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "Send Echo Request");

    otError error = OT_ERROR_NONE;

    uint32_t timestamp = otPlatAlarmMilliGetNow();

    otMessage *   message;
    otMessageInfo messageInfo;
    otIp6Address  pingDestinationAddr;

    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString(nodeMulticastAddr, &pingDestinationAddr);
    messageInfo.mPeerAddr    = pingDestinationAddr;
    messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;

    VerifyOrExit((message = otIp6NewMessage(aInstance, true)) != NULL, error = OT_ERROR_NO_BUFS);
    SuccessOrExit(error = otMessageAppend(message, &timestamp, sizeof(timestamp)));
    SuccessOrExit(error = otIcmp6SendEchoRequest(aInstance, message, &messageInfo, 1));

exit:
    if (error != OT_ERROR_NONE && message != NULL)
    {
        otMessageFree(message);
    }
}

void handlePingReceive(void *               aContext,
                       otMessage *          aMessage,
                       const otMessageInfo *aMessageInfo,
                       const otIcmp6Header *aIcmpHeader)
{
    (void)aContext;
    (void)aMessage;
    (void)aMessageInfo;
    (void)aIcmpHeader;

    otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_API, "Received Echo Request, blink LED 4");

    /* Blink LED 4 a few times then turn it back off */
    otSysGpioOutBlink(LED_GPIO_PORT, LED_4_PIN, ICMP_BLINKS);

    /* TODO: send acknowledgement reply */
}

enum IpProto /* copied from "net/ip6.hpp" */
{
    kProtoHopOpts  = 0,  ///< IPv6 Hop-by-Hop Option
    kProtoTcp      = 6,  ///< Transmission Control Protocol
    kProtoUdp      = 17, ///< User Datagram
    kProtoIp6      = 41, ///< IPv6 encapsulation
    kProtoRouting  = 43, ///< Routing Header for IPv6
    kProtoFragment = 44, ///< Fragment Header for IPv6
    kProtoIcmp6    = 58, ///< ICMP for IPv6
    kProtoNone     = 59, ///< No Next Header for IPv6
    kProtoDstOpts  = 60, ///< Destination Options for IPv6
};

enum /* copied from "net/ip6.hpp" */
{
    kVersionClassFlowSize = 4, /* Combined size of Version, Class, Flow Label in bytes */
};

/**
 * This structure represents an IPv6 header.
 *
 * copied from "net/ip6.hpp"
 */
OT_TOOL_PACKED_BEGIN
struct HeaderPoD
{
    union OT_TOOL_PACKED_FIELD
    {
        uint8_t  m8[kVersionClassFlowSize / sizeof(uint8_t)];
        uint16_t m16[kVersionClassFlowSize / sizeof(uint16_t)];
        uint32_t m32[kVersionClassFlowSize / sizeof(uint32_t)];
    } mVersionClassFlow;         ///< Version, Class, Flow Label
    uint16_t     mPayloadLength; ///< Payload Length
    uint8_t      mNextHeader;    ///< Next Header
    uint8_t      mHopLimit;      ///< Hop Limit
    otIp6Address mSource;        ///< Source
    otIp6Address mDestination;   ///< Destination
} OT_TOOL_PACKED_END HeaderPoD;

void handleIp6DatagramReceive(otMessage *aMessage, void *aContext)
{
    struct HeaderPoD ip6;

    otIcmp6Header icmpHeader;

    /* Read Ipv6 Header */
    otMessageRead(aMessage, 0, &ip6, sizeof(ip6));

    /* Copy source and destination address into messageInfo */
    otMessageInfo messageInfo;
    memset(&messageInfo, 0, sizeof(messageInfo));
    messageInfo.mPeerAddr = ip6.mSource;
    messageInfo.mSockAddr = ip6.mDestination;

    switch (ip6.mNextHeader)
    {
    case kProtoUdp:
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "Received UDP Datagram.");

        /* Don't call handleUdpReceive because it's receiving all the ports
         * here, let registered handler receive it (Ip6 code will call it).
        handleUdpReceive(aContext, aMessage, &messageInfo); */
        break;

    case kProtoIcmp6:
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "Received ICMP Message.");

        /* Read the ICMPv6 header and directly call ICMP handler since
         * ICMP Echo handling has been disabled in main():
         *   otIcmp6SetEchoMode(sInstance, OT_ICMP6_ECHO_HANDLER_DISABLED);
         */
        otMessageRead(aMessage, otMessageGetOffset(aMessage), &icmpHeader, sizeof(icmpHeader));
        if (OT_ICMP6_TYPE_ECHO_REQUEST == icmpHeader.mType)
        {
            handlePingReceive(aContext, aMessage, &messageInfo, &icmpHeader);
        }
        /* else if (OT_ICMP6_TYPE_ECHO_REPLY == aIcmpHeader.mType)
         * Not receiving an echo reply because ICMP Echo handling disabled
         */
        break;

    default:
        break;
    }
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
