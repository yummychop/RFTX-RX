/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/

/* Application header files */
#include "StateMachine.h"
#include "smartrf_settings/smartrf_settings.h"
#include "Board.h"
#include "StateMachine.h"
#include "rfSynchronizedPacket.h"

/* stdlib */
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

/* TiRTOS Kernel */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>
/* High-level Ti-Drivers */
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/rf/RF.h>

#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_data_entry.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_prop_cmd.h>
#include <driverlib/rf_prop_mailbox.h>


/* Define events that can be posted to the application state machine */
typedef enum {
    Event_SyncButtonPushed = StateMachine_Event00,
    Event_PacketReceived = StateMachine_Event01,
    Event_SyncMissed = StateMachine_Event02
} Event;

/* Declare state handler functions for the application. */
StateMachine_DECLARE_STATE(SetupState);
StateMachine_DECLARE_STATE(WaitingForSyncState);
StateMachine_DECLARE_STATE(SyncedRxState);


/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 *   - Button pins are high by default and pulled to low when pressed.
 */
PIN_Config pinTable[] =
{
    Board_LED2 | Board_LED1|PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

PIN_Config buttonPinTable[] =
{
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};
/***** Defines *****/
#define RX_TASK_STACK_SIZE 3800
#define RX_TASK_PRIORITY   1

/* Packet RX Configuration */
#define NUM_APPENDED_BYTES     5  /* The Data Entries data field will contain:
                                   * - 1 address byte in the header (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * - up to sizeof BeaconPacket bytes payload
                                   * - 4 bytes for the RAT time stamp (RF_cmdPropRx.rxConf.bAppendTimestamp = 0x1) */

#define SYMBOL_RATE            50000                    /* 50 kbits per second */
#define US_PER_SYMBOL          (1000000 / SYMBOL_RATE)
#define PREAMBLE_BITS          32
#define SYNCWORD_BITS          32

#define RX_START_MARGIN        RF_convertUsToRatTicks(566)   /* An arbitrarily chosen value to compensate for
                                                              * the potential drift of the RAT and the RTC. */

#define RX_TIMEOUT_TICKS       RF_convertUsToRatTicks((PREAMBLE_BITS + SYNCWORD_BITS) * US_PER_SYMBOL)
                                                             /* Tight, but ideal duration for receiving all bits of
                                                              * the preamble and the sync word. */

#define RX_TIMEOUT_MARGIN      RF_convertUsToRatTicks(1000)  /* Arbitrarily chosen margin added to the RX timeout
                                                              * to compensate calculation errors. */

#define RX_START_TO_SETTLE_TICKS   256 /* Time between RX start trigger and the radio
                                        * being ready to receive the first preamble bit.
                                        * This is a fixed value for CMD_PROP_RX. */
#define TX_START_TO_PREAMBLE_TICKS 384 /* Time between TX start trigger and first bit on air.
                                        * This is a fixed value for CMD_PROP_TX. */
#define BEACON_INTERVAL_MS  150
/***** Prototypes *****/
static void rxTaskFunction(UArg arg0, UArg arg1);
void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId);
void WaitingForSyncState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
void SyncedRxState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RX_TASK_STACK_SIZE];

static StateMachine_Struct stateMachine;

static PIN_Handle pinHandle;
static PIN_State pinState;
static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Queue object that the RF Core will fill with data */
static dataQueue_t rxQueue;

/* A single queue item that points to a data buffer  */
static rfc_dataEntryPointer_t rxItem;

/* Word-aligned buffer for the packet payload + meta data. */
static uint8_t rxBuffer[((sizeof(BeaconPacket) + NUM_APPENDED_BYTES) + 8)];

BeaconPacket beacon;
uint8_t RFrevCount;
int timeStart,timeStop,timeInterval;
/*
 * Form PDMStream
 *
 */
/* XDCtools Header files */
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* TI-RTOS Header files */
#include <ti/mw/display/Display.h>

/* PDM Driver */
#include <ti/drivers/pdm/PDMCC26XX.h>

/* Example/Board Header files */
#include <stdint.h>
#include <math.h>

Semaphore_Struct semStruct;
Semaphore_Handle saSem;

/* store the events for this application */
static uint16_t SA_events = 0x0001;
#define SA_DEBOUNCE_COUNT_IN_MS  25

#define SA_PCM_START             0x0001
#define SA_PCM_BLOCK_READY       0x0002
#define SA_PCM_WAITING           0x0003
#define SA_PCM_ERROR             0x0004
#define SA_PCM_STOP              0x0008
#define SA_PCM_DSP               0x0010
#define sampleNumber             640
//uint8_t SA_PCM_DSP=false;
static bool processingButtonPress = false;
//static int totalFrameCount = 0;
//static int currentFrameCount = 0;
//static int sessionCount = 0;
int callbackCount,callbackCountSend;
int16_t PCMsamples[sampleNumber];
//int yfil[384];
//int yprefil[384];
int position,energy;
/* Audio Protocol Define */
#define SA_PCM_BLOCK_SIZE_IN_SAMPLES       32
#define SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS  20

#define AUDIO_BUF_COMPRESSED_SIZE   ((SA_PCM_BLOCK_SIZE_IN_SAMPLES*\
                                      SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS*\
                                     sizeof(uint16_t))/ PCM_COMPRESSION_RATE)
#define AUDIO_BUF_UNCOMPRESSED_SIZE (SA_PCM_BLOCK_SIZE_IN_SAMPLES*\
                                     SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS*\
                                     sizeof(uint16_t))
#define numofPCMSamples 1920
#define maxdelay 250
/* SA PDM drivers related */
static void SA_PDMCC26XX_callbackFxn(PDMCC26XX_Handle handle,
        PDMCC26XX_StreamNotification *streamNotification);
static PDMCC26XX_Handle pdmHandle = NULL;
static void SA_processPDMData();
static void SA_audioDataCollection(int16_t *pPCMsamples, uint16_t numOfSamples);
//static float SA_audioEnvelopDetection(int16_t *pPCMsamples, uint16_t numOfSamples);
//static void SA_audioDSP(int16_t Watch1[], float y[]);
//static int16_t SA_audioCC(int16_t x[],int16_t y[],int16_t n);
static void SA_audioIIR(int16_t x[],int yfil[]);
static void *SA_audioMalloc(uint_least16_t size);
static void SA_audioFree(void *msg, size_t size);
static void SA_audioTask();
PDMCC26XX_Params pdmParams = {
        .callbackFxn = SA_PDMCC26XX_callbackFxn,
        .useDefaultFilter = true,
        .decimationFilter = NULL,
        .micGain = PDMCC26XX_GAIN_18,
        .micPowerActiveHigh = true,
        .applyCompression = false,
        .startupDelayWithClockInSamples = 512,
        .retBufSizeInBytes = AUDIO_BUF_UNCOMPRESSED_SIZE + PCM_METADATA_SIZE,
        .mallocFxn = (PDMCC26XX_MallocFxn) SA_audioMalloc,
        .freeFxn = (PDMCC26XX_FreeFxn) SA_audioFree,
        .custom = NULL };

/* SA audio streaming States */
typedef enum
{
  SA_AUDIO_IDLE,
  SA_AUDIO_STARTING,
  SA_AUDIO_STREAMING,
  SA_AUDIO_STOPPING,
  SA_AUDIO_ERROR
} saAudioState_t;
static saAudioState_t saAudioState = SA_AUDIO_IDLE;

void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);




/***** Includes *****/
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

#include <stdlib.h>
#include <driverlib/trng.h>
#include <driverlib/aon_batmon.h>
#include <DmNodeRadioTask.h>

#include "RadioProtocol.h"
#include "seb/SEB.h"


/***** Defines *****/
#define NODERADIO_TASK_STACK_SIZE 1024
#define NODERADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                 0xFFFFFFFF
#define RADIO_EVENT_SEND_ADC_DATA       (uint32_t)(1 << 0)
#define RADIO_EVENT_DATA_ACK_RECEIVED   (uint32_t)(1 << 1)
#define RADIO_EVENT_ACK_TIMEOUT         (uint32_t)(1 << 2)
#define RADIO_EVENT_SEND_FAIL           (uint32_t)(1 << 3)

#define NODERADIO_MAX_RETRIES 2
#define NORERADIO_ACK_TIMEOUT_TIME_MS (160)

#define NODE_0M_TXPOWER    -10

/***** Type declarations *****/
struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum NodeRadioOperationStatus result;
};


/***** Variable declarations *****/
//static Task_Params nodeRadioTaskParams;
Task_Struct nodeRadioTask;        /* not static so you can see in ROV */
//static uint8_t nodeRadioTaskStack[NODERADIO_TASK_STACK_SIZE];
Semaphore_Struct radioAccessSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioAccessSemHandle;
Event_Struct radioOperationEvent; /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;
Semaphore_Struct radioResultSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioResultSemHandle;
static struct RadioOperation currentRadioOperation;
static uint16_t adcData;
//static uint8_t nodeAddress = 0;
static struct DualModeSensorPacket dmSensorPacket;
static Node_AdertiserType advertiserType = Node_AdertiserMs;

/* The Eddystone UID spec advices to use the first 10 bytes of the sha-1 hash
 * of an owned domain or subdonmian. The subdomain of
 * http://www.ti.com/product/CC1350 = 792f082074ebc132032cad5fdaada66154e14e98 */
static uint8_t uidNameSpace[10] = {0x79, 0x2f, 0x08, 0x20, 0x74, 0xeb, 0xc1,
                                   0x32, 0x03, 0x2c};

/* previous Tick count used to calculate uptime for the TLM beacon */
//static uint32_t prevTicks;

/* uid Instance should be set to 6 LSB's of IEEE addr */
static uint8_t uidInstanceId[6];

/* propreitory advertisement packet */
/*
static uint8_t localNameAdvertisement[] = {
        0x02, //Length of this Data section
        0x01, //<<Flags>>
        0x02, //LE General Discoverable Mode
        0x18, //Length of this Data section
        0x09, //<<Complete local name>>
        'C', 'C', '1', '3', '5', '0', ' ',
        'L', 'a', 'u', 'n', 'c', 'h',
        'p', 'a', 'd', ' ', '-', ' ', '0', 'x', '0', '0'
        };
*/
static uint8_t localNameAdvertisement[] = {
        0x02, //Length of this Data section
        0x01, //<<Flags>>
        0x02, //LE General Discoverable Mode
        0x08, //Length of this Data section
        0x09, //<<Complete local name>>
       '1',   // audio receiver No.
	   '0',   // pos
	   '0',   // pos
	   '0',   // pos
	   '0',   // callbackCountSend
	   '0',   // callbackCountSend
	   '0'    // callbackCountSend
};
/* propreitory advertisement packet */
static uint8_t propAdvertisement[] = {
        0x02, //Length of this section
        0x01, //<<Flags>>
        0x02, //LE General Discoverable Mode
        0x06, //Length of this section
        0xff, //<<Manufacturer Specific Data>>
        0x0d,
        0x00,
        0x03,
        0x00,
        0x00}; //BTN state

SimpleBeacon_Frame propAdvFrame;
SimpleBeacon_Frame localNameAdvFrame;

static uint8_t bleMacAddr[6];

/* Pin driver handle */
extern PIN_Handle ledPinHandle;

/***** Prototypes *****/
//static void nodeRadioTaskFunction(UArg arg0, UArg arg1);
static void sendBleAdvertisement(struct DualModeSensorPacket sensorPacket);
static void BLE_Task();
uint8_t BLE_Task_Done=false;

/*
 * From WoR
 *
 */
/***** Includes *****/
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Drivers */
//#include <ti/drivers/rf/RF.h>
//#include <ti/drivers/PIN.h>
//#include <ti/drivers/pin/PINCC26XX.h>

/* Board Header files */
#include "Board.h"

/* RF settings, defines and helper files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include "driverlib/rf_prop_mailbox.h"


/***** Defines *****/
/* Wake-on-Radio wakeups per second */
#define WOR_WAKEUPS_PER_SECOND  2

/* Wake-on-Radio mode. Can be:
 * - RSSI only
 * - PQT, preamble detection
 * - Both, first RSSI and then PQT if RSSI  */
#define WOR_MODE CarrierSenseMode_RSSIandPQT

/* Threshold for RSSI based Carrier Sense in dBm */
#define WOR_RSSI_THRESHOLD      ((int8_t)(-111))

/* Macro used to set actual wakeup interval */
#define WOR_WAKE_UP_MARGIN_S 0.005f
#define WOR_WAKE_UP_INTERVAL_RAT_TICKS(x) \
    ((uint32_t)(4000000*(1.0f/(x) - (WOR_WAKE_UP_MARGIN_S))))

/* TI-RTOS Task configuration */
//#define RX_TASK_STACK_SIZE 1024
//#define RX_TASK_PRIORITY   2

/* TX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             31 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     1  /* Length byte included in the stored packet */

/***** Type declarations *****/
/* General wake-on-radio RX statistics */
struct WorStatistics {
  uint32_t doneIdle;
  uint32_t doneIdleTimeout;
  uint32_t doneRxTimeout;
  uint32_t doneOk;
};

/* Modes of carrier sense possible */
enum CarrierSenseMode {
    CarrierSenseMode_RSSI,
    CarrierSenseMode_PQT,
    CarrierSenseMode_RSSIandPQT,
};


/***** Variable declarations *****/
/* TX task objects and task stack */
//static Task_Params rxTaskParams;
//static Task_Struct rxTask;
//static uint8_t rxTaskStack[RX_TASK_STACK_SIZE];

/* RF driver object and handle */
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver object and handle */
//static PIN_Handle ledPinHandle;
//static PIN_State ledPinState;

/* General wake-on-radio sniff status statistics and statistics from the RF Core about received packets */
static volatile struct WorStatistics worStatistics;
static rfc_propRxOutput_t rxStatistics;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
/*
static PIN_Config pinTable[] =
{
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
*/
/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
    #pragma data_alignment = 4
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
        static uint8_t rxDataEntryBuffer [RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
            MAX_LENGTH, NUM_APPENDED_BYTES)] __attribute__ ((aligned (4)));
#else
    #error This compiler is not supported.
#endif

/* RX Data Queue and Data Entry pointer to read out received packets */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;

/* Received packet's length and pointer to the payload */
static uint8_t packetLength;
static uint8_t* packetDataPointer;

/* Sniff command for doing combined Carrier Sense and RX*/
static rfc_CMD_PROP_RX_SNIFF_t RF_cmdPropRxSniff;


/***** Prototypes *****/
static void rxTaskFunction(UArg arg0, UArg arg1);
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void initializeSniffCmdFromRxCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, rfc_CMD_PROP_RX_t* rxCmd);
static void configureSniffCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, enum CarrierSenseMode mode, uint32_t datarate, uint8_t wakeupPerSecond);
static uint32_t calculateSymbolRate(uint8_t prescaler, uint32_t rateWord);


/***** Function definitions *****/
void RxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RX_TASK_STACK_SIZE;
    rxTaskParams.priority = RX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rxTaskFunction, &rxTaskParams, NULL);
}

static void rxTaskFunction(UArg arg0, UArg arg1)
{

	StateMachine_exec(&stateMachine, SetupState);

}


int main(void)
{
	Semaphore_Params semParams;
    /* Call driver init functions. */
    Board_initGeneral();

    /* Initialize PDM driver (invokes I2S) */
    PDMCC26XX_init((PDMCC26XX_Handle) &(PDMCC26XX_config));
    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);
    assert(pinHandle != NULL);
    //buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    //assert(buttonPinHandle != NULL);
    //assert(PIN_registerIntCb(buttonPinHandle, &buttonCallbackFunction) == PIN_SUCCESS);
    //PIN_setOutputValue(pinHandle, Board_LED2, 1);
    // Initialise the application state machine.
    StateMachine_construct(&stateMachine);

    RxTask_init(pinHandle);

    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
     Semaphore_Params_init(&semParams);
     Semaphore_construct(&semStruct, 1, &semParams);

     /* Obtain instance handle */
     saSem = Semaphore_handle(&semStruct);
     /* This example has logging and many other debug capabilities enabled */
     System_flush();

     /* SysMin will only print to the console when you call flush or exit */
     System_flush();
    /* Start BIOS */
    BIOS_start();

    return (0);
}

/* Pin interrupt Callback function board buttons configured in the pinTable. */
void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId) {

    /* Debounce the button with a short delay */
    CPUdelay(CPU_convertMsToDelayCycles(10));
    if (PIN_getInputValue(pinId) == 1)
    {
        return;
    }

    switch (pinId)
    {
    case Board_BUTTON0:
        StateMachine_postEvents(&stateMachine, Event_SyncButtonPushed);
        break;
    }
}

void SetupState_function()
{
	static uint8_t RF_access;
	RF_Params rfParams;
	RF_Params_init(&rfParams);
	/* Route out LNA active pin to LED1 */
	//PINCC26XX_setMux(ledPinHandle, Board_LED0, PINCC26XX_MUX_RFC_GPO0);
	if(RF_access==0)
	{
	    /* Create queue and data entries */
	    if (RFQueue_defineQueue(&dataQueue,
	                            rxDataEntryBuffer,
	                            sizeof(rxDataEntryBuffer),
	                            NUM_DATA_ENTRIES,
	                            MAX_LENGTH + NUM_APPENDED_BYTES))
	    {
	        /* Failed to allocate space for all data entries */
	        while(1);
	    }

	    /* Copy all RX options from the SmartRF Studio exported RX command to the RX Sniff command */
	        initializeSniffCmdFromRxCmd(&RF_cmdPropRxSniff, &RF_cmdPropRx);
	        /* Configure RX part of RX_SNIFF command */
	        RF_cmdPropRxSniff.pQueue    = &dataQueue;
	        RF_cmdPropRxSniff.pOutput   = (uint8_t*)&rxStatistics;
	        RF_cmdPropRxSniff.maxPktLen = MAX_LENGTH;

	        /* Discard ignored packets and CRC errors from Rx queue */
	        RF_cmdPropRxSniff.rxConf.bAutoFlushIgnored = 1;
	        RF_cmdPropRxSniff.rxConf.bAutoFlushCrcErr  = 1;
	        /* Calculate datarate from prescaler and rate word */
	        uint32_t datarate = calculateSymbolRate(RF_cmdPropRadioDivSetup.symbolRate.preScale,
	                                              RF_cmdPropRadioDivSetup.symbolRate.rateWord);

	        /* Configure Sniff-mode part of the RX_SNIFF command */
	        configureSniffCmd(&RF_cmdPropRxSniff, WOR_MODE, datarate, WOR_WAKEUPS_PER_SECOND);
	        /* Request access to the radio */
	        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

	        /* Set frequency */
	        RF_EventMask result=RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, &callback, 0);
    }
            /*
            Semaphore_Params semParam;
            Semaphore_Params_init(&semParam);
            Semaphore_construct(&radioAccessSem, 1, &semParam);
            radioAccessSemHandle = Semaphore_handle(&radioAccessSem);


            Semaphore_construct(&radioResultSem, 0, &semParam);
            radioResultSemHandle = Semaphore_handle(&radioResultSem);


            Event_Params eventParam;
            Event_Params_init(&eventParam);
            Event_construct(&radioOperationEvent, &eventParam);
            radioOperationEventHandle = Event_handle(&radioOperationEvent);

            SimpleBeacon_init(true);
*/
               /* initialise the Simple Eddystone Beacon module
                * Set multiclient mode to true
                */
	           if(RF_access==0)
	           {
               SEB_init(true);

               //SEB_initUrl("https://www.ti.com/product/cc1350", NODE_0M_TXPOWER);

               SimpleBeacon_getIeeeAddr(bleMacAddr);

               propAdvFrame.deviceAddress = bleMacAddr;
               propAdvFrame.length = sizeof(propAdvertisement);
               propAdvFrame.pAdvData = propAdvertisement;

               localNameAdvFrame.deviceAddress = bleMacAddr;
               localNameAdvFrame.length = sizeof(localNameAdvertisement);
               localNameAdvFrame.pAdvData = localNameAdvertisement;
   	           RF_access=1;
	           }
               /* Save the current radio time */
               RF_cmdPropRxSniff.startTime = RF_getCurrentTime();
               while(1){
            	   /* Set next wakeup time in the future */
            	   RF_cmdPropRxSniff.startTime += WOR_WAKE_UP_INTERVAL_RAT_TICKS(WOR_WAKEUPS_PER_SECOND);
            	   sendBleAdvertisement(dmSensorPacket);
               	   	  /* Schedule RX */
               	   	  RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRxSniff, RF_PriorityNormal, &callback, RF_EventRxEntryDone);
                  	   	/* Log RX_SNIFF status */
               	   	    uint8_t worDone=0;
               	   	          switch(RF_cmdPropRxSniff.status) {
               	   	              //case PROP_DONE_IDLE:
               	   	                  /* Idle based on RSSI */
               	   	                  //worStatistics.doneIdle++;
               	   	                  //break;
               	   	              //case PROP_DONE_IDLETIMEOUT:
               	   	                  /* Idle based on PQT */
               	   	                  //worStatistics.doneIdleTimeout++;
               	   	                  //break;
               	   	              //case PROP_DONE_RXTIMEOUT:
               	   	                  /* Got valid preamble on the air, but did not find sync word */
               	   	                  //worStatistics.doneRxTimeout++;
               	   	                  //break;
               	   	              case PROP_DONE_OK:
               	   	                  /* Received packet */
               	   	                  //worStatistics.doneOk++;
               	   	                  worDone=1;
               	   	                  break;
               	   	              default:
               	   	                  /* Unhandled status */
               	   	                  break;
               	   	          };
               	   	          if(worDone==1) break;
               }


               /* Construct a circular RX queue with a single pointer-entry item. */
               rxItem.config.type = DATA_ENTRY_TYPE_PTR;
               rxItem.config.lenSz = 0;
               rxItem.length = sizeof(rxBuffer);
               rxItem.pNextEntry = (uint8_t*)&rxItem;
               rxItem.pData = (uint8_t*)&rxBuffer[0];
               rxItem.status = DATA_ENTRY_PENDING;
               rxQueue.pCurrEntry = (uint8_t*)&rxItem;
               rxQueue.pLastEntry = NULL;

               /* Modify CMD_PROP_RX command for application needs */
               RF_cmdPropRx.pQueue = &rxQueue;                /* Set the Data Entity queue for received data */
               RF_cmdPropRx.rxConf.bAutoFlushIgnored = true;  /* Discard ignored packets from Rx queue */
               RF_cmdPropRx.rxConf.bAutoFlushCrcErr = true;   /* Discard packets with CRC error from Rx queue */
               RF_cmdPropRx.rxConf.bIncludeHdr = true;        /* Put length field in front of queue entries. */
               RF_cmdPropRx.rxConf.bAppendTimestamp = true;   /* Append RX time stamp to the packet payload */
               RF_cmdPropRx.maxPktLen = sizeof(BeaconPacket); /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
               RF_cmdPropRx.pktConf.bRepeatOk = false;        /* Stop after receiving a single valid packet */
               RF_cmdPropRx.pktConf.bRepeatNok = true;

               /* Request access to the radio. This does not power-up the RF core, but only initialise
                * the driver and cache the setup command. */
               //RF_Params rfParams;
               //RF_Params_init(&rfParams);
               //rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
               //assert(rfHandle != NULL);

               /* Set the frequency. Now the RF driver powers the RF core up and runs the setup command from above.
                * The FS command is executed and also cached for later use when the RF driver does an automatic
                * power up. */
               //RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
               //assert((result == RF_EventLastCmdDone) && ((volatile RF_Op*)&RF_cmdFs)->status == DONE_OK);

               /* Route the LNA signal to an LED to indicate that the RF core is
                * active and receiving data.
                * Available signals are listed in the proprietary RF user's guide.
                */
               PINCC26XX_setMux(pinHandle, Board_LED2, PINCC26XX_MUX_RFC_GPO0);

    StateMachine_setNextState(&stateMachine, WaitingForSyncState);

}

void WaitingForSyncState_function()
{
    rxItem.status = DATA_ENTRY_PENDING;

    /* Start RX command to receive a single packet. */
    RF_cmdPropRx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropRx.endTrigger.triggerType = TRIG_NEVER;
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &WaitingForSyncState_rxCallback, RF_EventRxEntryDone);

    for (;;)
    {
        StateMachine_EventMask events = StateMachine_pendEvents(&stateMachine, Event_PacketReceived, BIOS_WAIT_FOREVER);

        if (events & Event_PacketReceived)
        {
            /* RX command has already stopped. Now examine the received data. */
            //uint8_t length;
            uint32_t rxTime;
            /*
            memcpy(&length, rxBuffer, 1);
            if (length != sizeof(BeaconPacket))
            {
                // This packet is not for us. Wait for next one
                StateMachine_setNextState(&stateMachine, WaitingForSyncState);
                break;
            }
            */
            /*
            memcpy(&beacon, (uint8_t*)rxBuffer + 1, sizeof(BeaconPacket));
                        if (beacon.addr!=1)
                        {
                            // This packet is not for us. Wait for next one
                            StateMachine_setNextState(&stateMachine, WaitingForSyncState);
                            break;
                        }
            */
            //memcpy(&beacon, (uint8_t*)rxBuffer + 1, sizeof(BeaconPacket));
            //memcpy(&rxTime, (uint8_t*)rxBuffer + 1 + sizeof(BeaconPacket), 4);

            //PIN_setOutputValue(pinHandle, Board_LED2, beacon.ledState);

            /* rxTime contains a calculated time stamp when the first preamble byte
             * was sent on air. As a time base for the next wake ups, we calculate
             * the time when this RX command would have been started for a synchronised
             * wake up. */

            //RF_cmdPropRx.startTime = rxTime - RX_START_TO_SETTLE_TICKS - RX_START_MARGIN;

            if(RFrevCount==1)
            {
               RFrevCount=0;
               SA_audioTask();
               BLE_Task();
               StateMachine_setNextState(&stateMachine, SetupState);
            }
            else
            StateMachine_setNextState(&stateMachine, WaitingForSyncState);
            break;
        }
    }
}

void WaitingForSyncState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventLastCmdDone)
    {
        if ((RF_cmdPropRx.status == PROP_DONE_OK) || (RF_cmdPropRx.status == PROP_DONE_ENDED))
        {
            /* Sync word has been found before end trigger and packet has been received */
            StateMachine_postEvents(&stateMachine, Event_PacketReceived);
            RFrevCount++;
        }
        else
        {
            // Everything else is an error.
            assert(false);
        }
    }
}

void SyncedRxState_function()
{
    rxItem.status = DATA_ENTRY_PENDING;

    /* Start RX command to receive a single packet. Use an absolute start trigger
     * and a predicted start time. The end time is calculated to be as tight as possible.
     * The compensation margin for RAT drift in both directions needs to be taken into account.  */
    RF_cmdPropRx.startTrigger.triggerType = TRIG_ABSTIME;
    //RF_cmdPropRx.startTime += beacon.beaconInterval;
    RF_cmdPropRx.startTime += RF_convertMsToRatTicks(BEACON_INTERVAL_MS);
    RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_START;
    RF_cmdPropRx.endTime = RX_START_TO_SETTLE_TICKS + RX_TIMEOUT_TICKS + RX_TIMEOUT_MARGIN + RX_START_MARGIN;

    /* Puts the RX command into the driver queue. Since the start trigger is absolute and has
     * a time somewhere in the future, the RF driver will power down the RF core and wait short
     * before the RX command is due. Then it will run the power-up sequence and dispatch the RX
     * command right on time.
     */
    //timeStart=Timestamp_get32();
    RF_CmdHandle cmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &SyncedRxState_rxCallback, RF_EventRxEntryDone);

    for (;;)
    {
        StateMachine_EventMask events = StateMachine_pendEvents(&stateMachine, Event_PacketReceived | Event_SyncButtonPushed | Event_SyncMissed, BIOS_WAIT_FOREVER);

        if (events & Event_PacketReceived)
        {
            /* RX command has already stopped. Now examine the received data. */
            //uint8_t length;
            uint32_t rxTime;
            /*
            memcpy(&length, rxBuffer, 1);
            if (length != sizeof(BeaconPacket))
            {
                // This packet is not for us. Wait for next one
                StateMachine_setNextState(&stateMachine, WaitingForSyncState);
                break;
            }
            */
            /*
            memcpy(&beacon, (uint8_t*)rxBuffer + 1, sizeof(BeaconPacket));
                                    if (beacon.addr!=1)
                                    {
                                        // This packet is not for us. Wait for next one
                                        StateMachine_setNextState(&stateMachine, WaitingForSyncState);
                                        break;
                                    }
            */
            //memcpy(&beacon, (uint8_t*)rxBuffer + 1, sizeof(BeaconPacket));
            memcpy(&rxTime, (uint8_t*)rxBuffer + 1 + sizeof(BeaconPacket), 4);

            /* The synchronisation offset might be used to calculate the clock drift between tranStateMachine_itter and receiver. */
            int32_t syncOffsetTime = rxTime - RX_START_TO_SETTLE_TICKS - RF_cmdPropRx.startTime;
            (void)syncOffsetTime; // need to reference the variable to prevent from a compiler warning

            /* Do an immediate re-synchronisation based on the new RX time. */
            //RF_cmdPropRx.startTime = rxTime - RX_START_TO_SETTLE_TICKS - RX_START_MARGIN;

            /* Display the current LED state of the TX board. */
            //PIN_setOutputValue(pinHandle, Board_LED2, beacon.ledState);
            if(RFrevCount==2)
            {
            //timeStop=Timestamp_get32();
            //timeInterval=timeStop-timeStart;
            RFrevCount=0;
            SA_audioTask();
            Task_sleep(30000);
            BLE_Task();
            StateMachine_setNextState(&stateMachine, SetupState);
            }
            else
            /* Wait for the next packet */
            StateMachine_setNextState(&stateMachine, SyncedRxState);
            break;
        }

        if (events & Event_SyncMissed)
        {
            /* Sync is missed. That means either we are out of sync or the
             * tranStateMachine_itter is in spontaneous mode. Try to receive the next packet. */
            StateMachine_setNextState(&stateMachine, SyncedRxState);
            break;
        }

        if (events & Event_SyncButtonPushed)
        {
            /* Force re-synchronisation */
            RF_cancelCmd(rfHandle, cmd, 0);
            RF_pendCmd(rfHandle, cmd, RF_EventCmdCancelled);
            StateMachine_setNextState(&stateMachine, WaitingForSyncState);
            RFrevCount=0;
            break;
        }
    }
}

void SyncedRxState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventLastCmdDone)
    {
        if ((RF_cmdPropRx.status == PROP_DONE_OK) || (RF_cmdPropRx.status == PROP_DONE_ENDED))
        {
            /* Sync word has been found before end trigger and packet has been received */
            StateMachine_postEvents(&stateMachine, Event_PacketReceived);
            RFrevCount++;

        }
        else if (RF_cmdPropRx.status == PROP_DONE_RXTIMEOUT)
        {
            StateMachine_postEvents(&stateMachine, Event_SyncMissed);
        }
    }
}

static void SA_audioTask(){
	while(1){
	                if (SA_events == SA_PCM_START) {
	                    if (pdmHandle == NULL)
	                    {
	                        /* Open PDM driver */
	                        pdmHandle = PDMCC26XX_open(&pdmParams);
	                    }
	                    if (pdmHandle && (saAudioState == SA_AUDIO_IDLE)) {
	                        saAudioState = SA_AUDIO_STARTING;
	                        /* Stream immediately if we simply dump over UART. */
	                        PDMCC26XX_startStream(pdmHandle);
	                        //timeStart=Timestamp_get32();

	                        saAudioState = SA_AUDIO_STREAMING;

	                    }
	                    SA_events=SA_PCM_WAITING;
	                }

	                if (SA_events == SA_PCM_STOP)
	               	                     {
	                	                     //int8_t n;
	               	                         if ((saAudioState != SA_AUDIO_IDLE) && (pdmHandle))
	               	                         {
	               	                             saAudioState = SA_AUDIO_STOPPING;
	               	                             /* Stop PDM stream */
	               	                             PDMCC26XX_stopStream(pdmHandle);
	               	                             /* In case no more callbacks will be made, attempt to flush remaining data now. */
	               	                             SA_processPDMData();
	               	                             }
	               	                         SA_events=SA_PCM_START;
	               	                         callbackCount=0;
	               	                         /*
	               	                         energy=0;// echo elimination
	               	                      	 for(n=0;n<29;n++){
	               	                      	     energy+=abs(yfil[position+n]);
	               	                      	 }
	               	                      	 energy/=29;
	               	                      	 */
	               	                         break;
	               	                     }
	                if(SA_events==SA_PCM_DSP){
                        int16_t i;
                        //uint16_t subResult1[9],subResult2[9],subResult3[9],subResult4[9],subResult5[9],subResult6[9];
                        //int yfil[sampleNumber]={0};
	                	//SA_audioIIR(PCMsamples,yfil);//IIR
	                	//for(i=0;i<(sampleNumber);i++)
	                	for(i=0;i<(sampleNumber);i++)
	                	{
	                		if(abs(PCMsamples[i])>100){
	                			   position=i;
	                			   callbackCountSend=callbackCount;
	                    		   SA_events=SA_PCM_STOP;
	                    		   break;
	                		}
/*
	                	    if(abs(yfil[i])>10&&abs(yfil[i])<20)
	                       {
	                    	   int16_t n, k,judgeCount1=0,judgeCount2=0,judgeCount3=0;
	                    		   for(n=1;n<15;n++){
	                    			   k=i+3*n;
	                    			   if(abs(yfil[k]-yfil[k+2])<abs(yfil[k+3]-yfil[k+5])) judgeCount1++;
	                    			   if(abs(yfil[k]-yfil[k+1])<abs(yfil[k+3]-yfil[k+4])) judgeCount2++;
	                    			   if(abs(yfil[k+1]-yfil[k+2])<abs(yfil[k+4]-yfil[k+5])) judgeCount3++;
	                    		   }
	                    		   if(judgeCount1>=13&&judgeCount2>=13||judgeCount1>=13&&judgeCount3>=13||judgeCount2>=13&&judgeCount3>=13)
	                    		   {
	                    		   if(i<47)
	                    			   position=0;
	                    		   else{
 	                    			   if(abs(yfil[i-1])>20||abs(yfil[i-2])>20||abs(yfil[i-3])>20)
	                    			      position=0xfff;
	                    		       else{
	                    			      position=i;
	                    			      callbackCountSend=callbackCount;
	                    		      }
	                    		   }
	                    		   SA_events=SA_PCM_STOP;
	                    		   break;
	                    	   }

	                       }
*/
	                	}
						if(SA_events!=SA_PCM_STOP)
	                	SA_events=SA_PCM_WAITING;

	                	                }

              }
}

/**
 *  @fn          SA_processPDMData
 *
 * @brief        Processed the received audio packetd from PDM driver and packed
 *               to send over RF4CE link.
 *
 * @param[in]    None.
 *
 * @param[out]   None.
 *
 * @return       None.
 *
 */

static void SA_processPDMData() {
    /*
#define MAX_VOLUME_IND        15
#define TOTAL_COUNT_STR_MAX    15
#define TOTAL_COUNT_BEG_IDX     7
    static int maxEnvSinceLastReport = 0;
    int i = 0;
    static char volumeIndicator[MAX_VOLUME_IND + 1] = "-";
    */
    PDMCC26XX_BufferRequest bufferRequest;
    if ( (saAudioState == SA_AUDIO_STREAMING) ||
         (saAudioState == SA_AUDIO_STARTING) ||
         (saAudioState == SA_AUDIO_STOPPING) ) {
        // Block ready, read it out and send it,
        // if we're not already sending
        while (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest)) {
            if (bufferRequest.status == PDMCC26XX_STREAM_BLOCK_READY) {
                if (!pdmParams.applyCompression) {
                	if(callbackCount>2)
                	{

                		/*
                		if(callbackCount==30)timeStart=Timestamp_get32();

                		                	if(callbackCount==31){
                		                	timeStop=Timestamp_get32();
                		                    timeInterval=timeStop-timeStart;
                		                	}
                		  */
                    SA_audioDataCollection((int16_t *)bufferRequest.buffer->pBuffer, AUDIO_BUF_UNCOMPRESSED_SIZE/2);
                    SA_events=SA_PCM_DSP;

                	}
                	/*
                     for(i=0;i<64;i++){
                    	 sum=0;
                    	 for(j=0;j<3;j++){
                    		 k=3*i+j;
                    		 if(PCMsamples[k]<0)
                    			 PCMsamples[k]=-PCMsamples[k];
                    		     sum+=PCMsamples[k];

                    	       }
                    	 if(sum>1200) {
                    	       consect[i]=1;//mark a possible position where sound may have arrived
                    	       position=i*3+(callbackCount-2)*192;

                    	 }

                     }
                     */
                     /*
                     for(i=0;i<64;i++){

                    	     if(consect[i]==0)
                    	      consectCount=0;
                    	     else
                    	      consectCount++;
                    	     if(consectCount==3)
                    	      position=(i-2)*3+(callbackCount-2)*192;

                    }
                   */

                    //* pPCMsamples=(int16_t)bufferRequest.buffer->pBuffer;
                    //PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
                    /*
                    gndNoiseSample[noiseTestCount]=abs(gndNoiseSample[noiseTestCount]);
                    if(noiseTestCount>=5)
                    gndNoise=gndNoise+gndNoiseSample[noiseTestCount];
                    noiseTestCount++;
                    if(noiseTestCount==10) gndNoise=gndNoise/5;
                    else
                    SA_audioDataCollection((int16_t *)bufferRequest.buffer->pBuffer, AUDIO_BUF_UNCOMPRESSED_SIZE);
                    */
                    /*
                    if (current > maxEnvSinceLastReport)
                    {
                        maxEnvSinceLastReport = current;
                    }
                    */
                }

                /*
                totalFrameCount++;
                currentFrameCount++;
                if ((bufferRequest.buffer->metaData.seqNum & 0x0000000F) == 0x00)
                {
                    Display_print1(display, 3, 1, "Frames %d", currentFrameCount);
                    if (totalFrameCount < 1000)
                    {
                        Display_print1(display, 4, 1, "Total  %d", totalFrameCount);
                    }
                    else
                    {
                        Display_print1(display, 4, 1, "Total  %dk", totalFrameCount/1000);
                    }
                    Display_print1(display, 6, 1, "Volume %d", maxEnvSinceLastReport);
                    volumeIndicator[0] = '-';
                    for (i = 1; i < ((maxEnvSinceLastReport > MAX_VOLUME_IND) ? MAX_VOLUME_IND : maxEnvSinceLastReport); i++)
                    {
                        volumeIndicator[i] = '-';
                    }
                    volumeIndicator[i] = '\0'; // Terminate string
                    Display_print0(display, 7, 1, volumeIndicator);
                    maxEnvSinceLastReport = 0;
                    if ((bufferRequest.buffer->metaData.seqNum & 0x0000001F) == 0x00)
                    {
                        PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
                    }
                    else if ((bufferRequest.buffer->metaData.seqNum & 0x0000001F) == 0x10)
                    {
                        PIN_setOutputValue(ledPinHandle, Board_LED2, 0);
                    }
                } else {
                    asm(" NOP");
                }
                */

                if (pdmParams.applyCompression) {
                    SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_COMPRESSED_SIZE);
                } else {
                    SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_UNCOMPRESSED_SIZE);
                }
            }
        }
        /*
         * We close the driver after flushing samples, after stopping the stream.
         */
        if (saAudioState == SA_AUDIO_STOPPING) {
            saAudioState = SA_AUDIO_IDLE;

            /* Close PDM driver */
            PDMCC26XX_close(pdmHandle);
            pdmHandle = NULL;


        }
    }
    else if (saAudioState == SA_AUDIO_IDLE)
    {
        // We may have received a callback for data after stopping the stream. Simply flush it.
        while (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest)) {
            if (pdmParams.applyCompression) {
                SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_COMPRESSED_SIZE);
            } else {
                SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_UNCOMPRESSED_SIZE);
            }
        }
    }
}

/**
 *  @fn          SA_PDMCC26XX_callbackFxn
 *
 *  @brief       Application callback function to handle notifications from PDM
 *               driver.
 *
 *  @param[in]   handle - PDM driver handle
 *               pStreamNotification - voice data stream
 *  @param[out]  None
 *
 *  @return  None.
 */
static void SA_PDMCC26XX_callbackFxn(PDMCC26XX_Handle handle, PDMCC26XX_StreamNotification *pStreamNotification) {
    if (pStreamNotification->status == PDMCC26XX_STREAM_BLOCK_READY) {
        //events =0;
        //events |= SA_PCM_BLOCK_READY;
      SA_processPDMData();
      if(saAudioState == SA_AUDIO_STREAMING)
      {
        callbackCount++;
        //if(callbackCount==2) callbackCount=0;
      }
    }
      else if (pStreamNotification->status == PDMCC26XX_STREAM_BLOCK_READY_BUT_PDM_OVERFLOW) {
        //events |= SA_PCM_BLOCK_READY;
        //events |= SA_PCM_ERROR;
         //PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
        if ((saAudioState == SA_AUDIO_STREAMING) ||
                (saAudioState == SA_AUDIO_STARTING)) {
                SA_events = SA_PCM_STOP;
            }
    } else if (pStreamNotification->status == PDMCC26XX_STREAM_STOPPING) {
        //events |= SA_PCM_BLOCK_READY;
        //events |= SA_PCM_ERROR;
       //PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
      if ((saAudioState == SA_AUDIO_STREAMING) ||
                (saAudioState == SA_AUDIO_STARTING)) {
                SA_events = SA_PCM_STOP;
            }
    } else {
        SA_events |= SA_PCM_ERROR;
    }

    Semaphore_post(saSem);
}

static int mallocCount = 0;
static void *SA_audioMalloc(uint_least16_t size)
{
    Error_Block eb;
    Error_init(&eb);
    if (size > 64)
    {
        mallocCount++;
    }
    return Memory_alloc(NULL, size, 0, &eb);
}

static int freeCount = 0;
static void SA_audioFree(void *msg, size_t size)
{
    if (size > 64)
    {
        freeCount++;
    }
    Memory_free(NULL, msg, size);
}

static void SA_audioDataCollection(int16_t *pPCMsamples, uint16_t numOfSamples)
{

    int i;
    for (i = 0; i < numOfSamples; i++){
        PCMsamples[i]=pPCMsamples[i];
    }

}
/*
static float SA_audioEnvelopDetection(int16_t *pPCMsamples, uint16_t numOfSamples){

    float sum=0;
    uint16_t i;
    for(i=0;i<numOfSamples;i++){
        sum+=pPCMsamples[i];
    }
    sum/=numOfSamples;
    return sum;
}
*/
/*
static void SA_audioDSP(int16_t Watch1[],float y[]){

  uint8_t k;
  uint8_t naxpy;
  uint8_t j;

  const float dv0[9] = { 3.12389E-5, 0.0, -0.00012, 0.0, 0.00019, 0.0,
      -0.00012, 0.0, 3.1239E-5 };

  const float dv1[9] = { 1.0, 5.38335, 14.46693, 24.29649, 27.74261,
      21.9239, 11.77931, 3.95509, 0.66301 };

    for (k = 0; k < 192; k++) {
      naxpy = 192 - k;
      if (naxpy >= 9) {
        naxpy = 9;
      }

      for (j = 0; j < naxpy; j++) {
        y[k + j] += Watch1[k] * dv0[j];
      }

      naxpy = 191 - k;
      if (naxpy >= 8) {
        naxpy = 8;
      }
      for (j = 1; j <= naxpy; j++) {
        y[k + j] += (-y[k])* dv1[j];
      }
    }
}
*/
/*
static int16_t  SA_audioCC(int16_t x[],int16_t y[],int16_t n){
	int16_t delay,sxymaxpt,i,j;
	long sxy,sxymax=0;
	   for (delay=0;delay<maxdelay;delay++) {
	      sxy = 0;
	      for (i=0;i<n;i++) {
	         j = i - delay;
	         while (j < 0)
	            j += n;
	         j %= n;
	         sxy += x[i] * y[j];
	      }
	      if(sxy>sxymax){
	    	  sxymax=sxy;
	    	  sxymaxpt=delay;
	      }
	   }
	   return sxymaxpt;
 }
*/
static void SA_audioIIR(int16_t x[],int yfil[]){
	/*
	 int16_t k;
	 int16_t j;
	 //int16_t i;
//const int dv17kifir[15]={-7070,6184,4128,-6144,5251,12328,-28254,38460,-28254,12328,5251,-6144,4128,6184,-7070};
const int16_t dv17kfir1[21]={-1142,1650,-487,-3584,6919,-2893,-8064,14751,-7067,-9717,18479,-9717,-7067,14751,-8064,-2893,6919,-3584,-487,1650,-1142};
//int yprefil[sampleNumber]={0};
for (k = 0; k < 21; k++) {
  for (j = k; j  < sampleNumber; j++) {
    yfil[j] += (dv17kfir1[k] * x[j - k])/100000;
  }
}
*/
	  int16_t k;
	  int16_t naxpy;
	  int16_t j;
	  int as;

	  int16_t dv0i[5]={3639,0,-7278,0,3639};
      int16_t dv1i[5]={1000,2062,2887,1886,837};
	  for (k = 0; k < sampleNumber; k++) {
	      naxpy = sampleNumber - k;
	      if (naxpy >= 5) {
	        naxpy = 5;
	      }

	      for (j = 0; j < naxpy; j+=2) {
	    	  yfil[k + j]+=(x[k] * dv0i[j])/10000;
	      }

	      naxpy = sampleNumber-1 - k;
	      if (naxpy >= 4) {
	        naxpy = 4;
	      }
	      as=-yfil[k];
	      for (j = 1; j <= naxpy; j++) {
	    	  yfil[k + j] +=(as * dv1i[j])/1000;
	      }
	      yfil[k]/=100;
	  }
}

enum NodeRadioOperationStatus NodeRadioTask_sendAdcData(uint16_t data)
{
    enum NodeRadioOperationStatus status;

    /* Get radio access sempahore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);

    /* Save data to send */
    adcData = data;

    /* Raise RADIO_EVENT_SEND_ADC_DATA event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_ADC_DATA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return status;
}


static void sendBleAdvertisement(struct DualModeSensorPacket sensorPacket)
{
    uint8_t txCnt, chan;

    //Prepare TLM frame interleaved with URL and UID
    if ((advertiserType == Node_AdertiserUrl) ||
        (advertiserType == Node_AdertiserMsUrl))
    {
        SEB_initTLM(sensorPacket.batt, sensorPacket.adcValue, sensorPacket.time100MiliSec);
    }

    if (advertiserType == Node_AdertiserUid)
    {
        //Prepare TLM frame interleaved with URL and UID
        SEB_initTLM(sensorPacket.batt, sensorPacket.adcValue, sensorPacket.time100MiliSec);

        //Set uid intance for the eddystone UID frame
        uidInstanceId[0] = sensorPacket.header.sourceAddress;
        SEB_initUID(uidNameSpace, uidInstanceId, NODE_0M_TXPOWER);
    }

    for (txCnt = 0; txCnt < SimpleBeacon_AdvertisementTimes; txCnt++)
    {
        for (chan = 37; chan < 40; chan++)
        {
            if ((advertiserType == Node_AdertiserMs) ||
                (advertiserType == Node_AdertiserMsUrl))
            {
                //set BTN value in Prop advertisement
                propAdvertisement[9] = !PIN_getInputValue(Board_BUTTON0);

                //advertisement advertise local name
                SimpleBeacon_sendFrame(localNameAdvFrame,  1, (uint64_t) 1<<chan);
                //advertisement advertise button value
                //SimpleBeacon_sendFrame(propAdvFrame, 1, (uint64_t) 1<<chan);
            }
            if ((advertiserType == Node_AdertiserUrl) ||
                (advertiserType == Node_AdertiserMsUrl))
            {
                SEB_sendFrame(SEB_FrameType_Url, bleMacAddr, 1, (uint64_t) 1<<chan);
                SEB_sendFrame(SEB_FrameType_Tlm, bleMacAddr, 1, (uint64_t) 1<<chan);
            }
            if (advertiserType == Node_AdertiserUid)
            {
                SEB_sendFrame(SEB_FrameType_Uuid, bleMacAddr, 1, (uint64_t) 1<<chan);
                SEB_sendFrame(SEB_FrameType_Tlm, bleMacAddr, 1, (uint64_t) 1<<chan);
            }
        }
        //sleep on all but last advertisement
        if(txCnt+1 < SimpleBeacon_AdvertisementTimes)
        {
            Task_sleep(SimpleBeacon_AdvertisementIntervals[txCnt]);
        }
    }

    /* Toggle activity LED */
    //PIN_setOutputValue(pinHandle, NODE_BLE_ACTIVITY_LED,!PIN_getOutputValue(NODE_BLE_ACTIVITY_LED));


}

static void BLE_Task(){

    uint8_t i;
    //PIN_setOutputValue(pinHandle, Board_LED1, 1);
    localNameAdvertisement[8] = ((position & 0x00F) < 0xa) ?
        	               (position & 0x00F) + 0x30:
        	               (position & 0x00F) - 0xa + 0x41;
                   localNameAdvertisement[7] = (((position & 0x0F0) >> 4) < 0xa) ?
                   	       ((position & 0x0F0) >> 4) + 0x30:
                   	       ((position & 0x0F0) >> 4) - 0xa + 0x41;
    	           localNameAdvertisement[6] = (((position & 0xF00) >> 8) < 0xa) ?
    	                   ((position & 0xF00) >> 8) + 0x30:
    	                   ((position & 0xF00) >> 8) - 0xa + 0x41;

    	           localNameAdvertisement[11] = ((callbackCountSend & 0x00F) < 0xa) ?
    	              	   (callbackCountSend & 0x00F) + 0x30:
    	              	   (callbackCountSend & 0x00F) - 0xa + 0x41;
    	           localNameAdvertisement[10] = (((callbackCountSend & 0x0F0) >> 4) < 0xa) ?
    	                   ((callbackCountSend & 0x0F0) >> 4) + 0x30:
    	                   ((callbackCountSend & 0x0F0) >> 4) - 0xa + 0x41;
    	           localNameAdvertisement[9] = (((callbackCountSend & 0xF00) >> 8) < 0xa) ?
    	          	       ((callbackCountSend & 0xF00) >> 8) + 0x30:
    	          	       ((callbackCountSend & 0xF00) >> 8) - 0xa + 0x41;
	           //for(i=0;i<30;i++)
	           //while(1)
	               	    {

	               	    	sendBleAdvertisement(dmSensorPacket);

	               	    }
   //BLE_Task_Done=true;
   //PIN_setOutputValue(pinHandle, Board_LED1, 0);
}


/* Calculates datarate from prescaler and rate word */
static uint32_t calculateSymbolRate(uint8_t prescaler, uint32_t rateWord)
{
    /* Calculate datarate according to TRM Section 23.7.5.2:
     * f_baudrate = (R * f_ref)/(p * 2^20)
     *   - R = rateWord
     *   - f_ref = 24Mhz
     *   - p = prescaler */
    uint64_t numerator = rateWord*24000000ULL;
    uint64_t denominator = prescaler*1048576ULL;
    uint32_t result = (uint32_t)(numerator/denominator);
    return result;
}

/* Copies all RX options from the SmartRF Studio exported RX command to the RX Sniff command */
static void initializeSniffCmdFromRxCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, rfc_CMD_PROP_RX_t* rxCmd)
{

    /* Copy RX configuration from RX command */
    memcpy(rxSniffCmd, rxCmd, sizeof(rfc_CMD_PROP_RX_t));

    /* Change to RX_SNIFF command from RX command */
    rxSniffCmd->commandNo = CMD_PROP_RX_SNIFF;
}

/* Configures Sniff-mode part of the RX_SNIFF command based on mode, datarate and wakeup interval */
static void configureSniffCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, enum CarrierSenseMode mode, uint32_t datarate, uint8_t wakeupPerSecond)
{
    /* Enable or disable RSSI */
    if ((mode == CarrierSenseMode_RSSI) || (mode == CarrierSenseMode_RSSIandPQT)) {
        rxSniffCmd->csConf.bEnaRssi        = 1;
    } else {
        rxSniffCmd->csConf.bEnaRssi        = 0;
    }

    /* Enable or disable PQT */
    if ((mode == CarrierSenseMode_PQT) || (mode == CarrierSenseMode_RSSIandPQT)) {
        rxSniffCmd->csConf.bEnaCorr        = 1;
        rxSniffCmd->csEndTrigger.triggerType  = TRIG_REL_START;
    } else {
        rxSniffCmd->csConf.bEnaCorr        = 0;
        rxSniffCmd->csEndTrigger.triggerType  = TRIG_NEVER;
    }

    /* General Carrier Sense configuration */
    rxSniffCmd->csConf.operation       = 1; /* Report Idle if RSSI reports Idle to quickly exit if not above
                                                 RSSI threshold */
    rxSniffCmd->csConf.busyOp          = 0; /* End carrier sense on channel Busy (the receiver will continue when
                                                 carrier sense ends, but it will then not end if channel goes Idle) */
    rxSniffCmd->csConf.idleOp          = 1; /* End on channel Idle */
    rxSniffCmd->csConf.timeoutRes      = 1; /* If the channel is invalid, it will return PROP_DONE_IDLE_TIMEOUT */

    /* RSSI configuration */
    rxSniffCmd->numRssiIdle            = 1; /* One idle RSSI samples signals that the channel is idle */
    rxSniffCmd->numRssiBusy            = 1; /* One busy RSSI samples signals that the channel is busy */
    rxSniffCmd->rssiThr    = (int8_t)WOR_RSSI_THRESHOLD; /* Set the RSSI threshold in dBm */

    /* PQT configuration */
    rxSniffCmd->corrConfig.numCorrBusy = 1;   /* One busy PQT samples signals that the channel is busy */
    rxSniffCmd->corrConfig.numCorrInv  = 1;   /* One busy PQT samples signals that the channel is busy */

    /* Calculate basic timing parameters */
    uint32_t symbolLengthUs  = 1000000UL/datarate;
    uint32_t preambleSymbols = (1000000UL/wakeupPerSecond)/symbolLengthUs;
    uint8_t syncWordSymbols  = RF_cmdPropRadioDivSetup.formatConf.nSwBits;

    /* Calculate sniff mode parameters */
    #define US_TO_RAT_TICKS 4
    #define CORR_PERIOD_SYM_MARGIN 16
    #define RX_END_TIME_SYM_MARGIN 8
    #define CS_END_TIME_MIN_TIME_SYM 30
    #define CS_END_TIME_MIN_TIME_STATIC_US 150

    /* Represents the time in which we need to receive corrConfig.numCorr* correlation peaks to detect preamble.
     * When continously checking the preamble quality, this period has to be wide enough to also contain the sync
     * word, with a margin. If it is not, then there is a chance the SNIFF command will abort while receiving the
     * sync word, as it no longer detects a preamble. */
    uint32_t correlationPeriodUs = (syncWordSymbols + CORR_PERIOD_SYM_MARGIN)*symbolLengthUs;

    /* Represents the time where we will force a check if preamble is present (only done once).
     * The main idea is that his should be shorter than "correlationPeriodUs" so that if we get RSSI valid, but
     * there is not a valid preamble on the air, we will leave RX as quickly as possible. */
    uint32_t csEndTimeUs = (CS_END_TIME_MIN_TIME_SYM*symbolLengthUs + CS_END_TIME_MIN_TIME_STATIC_US);

    /* Represents the maximum time from the startTrigger to when we expect a sync word to be received. */
    uint32_t rxEndTimeUs = (preambleSymbols + syncWordSymbols + RX_END_TIME_SYM_MARGIN)*symbolLengthUs;

    /* Set sniff mode timing configuration in sniff command in RAT ticks */
    rxSniffCmd->corrPeriod = (uint16_t)(correlationPeriodUs * US_TO_RAT_TICKS);
    rxSniffCmd->csEndTime  = (uint32_t)(csEndTimeUs * US_TO_RAT_TICKS);
    rxSniffCmd->endTime    = (uint32_t)(rxEndTimeUs * US_TO_RAT_TICKS);

    /* Set correct trigger types */
    rxSniffCmd->endTrigger.triggerType   = TRIG_REL_START;
    rxSniffCmd->startTrigger.triggerType = TRIG_ABSTIME;
    rxSniffCmd->startTrigger.pastTrig    = 1;
}

/* Called for every received packet and command done */
void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    /* If we've received a new packet and it's available to read out */
    if (e & RF_EventRxEntryDone)
    {
        do
        {
            /* Toggle LED on RX */
            //PIN_setOutputValue(ledPinHandle, Board_LED1, !PIN_getOutputValue(Board_LED1));

            /* Get current unhandled data entry */
            currentDataEntry = RFQueue_getDataEntry();

            /* Handle the packet data, located at &currentDataEntry->data:
             * - Length is the first byte with the current configuration
             * - Data starts from the second byte */
            packetLength      = *(uint8_t*)(&currentDataEntry->data);
            packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

            /* This code block is added to avoid a compiler warning.
            * Normally, an application will reference these variables for
            * useful data. */
            volatile uint8_t dummy;
            dummy = packetLength;
            dummy = dummy + packetDataPointer[0];

        } while(RFQueue_nextEntry() == DATA_ENTRY_FINISHED);
    }
}

