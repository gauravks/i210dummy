/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for Epl-Userspace-Event-Module

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/06/20 k.t.:   start of the implementation

****************************************************************************/

#include "user/EplEventu.h"
#include "user/EplNmtu.h"
#include "user/EplNmtMnu.h"
#include "user/EplSdoAsySequ.h"
#include "user/EplDlluCal.h"
#include "user/EplLedu.h"
#include "Benchmark.h"

#if EPL_USE_SHAREDBUFF != FALSE
    #include "SharedBuff.h"
#endif

#if (EPL_USE_SHAREDBUFF == FALSE) \
    || (EPL_EVENT_USE_KERNEL_QUEUE == FALSE)
    #include "kernel/EplEventk.h"
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    void  PUBLIC  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)
#endif

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
#if EPL_USE_SHAREDBUFF != FALSE
    tShbInstance        m_pShbKernelToUserInstance;
    BYTE                m_abRxBuffer[sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE];
#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
    tShbInstance        m_pShbUserToKernelInstance;
#endif
#endif
    tEplProcessEventCb  m_pfnApiProcessEventCb;

}tEplEventuInstance;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//#if EPL_USE_SHAREDBUFF == FALSE
static tEplEventuInstance EplEventuInstance_g;
//#endif

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

#if EPL_USE_SHAREDBUFF != FALSE
// callback function for incoming events
static void  EplEventuRxSignalHandlerCb (
    tShbInstance pShbRxInstance_p,
    unsigned long ulDataSize_p);
#endif

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Epl-User-Event>                                    */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplEventuInit
//
// Description: function initialize the first instance
//
//
//
// Parameters:  pfnApiProcessEventCb_p  = function pointer for API event callback
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuInit(tEplProcessEventCb pfnApiProcessEventCb_p)
{
tEplKernel Ret;


    Ret = EplEventuAddInstance(pfnApiProcessEventCb_p);


return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplEventuAddInstance
//
// Description: function add one more instance
//
//
//
// Parameters:  pfnApiProcessEventCb_p  = function pointer for API event callback
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuAddInstance(tEplProcessEventCb pfnApiProcessEventCb_p)
{
tEplKernel      Ret;
#if EPL_USE_SHAREDBUFF != FALSE
tShbError       ShbError;
unsigned int    fShbNewCreated;
#endif

    Ret = kEplSuccessful;


    // init instance variables
    EplEventuInstance_g.m_pfnApiProcessEventCb = pfnApiProcessEventCb_p;

#if EPL_USE_SHAREDBUFF != FALSE
    // init shared loop buffer
    // kernel -> user
    ShbError = ShbCirAllocBuffer (EPL_EVENT_SIZE_SHB_KERNEL_TO_USER,
                                  EPL_EVENT_NAME_SHB_KERNEL_TO_USER,
                                  &EplEventuInstance_g.m_pShbKernelToUserInstance,
                                  &fShbNewCreated);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventuAddInstance(): ShbCirAllocBuffer(K2U) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }


#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
    // user -> kernel
    ShbError = ShbCirAllocBuffer (EPL_EVENT_SIZE_SHB_USER_TO_KERNEL,
                                  EPL_EVENT_NAME_SHB_USER_TO_KERNEL,
                                  &EplEventuInstance_g.m_pShbUserToKernelInstance,
                                  &fShbNewCreated);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventuAddInstance(): ShbCirAllocBuffer(U2K) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }
#endif

    // register eventhandler
    ShbError = ShbCirSetSignalHandlerNewData (EplEventuInstance_g.m_pShbKernelToUserInstance,
                                    EplEventuRxSignalHandlerCb,
                                    kShbPriorityNormal);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventuAddInstance(): ShbCirSetSignalHandlerNewData(K2U) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }

Exit:
#endif

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplEventuDelInstance
//
// Description: function delete instance an free the bufferstructure
//
//
//
// Parameters:
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuDelInstance()
{
tEplKernel      Ret;
#if EPL_USE_SHAREDBUFF != FALSE
tShbError       ShbError;
#endif

    Ret = kEplSuccessful;

#if EPL_USE_SHAREDBUFF != FALSE
    // set eventhandler to NULL
    ShbError = ShbCirSetSignalHandlerNewData (EplEventuInstance_g.m_pShbKernelToUserInstance,
                                    NULL,
                                    kShbPriorityNormal);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventuDelInstance(): ShbCirSetSignalHandlerNewData(K2U) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }

#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
    // free buffer User -> Kernel
    ShbError = ShbCirReleaseBuffer (EplEventuInstance_g.m_pShbUserToKernelInstance);
    if((ShbError != kShbOk) && (ShbError != kShbMemUsedByOtherProcs))
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventuDelInstance(): ShbCirReleaseBuffer(U2K) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }
    else
    {
        EplEventuInstance_g.m_pShbUserToKernelInstance = NULL;
    }
#endif

    // free buffer  Kernel -> User
    ShbError = ShbCirReleaseBuffer (EplEventuInstance_g.m_pShbKernelToUserInstance);
    if((ShbError != kShbOk) && (ShbError != kShbMemUsedByOtherProcs))
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventuDelInstance(): ShbCirReleaseBuffer(K2U) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }
    else
    {
        EplEventuInstance_g.m_pShbKernelToUserInstance = NULL;
    }

#endif

return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplEventuProcess
//
// Description: Kernelthread that dispatches events in kernelspace
//
//
//
// Parameters:  pEvent_p = pointer to event-structure from buffer
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuProcess(tEplEvent* pEvent_p)
{
tEplKernel              Ret;
tEplEventSource         EventSource;

    Ret = kEplSuccessful;

    // check m_EventSink
    switch(pEvent_p->m_EventSink)
    {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)
        // NMT-User-Module
        case kEplEventSinkNmtu:
        {
            Ret = EplNmtuProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceNmtu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // NMT-MN-User-Module
        case kEplEventSinkNmtMnu:
        {
            Ret = EplNmtMnuProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceNmtMnu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

#if ((((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)   \
     || (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0))
        // events for asynchronous SDO Sequence Layer
        case kEplEventSinkSdoAsySeq:
        {
            Ret = EplSdoAsySeqProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceSdoAsySeq;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

        // LED user part module
        case kEplEventSinkLedu:
        {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)
            Ret = EplLeduProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceLedu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            break;
        }

        // event for EPL api
        case kEplEventSinkApi:
        {
            if (EplEventuInstance_g.m_pfnApiProcessEventCb != NULL)
            {
                Ret = EplEventuInstance_g.m_pfnApiProcessEventCb(pEvent_p);
                if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
                {
                    EventSource = kEplEventSourceEplApi;

                    // Error event for API layer
                    EplEventuPostError(kEplEventSourceEventu,
                                    Ret,
                                    sizeof(EventSource),
                                    &EventSource);
                }
            }
            break;

        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLU)) != 0)
        case kEplEventSinkDlluCal:
        {
            Ret = EplDlluCalProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceDllu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;

        }
#endif

        case kEplEventSinkErru:
        {
            /*
            Ret = EplErruProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceErru;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            */
            break;

        }

        // unknown sink
        default:
        {
            Ret = kEplEventUnknownSink;

            // Error event for API layer
            EplEventuPostError(kEplEventSourceEventu,
                            Ret,
                            sizeof(pEvent_p->m_EventSink),
                            &pEvent_p->m_EventSink);
        }

    } // end of switch(pEvent_p->m_EventSink)

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplEventuPost
//
// Description: post events from userspace
//
//
//
// Parameters:  pEvent_p = pointer to event-structure from buffer
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuPost(tEplEvent * pEvent_p)
{
tEplKernel      Ret;
#if EPL_USE_SHAREDBUFF != FALSE
tShbError       ShbError;
tShbCirChunk    ShbCirChunk;
unsigned long   ulDataSize;
unsigned int    fBufferCompleted;
#endif

    Ret = kEplSuccessful;


#if EPL_USE_SHAREDBUFF != FALSE
    // 2006/08/03 d.k.: Event and argument are posted as separate chunks to the event queue.
    ulDataSize = sizeof(tEplEvent) + ((pEvent_p->m_pArg != NULL) ? pEvent_p->m_uiSize : 0);
#endif

    // decide in which buffer the event have to write
    switch(pEvent_p->m_EventSink)
    {
        // kernelspace modules
        case kEplEventSinkSync:
        case kEplEventSinkNmtk:
        case kEplEventSinkDllk:
        case kEplEventSinkDllkCal:
        case kEplEventSinkPdok:
        case kEplEventSinkPdokCal:
        case kEplEventSinkErrk:
        {
#if (EPL_USE_SHAREDBUFF != FALSE) \
    && (EPL_EVENT_USE_KERNEL_QUEUE != FALSE)
            // post message
            ShbError = ShbCirAllocDataBlock (EplEventuInstance_g.m_pShbUserToKernelInstance,
                                   &ShbCirChunk,
                                   ulDataSize);
            if (ShbError != kShbOk)
            {
                EPL_DBGLVL_EVENTK_TRACE("EplEventuPost(): ShbCirAllocDataBlock(U2K) -> 0x%X\n", ShbError);
                Ret = kEplEventPostError;
                goto Exit;
            }
            ShbError = ShbCirWriteDataChunk (EplEventuInstance_g.m_pShbUserToKernelInstance,
                                   &ShbCirChunk,
                                   pEvent_p,
                                   sizeof (tEplEvent),
                                   &fBufferCompleted);
            if (ShbError != kShbOk)
            {
                EPL_DBGLVL_EVENTK_TRACE("EplEventuPost(): ShbCirWriteDataChunk(U2K) -> 0x%X\n", ShbError);
                Ret = kEplEventPostError;
                goto Exit;
            }
            if (fBufferCompleted == FALSE)
            {
                ShbError = ShbCirWriteDataChunk (EplEventuInstance_g.m_pShbUserToKernelInstance,
                                       &ShbCirChunk,
                                       pEvent_p->m_pArg,
                                       (unsigned long) pEvent_p->m_uiSize,
                                       &fBufferCompleted);
                if ((ShbError != kShbOk) || (fBufferCompleted == FALSE))
                {
                    EPL_DBGLVL_EVENTK_TRACE("EplEventuPost(): ShbCirWriteDataChunk2(U2K) -> 0x%X\n", ShbError);
                    Ret = kEplEventPostError;
                    goto Exit;
                }
            }
#else
            #if EPL_EVENT_USE_KERNEL_QUEUE == FALSE
                EplTgtEnableGlobalInterrupt(FALSE);
            #endif
            Ret = EplEventkProcess(pEvent_p);
            #if EPL_EVENT_USE_KERNEL_QUEUE == FALSE
                EplTgtEnableGlobalInterrupt(TRUE);
            #endif
#endif

            break;
        }

        // userspace modules
        case kEplEventSinkNmtMnu:
        case kEplEventSinkNmtu:
        case kEplEventSinkSdoAsySeq:
        case kEplEventSinkApi:
        case kEplEventSinkDlluCal:
        case kEplEventSinkErru:
        case kEplEventSinkLedu:
        {
#if EPL_USE_SHAREDBUFF != FALSE
            // post message
            ShbError = ShbCirAllocDataBlock (EplEventuInstance_g.m_pShbKernelToUserInstance,
                                   &ShbCirChunk,
                                   ulDataSize);
            if(ShbError != kShbOk)
            {
                EPL_DBGLVL_EVENTK_TRACE("EplEventuPost(): ShbCirAllocDataBlock(K2U) -> 0x%X\n", ShbError);
                Ret = kEplEventPostError;
                goto Exit;
            }
            ShbError = ShbCirWriteDataChunk (EplEventuInstance_g.m_pShbKernelToUserInstance,
                                   &ShbCirChunk,
                                   pEvent_p,
                                   sizeof (tEplEvent),
                                   &fBufferCompleted);
            if(ShbError != kShbOk)
            {
            	printk("Write Error 1\n");
                EPL_DBGLVL_EVENTK_TRACE("EplEventuPost(): ShbCirWriteDataChunk(K2U) -> 0x%X\n", ShbError);
                Ret = kEplEventPostError;
                goto Exit;
            }
            if (fBufferCompleted == FALSE)
            {
                ShbError = ShbCirWriteDataChunk (EplEventuInstance_g.m_pShbKernelToUserInstance,
                                       &ShbCirChunk,
                                       pEvent_p->m_pArg,
                                       (unsigned long) pEvent_p->m_uiSize,
                                       &fBufferCompleted);
                if ((ShbError != kShbOk) || (fBufferCompleted == FALSE))
                {
                	printk("Write Error 2\n");
                    EPL_DBGLVL_EVENTK_TRACE("EplEventuPost(): ShbCirWriteDataChunk2(K2U) -> 0x%X\n", ShbError);
                    Ret = kEplEventPostError;
                    goto Exit;
                }
            }

#else
            Ret = EplEventuProcess(pEvent_p);
#endif

            break;
        }

        default:
        {
            Ret = kEplEventUnknownSink;
        }


    }// end of switch(pEvent_p->m_EventSink)

#if EPL_USE_SHAREDBUFF != FALSE
Exit:
#endif
    return Ret;

}
//---------------------------------------------------------------------------
//
// Function:    EplEventuPostError
//
// Description: post errorevent from userspace
//
//
//
// Parameters:  EventSource_p   = source-module of the errorevent
//              EplError_p     = code of occurred error
//              uiArgSize_p     = size of the argument
//              pArg_p          = pointer to the argument
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuPostError(tEplEventSource EventSource_p,
                                     tEplKernel      EplError_p,
                                     unsigned int    uiArgSize_p,
                                     void*           pArg_p)
{
tEplKernel      Ret;
tEplEventError  EventError;
tEplEvent       EplEvent;

    Ret = kEplSuccessful;

    // create argument
    EventError.m_EventSource = EventSource_p;
    EventError.m_EplError = EplError_p;
    uiArgSize_p = (unsigned int) min ((size_t) uiArgSize_p, sizeof (EventError.m_Arg));
    EPL_MEMCPY(&EventError.m_Arg, pArg_p, uiArgSize_p);

    // create event
    EplEvent.m_EventType = kEplEventTypeError;
    EplEvent.m_EventSink = kEplEventSinkApi;
    EPL_MEMSET(&EplEvent.m_NetTime, 0x00, sizeof(EplEvent.m_NetTime));
    EplEvent.m_uiSize = (memberoffs (tEplEventError, m_Arg) + uiArgSize_p);
    EplEvent.m_pArg = &EventError;

    // post errorevent
    Ret = EplEventuPost(&EplEvent);

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplEventuRxSignalHandlerCb()
//
// Description: Callback-function for events from kernelspace
//
//
//
// Parameters:  pShbRxInstance_p    = Instance-pointer for buffer
//              ulDataSize_p        = size of data
//
//
// Returns: void
//
//
// State:
//
//---------------------------------------------------------------------------
#if EPL_USE_SHAREDBUFF != FALSE
static void  EplEventuRxSignalHandlerCb (
    tShbInstance pShbRxInstance_p,
    unsigned long ulDataSize_p)
{
tEplEvent      *pEplEvent;
tShbError       ShbError;
BYTE*           pabDataBuffer;

    TGT_DBG_SIGNAL_TRACE_POINT(21);

    pabDataBuffer = &EplEventuInstance_g.m_abRxBuffer[0];

    // copy data from event queue
    ShbError = ShbCirReadDataBlock (pShbRxInstance_p,
                            pabDataBuffer,
                            sizeof(EplEventuInstance_g.m_abRxBuffer),
                            &ulDataSize_p);
    if(ShbError != kShbOk)
    {
        EplEventuPostError(kEplEventSourceEventu, kEplEventReadError, sizeof (ShbError), &ShbError);
        // error goto exit
        goto Exit;
    }

    // resolve the pointer to the event structure
    pEplEvent = (tEplEvent *) pabDataBuffer;
    // set Datasize
    pEplEvent->m_uiSize = (ulDataSize_p - sizeof(tEplEvent));
    if(pEplEvent->m_uiSize > 0)
    {
        // set pointer to argument
        pEplEvent->m_pArg = &pabDataBuffer[sizeof(tEplEvent)];
    }
    else
    {
        //set pointer to NULL
        pEplEvent->m_pArg = NULL;
    }

    BENCHMARK_MOD_28_SET(1);
    // call processfunction
    EplEventuProcess(pEplEvent);

    BENCHMARK_MOD_28_RESET(1);

Exit:
    return;
}
#endif

// EOF

