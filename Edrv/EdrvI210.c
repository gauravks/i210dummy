/****************************************************************************

    Copyright (c) 2013 Kalycito Infotech Private Limited

    Project: openPOWERLINK

    Description: Ethernet driver for Intel's I210 Ethernet Controller

    License:

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions
        are met:

        1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

        2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

        3. Neither the name of Kalycito Infotech nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without prior written permission. For written
        permission, please contact info@kalycito.com.

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

        1. The validity or enforceability in that jurisdiction of any other
        provision of this License; or
        2. The validity or enforceability in other jurisdictions of that or
        any other provision of this License.

    Based on
        1. Edrv82573.c and igb_main.c
****************************************************************************/
#include "global.h"
#include "EplInc.h"
#include "edrv.h"

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>

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
#define VENDOR_ID 0x8086
#define DEVICE_ID 0x1533

#define PROMICIOUS_MODE
#define	QAV_MODE
//#define DISABLE_AUTONEG
#define CONFIG_BAR  0
#define DRIVER_NAME "epl"

#ifdef USE_MULTIPLE_QUEUE
#define EDRV_MAX_RX_QUEUES       4
#define EDRV_MAX_TX_QUEUES       4
#define EDRV_MAX_QUEUE_VECTOR	 4
#else
#define EDRV_MAX_RX_QUEUES       1
#define EDRV_MAX_TX_QUEUES       1
#endif

#define EDRV_CTRL_REG            0x00000
#define EDRV_CTRL_EXTN_REG       0x00018
#define EDRV_STATUS_REG          0x00008
#define EDRV_MDIC_REG            0x00020
#define EDRV_INTR_READ_REG       0x01500
#define EDRV_INTR_SET_REG        0x01504
#define EDRV_INTR_MASK_SET_READ  0x01508
#define EDRV_INTR_MASK_CLEAR     0x0150C
#define EDRV_INTR_ACK_AUTO_MASK  0x01510
#define EDRV_INTR_GPIE_REG       0x01514
#define EDRV_IVAR0_REG			 0x01700
#define EDRV_IVAR_MISC			 0x01740 /* IVAR for "other" causes - RW */
#define EDRV_EXT_INTR_REG        0X01580
#define EDRV_EXT_INTR_MASK_CLEAR 0x01528
#define EDRV_EXT_INTR_MASK_SET   0x01524
#define EDRV_MDICNFG_REG         0x00E04
#define EDRV_SWSM_REG            0x05B50     // Software Semaphore
#define EDRV_TIPG_REG            0x00410     // Transmit Inter Packet Gap
#define EDRV_TXPBSIZE_REG        0x03404
#define EDRV_DCA_CTRL_REG        0x05B74
#define EDRV_EECD_REG	         0x00010
#define EDRV_SW_FW_SYNC	         0x05B5C
#define	EDRV_SYSTIMR_REG		 0x0B6F8
#define EDRV_SYSTIML_REG		 0x0B600 /* System time register Low - RO */
#define EDRV_SYSTIMH_REG		 0x0B604 /* System time register High - RO */
#define EDRV_TSAUXC_REG			 0x0B640
#define EDRV_STAT_TPT            0x040D4
#define EDRV_MRQC_REG			 0x05818
#define EDRV_EEER_REG			 0x00E30

#define SDP0_SET_DIR_OUT		0x00400000
#define SDP0_SET_HIGH			0x00040000
#define SDP1_SET_DIR_OUT		0x00800000
#define SDP1_SET_HIGH			0x00080000
#define SDP2_SET_DIR_OUT    	0x00000400
#define SDP3_SET_DIR_OUT    	0x00000800
#define SDP2_SET_HIGH       	0x00000040
#define SDP3_SET_HIGH       	0x00000080


#define EDRV_SWSM_SMBI           0x00000001
#define EDRV_SWSM_SWESMBI        0x00000002  // Software EEPROM Semaphore Bit
#define EDRV_TIPG_DEF            0x00702008
#define EDRV_EECD_AUTO_RD		 0x00000200
#define EDRV_SWFW_PHY0_SM	     0x02
// Tx Registers
#define EDRV_TCTL_REG            0x00400
#define EDRV_TCTL_EXT_REG        0x00404
#define EDRV_DTXCTL_REG          0x03590
#define EDRV_DTX_MAX_PKTSZ_REG   0x0355C
#define EDRV_TXPBSIZE_REG		 0x03404
#define EDRV_RXPBSIZE_REG		 0x02404
#define EDRV_TQAVCTRL_REG        0x03570
#define EDRV_LAUNCH_OSO			 0x03578
#define EDRV_TQAVARBCTRL_REG	 0x03574 // TODO :Verify this register
#define EDRV_TXPBSIZE_DEF		 0x04104208 // 8 Kb TQ0, 8Kb TQ1, 4 Kb TQ2, 4Kb TQ3 and 4 Kb Os2Bmc
#define EDRV_TSAUXC_SAMP_AUTO	 0x00000008
#define EDRV_TSAUXC				 0x0B640
#define EDRV_AUXSTMPL0 			 0x0B65C /* Auxiliary Time Stamp 0 Reg - Low */
#define EDRV_AUXSTMPH0 			 0x0B660 /* Auxiliary Time Stamp 0 Reg - Low */

#define EDRV_TDBAL(n)            ((n < 4) ? (0x0E000 + 0x40 * n) :\
                                 (0x0E000 + 0x40 * (n - 4)))
#define EDRV_TDBAH(n)            ((n < 4) ? (0x0E004 + 0x40 * n) :\
                                 (0x0E004 + 0x40 * (n - 4)))
#define EDRV_TDLEN(n)            ((n < 4) ? (0x0E008 + 0x40 * n) :\
                                 (0x0E008 + 0x40 * (n - 4)))
#define EDRV_TDHEAD(n)           ((n < 4) ? (0x0E010 + 0x40 * n) :\
                                 (0x0E010 + 0x40 * (n - 4)))
#define EDRV_TDTAIL(n)           ((n < 4) ? (0x0E018 + 0x40 * n) :\
                                 (0x0E018 + 0x40 * (n - 4)))
#define EDRV_TXDCTL(n)           ((n < 4) ? (0x0E028 + 0x40 * n) :\
                                 (0x0E028 + 0x40 * (n - 4)))
#define EDRV_TQAVCC(n)           ((n < 2) ? (0x03004 + 0x40 * n) :\
                                 (0x03004 + 0x40 * (n - 2)))
#define EDRV_TQAVHC(n)           ((n < 2) ? (0x0300C + 0x40 * n) :\
                                 (0x0300C + 0x40 * (n - 2)))
#define EDRV_TDWBAL(n)           ((n < 4) ? (0x0E038 + 0x40 * n) :\
                                 (0x0E038 + 0x40 * (n - 1)))
// Rx Registers
#define EDRV_RCTL_REG			 0x00100
#define EDRV_RXPBSIZE_REG		 0x02404
#define EDRV_RXPBSIZE_CLEAR		 0x3F
#define EDRV_RXPBSIZE_DEF		 0x8000001E
#define EDRV_SRRCTL(n)           ((n < 4) ? (0x0C00C + 0x40 * n) :\
                                 (0x0C00C + 0x40 * (n - 4)))
#define EDRV_RDBAL(n)            ((n < 4) ? (0x0C000 + 0x40 * n) :\
                                 (0x0C000 + 0x40 * (n -4)))
#define EDRV_RDBAH(n)            ((n < 4) ? (0x0C004 + 0x40 * n) :\
                                 (0x0C004 + 0x40 * (n - 4)))
#define EDRV_RDLEN(n)            ((n < 4) ? (0x0C008 + 0x40 * n) :\
                                 (0x0C008 + 0x40 * (n - 4)))
#define EDRV_RDHEAD(n)           ((n < 4) ? (0x0C010 + 0x40 * n) :\
                                 (0x0C010 + 0x40 * (n - 4)))
#define EDRV_RDTAIL(n)           ((n < 4) ? (0x0C018 + 0x40 * n) :\
                                 (0x0C018 + 0x40 * (n - 4)))
#define EDRV_RXDCTL(n)           ((n < 4) ? (0x0C028 + 0x40 * n) :\
                                 (0x0C028 + 0x40 * (n - 4)))
#define EDRV_PQGPRC(n)           ((n < 4) ? (0x10010 + 0x100 * n) :\
                                 (0x10010 + 0x100  * (n - 4)))
#define EDRV_RAL(n)				 (0x05400 + 8 * n)
#define EDRV_RAH(n)				 (0x05404 + 8 * n)


// MDIC specific defines
#define EDRV_MDIC_DATA_MASK      0x0000FFFF
#define EDRV_MDIC_OP_READ        0x08000000
#define EDRV_MDIC_OP_WRITE       0x04000000
#define EDRV_MDIC_RD             0x10000000
#define EDRV_MDIC_INTR_DIS       0x20000000
#define EDRV_MDIC_ERR            0x40000000
#define EDRV_MDIC_REGADD_MASK    0x001F0000

// Interrupt Defines
#define EDRV_IVAR_VALID			     0x80
#define EDRV_EIMC_CLEAR_ALL      0xC000001F
#define EDRV_INTR_GPIE_NSICR     (1 << 0 )
#define EDRV_INTR_GPIE_MULT_MSIX (1 << 4 )
#define EDRV_INTR_GPIE_PBA		   (1 << 31)
#define EDRV_INTR_ICR_TXDW		   0x00000001
#define EDRV_INTR_ICR_RXDW		 (1 << 7 )
#define	EDRV_INTR_ICR_RXDMT0	 (1 << 4 )
#define EDRV_INTR_ICR_RXMISS		(1 << 6)
#define EDRV_INTR_ICR_FER		 (1 << 22)
#define EDRV_EIMC_OTHR_EN        (1 << 31)
#define EDRV_EICS_OTHER			     (1 << 0 )
#define EDRV_EICS_QUEUE			     0x0000001E
#define EDRV_EICS_TXRXQUEUE1	   (1 << 1 )
#define EDRV_EICS_TXRXQUEUE2	   (1 << 2 )
#define EDRV_EICS_TXRXQUEUE3	   (1 << 3 )
#define EDRV_EICS_TXRXQUEUE4       (1 << 4 )

#define EDRV_INTR_ICR_MASK_DEF (EDRV_INTR_ICR_TXDW \
                               | EDRV_INTR_ICR_RXDW \
                               | EDRV_INTR_ICR_RXDMT0 \
                               | EDRV_INTR_ICR_RXMISS \
                               | EDRV_INTR_ICR_FER)
// Phy Defines
#define PHY_CONTROL_REG_OFFSET          0x00
#define PHY_STATUS_REG_OFFSET           0x01
#define PHY_LINK_SPEED_100              0x2000
#define PHY_RESET                       0x8000
#define PHY_MODE_FD                     0x0100
#define PHY_CONTROL_POWER_DOWN          0x0800
#define PHY_I210_COPPER_SPEC		        0x0010
#define PHY_I210_CS_POWER_DOWN		      0x0002
// Control Register Defines
#define EDRV_CTRL_FD                    0x00000001
#define EDRV_CTRL_MASTER_DIS            (1 << 2 )
#define EDRV_CTRL_SLU                   (1 << 6 )
#define EDRV_CTRL_ILOS                  (1 << 7 )
#define EDRV_CTRL_SPEED_100             (1 << 8 )
#define EDRV_CTRL_FRCSPD                (1 << 11)
#define EDRV_CTRL_RST                   (1 << 26)
#define EDRV_CTRL_RFCE                  (1 << 27)
#define EDRV_CTRL_TFCE                  (1 << 28)
#define EDRV_CTRL_DEV_RST               (1 << 29)
#define EDRV_CTRL_PHY_RST               (1 << 31)

// Control Extension Define
#define EDRV_CTRL_EXTN_DRV_LOAD         (1 << 28)

// Status registers defines
#define EDRV_STATUS_MASTER_EN           (1 << 19)
#define EDRV_STATUS_PF_RST_DONE         (1 << 21)
#define EDRV_STATUS_LU                  (1 << 1)
#define EDRV_STATUS_DEV_RST_SET         (1 << 20)
#define EDRV_SW_RST_DONE_TIMEOUT        10   // ms
#define EDRV_MASTER_DIS_TIMEOUT         90   // ms
#define EDRV_LINK_UP_TIMEOUT            3000 // ms
#define EDRV_AUTO_READ_DONE_TIMEOUT		  10
// Tx Descriptor Defines
#define EDRV_MAX_TX_DESCRIPTOR          256  //Max no of Desc in mem
#define EDRV_MAX_TX_DESC_LEN           (EDRV_MAX_TX_DESCRIPTOR - 1)     //one slot to diff full
#define EDRV_MAX_TTX_DESC_LEN		   ((EDRV_MAX_TX_DESCRIPTOR >> 1) -1)
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS    			128			//Max no of Buffers
#endif

#define EDRV_MAX_FRAME_SIZE             0x600 // 1536
//AB: changed Buff size
#define EDRV_TX_BUFFER_SIZE             ( EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE)

#define EDRV_TX_DESCS_SIZE              (EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvAdvTxDesc))

#define EDRV_TDESC_CMD_DEXT             (1 << 29) // Descriptor type
#define EDRV_TDESC_CMD_RS               (1 << 27) // Report Status
#define EDRV_TDESC_CMD_IC               (1 << 26) // Insert Checksum
#define EDRV_TDESC_CMD_IFCS             (1 << 25) // Insert FCS
#define EDRV_TDESC_CMD_EOP              (1 << 24) // End of Packet
#define EDRV_TDESC_STATUS_DD            (1 << 0 ) // Descriptor done
#define EDRV_TDESC_DTYP_CTXT			(2 << 20) // Context Descriptor
#define EDRV_TDESC_DTYP_ADV 			(3 << 20) // Advance Tx Descriptor

#define EDRV_TCTL_CT		            0x000000F0
#define EDRV_TCTL_SWFLSH				0x04000000
#define EDRV_TCTL_CLEAR_CT				0x00000ff0
#define EDRV_TCTL_EN		            0x00000002
#define EDRV_TCTL_PSP		            0x00000008
#define EDRV_TCTL_RTLC		            0x01000000
#define EDRV_TCTL_EXT_COLD_CLEAR        0x000FFC00
#define EDRV_TCTL_EXT_COLD              0x0003F000 // default value as per 802.3 spec
#define EDRV_TXDCTL_PTHRESH             0
#define EDRV_TXDCTL_HTHRESH             0
#define EDRV_TXDCTL_WTHRESH             0
#define EDRV_TXDCTL_QUEUE_EN            0x02000000
#define EDRV_TXDCTL_PRIORITY			(1 << 27)
#define EDRV_TQAVCTRL_TXMODE			(1 << 0 )
#define EDRV_TQAVCTRL_FETCH_ARB 		(1 << 4 )
#define EDRV_TQAVCTRL_DTRANSARB			(1 << 8 )
#define EDRV_TQAVCTRL_TRANSTIM			(1 << 9 )
#define EDRV_TQAVCTRL_SP_WAIT_SR 		(1 << 10)
#define EDRV_TQAVCTRL_1588_STAT_EN		(1 << 2 )
#define EDRV_TQAVCC_QUEUE_MODE_SR			(1 << 31)
#define EDRV_TQAVCTRL_FETCH_TM_SHIFT	16
#define EDRV_LAUNCH_OSO_SHIFT			5

#define EDRV_TQAVARBCTRL_TXQPRIO(_q, _n)  (((_n) & 0x3) << (_q << 2))
#define EDRV_TXPBSIZE_TX1PB_SHIFT    6
#define EDRV_TXPBSIZE_TX2PB_SHIFT    12
#define EDRV_TXPBSIZE_TX3PB_SHIFT    18
//Rx Descriptor Defines
#define EDRV_MAX_RX_DESCRIPTOR          256  //Max no of Desc in mem
#define EDRV_MAX_RX_DESC_LEN           (EDRV_MAX_RX_DESCRIPTOR - 1)
#define EDRV_MAX_RX_BUFFERS             256  //Max no of Buffers
#define EDRV_RX_BUFFER_SIZE             (EDRV_MAX_RX_BUFFERS * EDRV_MAX_FRAME_SIZE)
#define EDRV_RX_DESCS_SIZE              (EDRV_MAX_RX_DESCRIPTOR * sizeof(tEdrvAdvRxDesc))

// Rx Descriptor Defines
#define EDRV_RDESC_STATUS_EOP 			(1 << 1)
#define EDRV_RDESC_STATUS_DD			(1 << 0)
#define EDRV_RDESC_ERRORS_RXE			(1 << 31)


//Rx Register defines
#define EDRV_RCTL_EN			(1 << 1 )
#define EDRV_RCTL_SBP			(1 << 2 ) // Store Bad Packets
#define EDRV_RCTL_UPE			(1 << 3 ) // Unicast Enable
#define EDRV_RCTL_MPE			(1 << 4 ) // Multicast Enable
#define EDRV_RCTL_LPE			(1 << 5 ) // Long packet Reception enable
#define EDRV_RCTL_LBM_MAC		(1 << 6 ) // Loop back mode
#define EDRV_RCTL_LBM_CLEAR		(3 << 6 )
#define EDRV_RCTL_SWFLUSH 		(1 << 26)
#define EDRV_RCTL_MO_SHIFT		12
#define EDRV_RCTL_MO_36_47		(0 << 12) // Multicast Offset bit[47:36]
#define EDRV_RCTL_MO_35_46		(1 << 12) // Multicast Offset bit[46:35]
#define EDRV_RCTL_MO_34_45		(1 << 13) // Multicast Offset bit[45:34]
#define EDRV_RCTL_MO_32_43		(3 << 12) // Multicast Offset bit[43:32]
#define EDRV_RCTL_BAM			(1 << 15) // Accept Broadcast packets
#define EDRV_RCTL_BSIZE_OFFSET	16		  // Buffer Size Offset (00:2048, 01:1024, 10:512, 11:256)
#define EDRV_RCTL_VFE			(1 << 16) // Enable Vlan Filter
#define EDRV_RCTL_PSP			(1 << 21) // Pad Small packets
#define EDRV_RCTL_SECRC			(1 << 26) // Strip CRC
#define EDRV_SRRCTL_DESCTYPE_ADV (1 << 25) // Advance Rx Descriptor one buffer
#define EDRV_SRRCTL_TIMESTAMP	(1 << 30) // Time stamp received packets
#define EDRV_SRRCTL_DROP_EN		(1 << 31) // drop packets if no descriptors available
#define EDRV_RXDCTL_PTHRESH     0
#define EDRV_RXDCTL_HTHRESH     0
#define EDRV_RXDCTL_WTHRESH     1
#define EDRV_RXDCTL_QUEUE_EN    (1 << 25)
#define EDRV_STAT_TPR           0x040D0
#define EDRV_STAT_GPRC			0x04074
#define EDRV_STAT_BPRC					0x04078
#define EDRV_STAT_MPRC					0x0407c


#define EDRV_GET_RX_DESC(pQueue , iIndex) (&(((tEdrvAdvRxDesc *)pQueue->m_pDescVirt)[iIndex]))
#define EDRV_GET_TX_DESC(pQueue , iIndex) (&(((tEdrvAdvTxDesc *)pQueue->m_pDescVirt)[iIndex]))
#define EDRV_GET_CTXT_DESC(pQueue , iIndex) (&(((tEdrvContextDesc *)pQueue->m_pDescVirt)[iIndex]))
#define EDRV_GET_TTX_DESC(pQueue, iIndex ) (&(((tEdrvTtxDesc *)pQueue->m_pDescVirt)[iIndex]))

#define EDRV_REGDW_READ(dwReg)          readl((unsigned char  *)EdrvInstance_l.m_pIoAddr + dwReg)
#define EDRV_REGDW_WRITE(dwReg, dwVal)  writel(dwVal, (unsigned char *)EdrvInstance_l.m_pIoAddr + dwReg)
#define EDRV_REGB_READ(dwReg)           readb(EdrvInstance_l.m_pIoAddr + dwReg)


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

#define EDRV_COUNT_SEND                 TGT_DBG_SIGNAL_TRACE_POINT(2)
#define EDRV_COUNT_TIMEOUT              TGT_DBG_SIGNAL_TRACE_POINT(3)
#define EDRV_COUNT_PCI_ERR              TGT_DBG_SIGNAL_TRACE_POINT(4)
#define EDRV_COUNT_TX                   TGT_DBG_SIGNAL_TRACE_POINT(5)
#define EDRV_COUNT_RX                   TGT_DBG_SIGNAL_TRACE_POINT(6)
#define EDRV_COUNT_LATECOLLISION        TGT_DBG_SIGNAL_TRACE_POINT(10)
#define EDRV_COUNT_TX_COL_RL            TGT_DBG_SIGNAL_TRACE_POINT(11)
#define EDRV_COUNT_TX_FUN               TGT_DBG_SIGNAL_TRACE_POINT(12)
#define EDRV_COUNT_TX_TEST              TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_ERR_CRC           TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_ERR_MULT          TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_ERR_SEQ           TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_ERR_OTHER         TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_ORUN              TGT_DBG_SIGNAL_TRACE_POINT(18)


#define EDRV_TRACE_CAPR(x)              TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x06000000)
#define EDRV_TRACE_RX_CRC(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0E000000)
#define EDRV_TRACE_RX_ERR(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0F000000)
#define EDRV_TRACE_RX_PUN(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x11000000)
#define EDRV_TRACE(x)                   TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF0000) | 0x0000FEC0)
//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    __le64                 m_le_qwBufferAddr;
    __le32                 m_le_dwLengthCmd;
    //u8                  m_bChecksumOffset;
   // u8                  m_bCmd;
   __le32                  m_le_dwStatus;
   // u8                  m_bReserved;
   // u16                 m_bVlan;
}tEdrvTxDesc;

typedef union {
	struct {
		__le64 m_le_qwBufferAddr;    /* Address of descriptor's data buf */
		__le32 m_dwCmdTypeLen;
		__le32 m_dwStatusIdxPaylen;
	} m_sRead;
	struct {
		__le64 m_le_qwTimeStamp;       /* Reserved */
		__le32 m_le_dwRsvd;
		__le32 m_le_dwstatus;
	} m_sWb;
}tEdrvAdvTxDesc;

typedef struct {
		__le32 m_dwIpMaclenVlan;
		__le32 m_dwLaunchTime;
		__le32 m_dwTucmdType;
		__le32 m_dwIdxL4lenMss;
}tEdrvContextDesc;

typedef struct{
	tEdrvContextDesc	m_CtxtDesc;
	tEdrvAdvTxDesc		m_Adv_desc;
}tEdrvTtxDesc;

typedef union
{
	struct{
		__le64                 m_le_qwBufferAddr;
		__le64                 m_le_qwHeaderAddr;
		}sRead;

	struct{
			__u16			   m_wRssPktType;
			__u16			   m_wHeaderLen;
			__u32			   m_dwRssHash;
			__u32			   m_dwExtStatusError;
			__u32			   m_dwLenVlanTag;
	}sWb;
}tEdrvAdvRxDesc;
// Structure for Queue Vector
typedef struct{
	unsigned int		m_uiQueueIdx;
	unsigned int		m_uiVector;
	//TODO:Add Call back for this vector here

	//struct tEdrvQueue *		m_pTxQueue;
	//struct tEdrvQueue *		m_pRxQueue;
	char 					m_strName[25];
}tEdrvQVector;

typedef struct
{
	tEdrvAdvTxDesc *Desc;
	u64				m_qwLaunchTime;
	u64				m_ActualTime;

}tEdrvTestpacket;

typedef struct
{
	dma_addr_t		m_DmaAddr;
	BYTE 		   *m_VirtAddr;
	unsigned int	m_uilen;

}tEdrvPktBuff;

typedef struct
{
	tEdrvQVector		*m_Qvector;
	unsigned int		m_iIndex;
	dma_addr_t          m_DescDma;
	tEdrvTxBuffer*      m_apTxBuffer[EDRV_MAX_TX_DESCRIPTOR];
	/*union
	{
		tEdrvTxDesc         *m_pTxDesc;
		tEdrvAdvTxDesc		*m_pTxAdvDesc;
		tEdrvContextDesc	*m_pTxCxtDesc;
		// TODO: Add Rx Desc declaration here
	};*/
	tEdrvTestpacket		*m_pTestPacket;
	void                *m_pDescVirt;	// pointer to Tx descriptors
	void __iomem 		*m_pQueueTail;
	unsigned char __iomem        *m_pbBuf;
	tEdrvPktBuff		*m_PktBuff;
	int					m_iNextDesc;
	int					m_iNextWb;
}tEdrvQueue;

typedef struct
{
    struct pci_dev      *m_pPciDev;      // pointer to PCI device structure
    void                *m_pIoAddr;      // pointer to register space of Ethernet controller
    //unsigned char __iomem        *m_pbTxBuf[EDRV_MAX_TX_QUEUES];      // pointer to Tx buffer
   // unsigned char __iomem        *m_pbRxBuf[EDRV_MAX_RX_QUEUES];      // pointer to Tx buffer
    dma_addr_t          m_TxBuffDma;
    tEdrvQueue *		m_pTxQueue[EDRV_MAX_TX_QUEUES];
    tEdrvQueue *		m_pRxQueue[EDRV_MAX_RX_QUEUES];
       // Virtual address for Desc
    dma_addr_t          m_TxDescDma;   // Dma phy Address
    dma_addr_t          m_RxDescDma;	//
    void                *m_pTxTail;
    int                 m_iIrq;
    BYTE*               m_pbTxBuf;
    BOOL                m_afTxBufUsed[EDRV_MAX_TX_BUFFERS];
    //tEdrvPktBuff		*m_PktBuff;
    unsigned int		m_TxMaxQueue;
    unsigned int		m_RxMaxQueue;
    unsigned int		m_NumQVectors;
    struct msix_entry   *m_pMsixEntry;
#ifdef USE_MULTIPLE_QUEUE
    tEdrvQVector *		m_pQvector[EDRV_MAX_QUEUE_VECTOR];
#endif
    tEdrvInitParam      m_InitParam;
}tEdrvInstance;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static int EdrvInitOne(struct pci_dev *pPciDev,
                       const struct pci_device_id *pId);

static void EdrvRemoveOne(struct pci_dev *pPciDev);

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static struct pci_device_id aEdrvPciTbl[]={
  {0x8086, 0x1533, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, //I210
  {0, }
};

MODULE_DEVICE_TABLE (pci, aEdrvPciTbl);

  static struct pci_driver EdrvDriver = {
      .name         = DRIVER_NAME,
      .id_table     = aEdrvPciTbl,
      .probe        = EdrvInitOne,
      .remove       = EdrvRemoveOne,
  };

tEdrvInstance EdrvInstance_l;

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <edrv>                                              */
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
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//
// Function:    EdrvInit
//
// Description: function for init of the Ethernet controller
//
// Parameters:  pEdrvInitParam_p    = pointer to struct including the init-parameters
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvInit(tEdrvInitParam * pEdrvInitParam_p)
{
	tEplKernel  Ret;
	int         iResult;
	int         iIndex;

	Ret = kEplSuccessful;

	// clear instance structure
	EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

	if(NULL != pEdrvInitParam_p)
	{
		// save the init data
		EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;
	}


	// clear driver structure
	EPL_MEMSET(&EdrvDriver, 0, sizeof (EdrvDriver));

	EdrvDriver.name         = DRIVER_NAME,
	EdrvDriver.id_table     = aEdrvPciTbl,
	EdrvDriver.probe        = EdrvInitOne,
	EdrvDriver.remove       = EdrvRemoveOne,

	printk("Registering Driver.....");
	iResult = pci_register_driver (&EdrvDriver);
	if(0 != iResult)
	{
		printk("%s pci_register_driver failed with %d\n", __FUNCTION__, iResult);
		Ret = EdrvShutdown();
	    Ret = kEplNoResource;
	    goto Exit;
	}
	printk("Done!!!\n");

	// local MAC address might have been changed in EdrvInitOne
	EPL_MEMCPY(pEdrvInitParam_p->m_abMyMacAddr, EdrvInstance_l.m_InitParam.m_abMyMacAddr, 6);

	printk("%s local MAC = ", __FUNCTION__);
	for (iIndex = 0; iIndex < 6; iIndex++)
	{
	     printk("%02X ", (unsigned int)pEdrvInitParam_p->m_abMyMacAddr[iIndex]);
	}
	printk("\n");

Exit:
	 return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvShutdown
//
// Description: Shutdown the Ethernet controller
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvShutdown(void)
{
	// unregister PCI driver
	printk("%s calling pci_unregister_driver()\n", __FUNCTION__);
	pci_unregister_driver (&EdrvDriver);

	return kEplSuccessful;
}
static inline struct device *pci_dev_to_dev(struct pci_dev *pdev)
{
	return &pdev->dev;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvDefineRxMacAddrEntry
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to set
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvDefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
	tEplKernel      Ret = kEplSuccessful;
	printk("%s\n",__FUNCTION__);
	    return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvUndefineRxMacAddrEntry
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to reset
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
	tEplKernel      Ret = kEplSuccessful;
	printk("%s\n",__FUNCTION__);
		    return Ret;
}
tEplKernel EdrvChangeFilter(tEdrvFilter*    pFilter_p,
                            unsigned int    uiCount_p,
                            unsigned int    uiEntryChanged_p,
                            unsigned int    uiChangeFlags_p)
{
tEplKernel      Ret = kEplSuccessful;

    return Ret;
}

void EdrvSetGpio(INT iPinNo)
{
	DWORD dwReg;

	dwReg = 0;

	switch(iPinNo)
	{
	case 0:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
		dwReg &= ~SDP0_SET_HIGH;
		dwReg |= SDP0_SET_DIR_OUT;
		EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);
		break;
	case 1:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
		dwReg &= ~SDP1_SET_HIGH;
		dwReg |= SDP1_SET_DIR_OUT;
		EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);
		break;
	case 2:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
		dwReg &= ~SDP2_SET_HIGH;
		dwReg |= SDP2_SET_DIR_OUT;
		EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG,dwReg);
		break;
	case 3:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
		dwReg &= ~SDP3_SET_HIGH;
		dwReg |= SDP3_SET_DIR_OUT;
		EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG,dwReg);
		break;
	}

}
void EdrvClearGpio(INT iPinNo)
{
	DWORD dwReg;

	dwReg = 0;

	switch(iPinNo)
	{
	case 0:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
		dwReg |= SDP0_SET_HIGH;
		EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);
		break;
	case 1:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
		dwReg |= SDP1_SET_HIGH;
		EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);
		break;
	case 2:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
		dwReg |= SDP2_SET_HIGH;
		EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG,dwReg);
		break;
	case 3:
		dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
		dwReg |= SDP3_SET_HIGH;
		EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG,dwReg);
		break;
	}

}

//---------------------------------------------------------------------------
//
// Function:    EdrvAllocTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//                          = kEplEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvAllocTxMsgBuffer       (tEdrvTxBuffer * pBuffer_p)
{
	tEplKernel Ret = kEplSuccessful;
	DWORD	dwChannel;

	if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
	{
	     Ret = kEplEdrvNoFreeBufEntry;
	     goto Exit;
	}
//printk("%s\n",__FUNCTION__);
	if (EdrvInstance_l.m_pbTxBuf == NULL)
	{
        printk("%s Tx buffers currently not allocated\n", __FUNCTION__);
	    Ret = kEplEdrvNoFreeBufEntry;
	    goto Exit;
	}

	for (dwChannel = 0; dwChannel < EDRV_MAX_TX_BUFFERS; dwChannel++)
	{
	    if (EdrvInstance_l.m_afTxBufUsed[dwChannel] == FALSE)
	    {
	        // free channel found
	        EdrvInstance_l.m_afTxBufUsed[dwChannel] = TRUE;
	        pBuffer_p->m_BufferNumber.m_dwVal = dwChannel;
	        pBuffer_p->m_pbBuffer = EdrvInstance_l.m_pbTxBuf + (dwChannel * EDRV_MAX_FRAME_SIZE);
	        pBuffer_p->m_uiMaxBufferLen = EDRV_MAX_FRAME_SIZE;
	        break;
	    }
	}
	//printk("Nu:%d tBuff %p buff:%p\n",pBuffer_p->m_BufferNumber.m_dwVal,pBuffer_p,pBuffer_p->m_pbBuffer);
	if (dwChannel >= EDRV_MAX_TX_BUFFERS)
	{
	    Ret = kEplEdrvNoFreeBufEntry;
	    goto Exit;
	}

Exit:
	 return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EdrvReleaseTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvReleaseTxMsgBuffer     (tEdrvTxBuffer * pBuffer_p)
{
	unsigned int uiBufferNumber;
	//printk("%s\n",__FUNCTION__);
	uiBufferNumber = pBuffer_p->m_BufferNumber.m_dwVal;

	if (uiBufferNumber < EDRV_MAX_TX_BUFFERS)
	{
	    EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] = FALSE;
	}


  return kEplSuccessful;
}
#ifdef QAV_MODE
int
EdrvGetTxTimeStamp( __u64	*pqwCurtime_p)
{
	__u64	t0, t1;
	DWORD	timh, timl;

	if (NULL == pqwCurtime_p) return -1;

	/* sample the timestamp bracketed by the RDTSC */
	//rdtscll(t0);
	//EDRV_REGDW_WRITE(EDRV_TSAUXC, EDRV_TSAUXC_SAMP_AUTO);
	//rdtscll(t1);

	timl = EDRV_REGDW_READ(0x0B618);
	timh = EDRV_REGDW_READ(0x0B61C);

	*pqwCurtime_p = (__u64)timh * 1000000000 + (__u64)timl;


	return(0);
}
#endif
#ifdef QAV_MODE
int
EdrvGetMacClock( __u64	*pqwCurtime_p)
{
//	__u64	t0, t1;
	DWORD	timh, timl;

	if (NULL == pqwCurtime_p) return -1;

	/* sample the timestamp bracketed by the RDTSC */
//	rdtscll(t0);
	EDRV_REGDW_WRITE(EDRV_TSAUXC, EDRV_TSAUXC_SAMP_AUTO);
//	rdtscll(t1);

	timl = EDRV_REGDW_READ(EDRV_AUXSTMPL0);
	timh = EDRV_REGDW_READ(EDRV_AUXSTMPH0);

	*pqwCurtime_p = (__u64)timh * 1000000000 + (__u64)timl;


	return(0);
}
#endif
//---------------------------------------------------------------------------
//
// Function:    EdrvSendTxMsg
//
// Description: immediately starts the transmission of the buffer
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvSendTxMsg              (tEdrvTxBuffer * pTxBuffer_p)
{
	tEplKernel      Ret = kEplSuccessful;

	unsigned int    	uiBufferNumber;
	tEdrvAdvTxDesc		*pTxAdvDesc;
	tEdrvQueue			*pTxQueue;
	INT					iQueue = 0,iIndex = 0;
	dma_addr_t 			TxDma;
#ifdef QAV_MODE
	tEdrvTtxDesc		*pTtxDesc;
	QWORD				qwCurtime;
	QWORD				qwLaunchTime,qwTime;
#endif
	uiBufferNumber = pTxBuffer_p->m_BufferNumber.m_dwVal;

	//printk("Send Msg\n");
	if ((uiBufferNumber >= EDRV_MAX_TX_BUFFERS)
	        || (EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] == FALSE))
	{
			printk("No Buff\n");
	        Ret = kEplEdrvBufNotExisting;

	        goto Exit;
	}
	//printk("No:%d tBuff %p buff:%p\n",uiBufferNumber,pTxBuffer_p,pTxBuffer_p->m_pbBuffer);
	//printk("tBuff %p \n",pTxBuffer_p);
#ifdef USE_MULTIPLE_QUEUE
	//TODO: Assign queue based on Packet Type
#else
	pTxQueue = EdrvInstance_l.m_pTxQueue[iQueue];
	//printk("{%s} W:%d N:%d\n",__FUNCTION__,pTxQueue->m_iNextWb,pTxQueue->m_iNextDesc);
	iIndex = pTxQueue->m_iNextDesc;

#ifdef QAV_MODE
	if(((iIndex + 1) & EDRV_MAX_TTX_DESC_LEN) == pTxQueue->m_iNextWb)
	{
			printk("No free Desc\n");
				Ret = kEplEdrvNoFreeTxDesc;
				goto Exit;
	}

	pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue,iIndex);

	pTtxDesc->m_CtxtDesc.m_dwIdxL4lenMss = 0;
	pTtxDesc->m_CtxtDesc.m_dwIpMaclenVlan = 0;
	//EdrvGetWallClock(&qwCurtime);

	qwLaunchTime = pTxBuffer_p->m_qwLaunchTime;
	//printk("Lt: %lld\n",qwLaunchTime);
	do_div(qwLaunchTime,1000000000);
	qwTime = pTxBuffer_p->m_qwLaunchTime - (qwLaunchTime * 1000000000);

	//pTxBuffer_p->m_qwLaunchTime = qwTime;
	//wmb();
	//printk("Qt: %lld\n",qwTime);
	do_div(qwTime,32);

	pTtxDesc->m_CtxtDesc.m_dwLaunchTime = qwTime;
	//pTtxDesc->m_CtxtDesc.m_dwLaunchTime = (qwTime >> 5);

	pTtxDesc->m_CtxtDesc.m_dwTucmdType = (EDRV_TDESC_CMD_DEXT | EDRV_TDESC_DTYP_CTXT) ;

	TxDma = dma_map_single(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),\
		   	   	   	  pTxBuffer_p->m_pbBuffer,\
		   	   	   		pTxBuffer_p->m_uiTxMsgLen,DMA_TO_DEVICE );

	if (dma_mapping_error(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),TxDma))
	{
	   	//TODO: Assign Error Here
		Ret = kEplEdrvNoFreeBufEntry;
	    goto Exit;
	}

//	if(pTxBuffer_p->m_pbBuffer[20] == 1)
//	{
//		TgtDbgSignalTracePoint(25);
//	}
	pTxQueue->m_apTxBuffer[iIndex] = pTxBuffer_p;
	EDRV_COUNT_SEND;
	pTxQueue->m_PktBuff[iIndex].m_DmaAddr = TxDma;
	pTxQueue->m_PktBuff[iIndex].m_VirtAddr = pTxBuffer_p->m_pbBuffer;
	pTxQueue->m_PktBuff[iIndex].m_uilen = pTxBuffer_p->m_uiTxMsgLen;
	//mb();
	pTtxDesc->m_Adv_desc.m_sRead.m_le_qwBufferAddr = (__le64)TxDma;
	pTtxDesc->m_Adv_desc.m_sRead.m_dwCmdTypeLen = (unsigned int) pTxBuffer_p->m_uiTxMsgLen;
	pTtxDesc->m_Adv_desc.m_sRead.m_dwCmdTypeLen |= ( EDRV_TDESC_CMD_DEXT | \
											EDRV_TDESC_DTYP_ADV | \
					   	   	   	   	   	   	EDRV_TDESC_CMD_EOP | \
					   	   	   	   	   	   	EDRV_TDESC_CMD_IFCS |\
					   	   	   	   	   	   	EDRV_TDESC_CMD_RS) ;
	pTtxDesc->m_Adv_desc.m_sRead.m_dwStatusIdxPaylen = (pTxBuffer_p->m_uiTxMsgLen << 14);
	//	pTxAdvDesc->m_sRead.m_dwStatusIdxPaylen |= (1 << 1);

		iIndex =  ((iIndex + 1) & EDRV_MAX_TTX_DESC_LEN);
		// increment Tx descriptor queue tail pointer
		pTxQueue->m_iNextDesc = iIndex;
		// Transmit the packet
		EDRV_REGDW_WRITE(EDRV_TDTAIL(iQueue),(iIndex * 2) );
		//mmiowb();

#else
		if(((iIndex + 1) & EDRV_MAX_TX_DESC_LEN) == pTxQueue->m_iNextWb)
		{
			printk("No free Desc\n");
			Ret = kEplEdrvNoFreeTxDesc;
			goto Exit;
		}
		pTxAdvDesc = EDRV_GET_TX_DESC(pTxQueue,iIndex);

			TxDma = dma_map_single(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),\
			   	   	   	  pTxBuffer_p->m_pbBuffer,\
			   	   	   		pTxBuffer_p->m_uiTxMsgLen,DMA_TO_DEVICE );

			if (dma_mapping_error(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),TxDma))
			{
			   	//TODO: Assign Error Here
				Ret = kEplEdrvNoFreeBufEntry;
			    goto Exit;
			}

			if(pTxBuffer_p->m_pbBuffer[14] == 1)
				{
					//TgtDbgSignalTracePoint(25);
				}
			pTxQueue->m_apTxBuffer[iIndex] = pTxBuffer_p;
			EDRV_COUNT_SEND;
			pTxQueue->m_PktBuff[iIndex].m_DmaAddr = TxDma;
			pTxQueue->m_PktBuff[iIndex].m_VirtAddr = pTxBuffer_p->m_pbBuffer;
			pTxQueue->m_PktBuff[iIndex].m_uilen = pTxBuffer_p->m_uiTxMsgLen;

			pTxAdvDesc->m_sRead.m_le_qwBufferAddr = (__le64)TxDma;
			pTxAdvDesc->m_sRead.m_dwCmdTypeLen = (unsigned int) pTxBuffer_p->m_uiTxMsgLen;
			pTxAdvDesc->m_sRead.m_dwCmdTypeLen |= ( EDRV_TDESC_CMD_DEXT | \
						   	   	   	   	   	   	   	   	   EDRV_TDESC_DTYP_ADV | \
						   	   	   	   	   	   	   	   	   EDRV_TDESC_CMD_EOP | \
						   	   	   	   	   	   	   	   	   EDRV_TDESC_CMD_IFCS |\
						   	   	   	   	   	   	   	   	   EDRV_TDESC_CMD_RS) ;
			pTxAdvDesc->m_sRead.m_dwStatusIdxPaylen = (pTxBuffer_p->m_uiTxMsgLen << 14);
		//	pTxAdvDesc->m_sRead.m_dwStatusIdxPaylen |= (1 << 1);

			iIndex =  ((iIndex + 1) & EDRV_MAX_TX_DESC_LEN);
			// increment Tx descriptor queue tail pointer
			pTxQueue->m_iNextDesc = iIndex;
			// Transmit the packet
			EDRV_REGDW_WRITE(EDRV_TDTAIL(iQueue),iIndex);
		//	mmiowb();
#endif



#endif

Exit:
	return Ret;

}
//---------------------------------------------------------------------------
//
// Function:    EdrvReleaseRxBuffer
//
// Description: Release a RxBuffer to the Edrv
//
// Parameters:  pbRxBuffer_p    = Pointer to buffer
//
// Returns:     None
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvReleaseRxBuffer(tEdrvRxBuffer* pRxBuffer_p)
{
	tEplKernel      Ret = kEplSuccessful;
	printk("%s\n",__FUNCTION__);
	return Ret;
}
//---------------------------------------------------------------------------
//
// Function:     EdrvInterruptHandler
//
// Description:  interrupt handler
//
// Parameters:   void
//
// Returns:      void
//
// State:
//
//---------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static irqreturn_t TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p)
#else
static int TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p)
#endif
{
	DWORD           dwStatus;
	int             iHandled;
	INT				iIndex;
	int				Ret;
	tEdrvQueue *pTxQueue = EdrvInstance_l.m_pTxQueue[0];
	tEdrvQueue *pRxQueue = EdrvInstance_l.m_pRxQueue[0];

	iHandled = IRQ_HANDLED;

	// Acknowledge the Interrupt
	//dwStatus = 0;
	//dwStatus = EDRV_REGDW_READ(EDRV_EXT_INTR_REG);
	//EDRV_REGDW_WRITE(EDRV_EXT_INTR_REG,dwStatus);

	// Read the interrupt status
	dwStatus = EDRV_REGDW_READ(EDRV_INTR_READ_REG);
	//printk("ICR :%x\n",dwStatus);
	if ((dwStatus & EDRV_INTR_ICR_MASK_DEF) == 0)
	{
	    iHandled = IRQ_NONE;
	    EDRV_COUNT_PCI_ERR;
	    goto Exit;
	}
	//EDRV_REGDW_WRITE(EDRV_INTR_READ_REG,dwStatus);

	//Process Rx
	if ((dwStatus & ( EDRV_INTR_ICR_RXDW | EDRV_INTR_ICR_RXDMT0)) != 0)
	{
		tEdrvAdvRxDesc*    pAdvRxDesc;
		tEdrvRxBuffer      RxBuffer;
		iIndex = pRxQueue->m_iNextWb;
		//printk("RX\n");
		pAdvRxDesc = EDRV_GET_RX_DESC(pRxQueue,iIndex);

		while((pAdvRxDesc->sWb.m_dwExtStatusError & EDRV_RDESC_STATUS_DD) && \
				(pAdvRxDesc->sWb.m_dwExtStatusError & EDRV_RDESC_STATUS_EOP))
		{
			UINT					uiRcvLen;
			if(pAdvRxDesc->sWb.m_dwExtStatusError & EDRV_RDESC_ERRORS_RXE)
			{
				EDRV_COUNT_RX_ERR_CRC;
				//printk("R1\n");
			}
			else
			{
				//printk("Wb:%d\n",iIndex);
				//EdrvSetGpio(2);
				//printk("R\n");
				// good packet
				//printk("T:%d H:%d\n",EDRV_REGDW_READ(EDRV_RDTAIL(pRxQueue->m_iIndex)),EDRV_REGDW_READ(EDRV_RDTAIL(pRxQueue->m_iIndex)));
				RxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
				uiRcvLen = (0x0000FFFF & pAdvRxDesc->sWb.m_dwLenVlanTag);
				RxBuffer.m_uiRxMsgLen = uiRcvLen;
				//printk("RXLen : %d\n",uiRcvLen);
				RxBuffer.m_pbBuffer = (BYTE *)pRxQueue->m_PktBuff[iIndex].m_VirtAddr;
				EDRV_COUNT_RX;

				dma_sync_single_for_cpu(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),\
										pRxQueue->m_PktBuff[iIndex].m_DmaAddr,\
										uiRcvLen,\
										DMA_FROM_DEVICE);

				//printk("%x\n",RxBuffer.m_pbBuffer[14]);
				// Forward the Rcv packet to DLL
				if(NULL != EdrvInstance_l.m_InitParam.m_pfnRxHandler)
				{
					Ret = EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);
				//	printk("Ret:%d\n",Ret);
				}
				else
				{
					printk("[EPL]:No Rx CallBack Registered\n");
				}

				// Prepare descriptor for next Rcv
				pAdvRxDesc->sRead.m_le_qwBufferAddr = (__le64)pRxQueue->m_PktBuff[iIndex].m_DmaAddr;
				pAdvRxDesc->sRead.m_le_qwHeaderAddr = 0;
				//pAdvRxDesc->sWb.m_dwExtStatusError = 0;
				EDRV_REGDW_WRITE(EDRV_RDTAIL(pRxQueue->m_iIndex),iIndex);
				iIndex =((iIndex + 1) & EDRV_MAX_RX_DESC_LEN);
				pRxQueue->m_iNextWb = iIndex;

				pAdvRxDesc = EDRV_GET_RX_DESC(pRxQueue,iIndex);
				break;
				//EdrvClearGpio(2);
			}
		}
	}

	//Process Tx
	if ((dwStatus & (EDRV_INTR_ICR_TXDW)) != 0)
	{
		EDRV_COUNT_TX;
		//printk("Tx \n");

		while(pTxQueue->m_iNextWb != pTxQueue->m_iNextDesc)
		{
			#ifdef QAV_MODE
			tEdrvTtxDesc		*pTtxDesc;
			#endif
			tEdrvAdvTxDesc*    pAdvTxDesc;

			EDRV_COUNT_TX_TEST;
			iIndex = 0;
//#ifdef QAV_MODE
//			iIndex = ((pTxQueue->m_iNextWb + 1) & EDRV_MAX_TX_DESC_LEN);
//			printk("W:%d N:%d\n",pTxQueue->m_iNextWb,pTxQueue->m_iNextDesc);
//#else
			iIndex = pTxQueue->m_iNextWb;
			//printk("W:%d N:%d\n",pTxQueue->m_iNextWb,pTxQueue->m_iNextDesc);
//#endif
			#ifdef QAV_MODE
			pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue,iIndex);
			pAdvTxDesc = &(pTtxDesc->m_Adv_desc);
			#else
			pAdvTxDesc = EDRV_GET_TX_DESC(pTxQueue,iIndex);
			#endif

			if(pAdvTxDesc->m_sWb.m_le_dwstatus & EDRV_TDESC_STATUS_DD)
			{
				//printk("Process Tx\n");
				// Process the send packet
				tEdrvTxBuffer*  pTxBuffer;
				DWORD           dwTxStatus;
				QWORD 			Lat,Sec;
				//printk("T\n");
				dwTxStatus = pAdvTxDesc->m_sWb.m_le_dwstatus;
				pAdvTxDesc->m_sWb.m_le_dwstatus = 0;
				pTxBuffer = pTxQueue->m_apTxBuffer[iIndex];
				pTxQueue->m_apTxBuffer[iIndex] = NULL;

			//	if(pTxBuffer->m_pbBuffer[20] == 1)
			//	{
			//		TgtDbgSignalTracePoint(26);
			//	}
							//	Sec = (0xFFFFFFFF & pAdvTxDesc->m_sWb.m_le_qwTimeStamp);
							//	Lat = Sec - pTxBuffer->m_qwLaunchTime ;
								//EdrvGetTxTimeStamp(&Lat);
							//	printk("Type :%d (%lld - %lld) %lld\n ",pTxBuffer->m_pbBuffer[14],Sec,pTxBuffer->m_qwLaunchTime,Lat);
								//printk("Type %d\n",pTxQueue->m_PktBuff[iIndex].m_VirtAddr[14]);
				//dma_unmap_single(&EdrvInstance_l.m_pPciDev->dev,\
								 pTxQueue->m_PktBuff[iIndex].m_DmaAddr,\
								 pTxQueue->m_PktBuff[iIndex].m_uilen, \
								 DMA_TO_DEVICE);
#ifdef QAV_MODE
				iIndex =((iIndex + 1) & EDRV_MAX_TTX_DESC_LEN);
#else
				iIndex =((iIndex + 1) & EDRV_MAX_TX_DESC_LEN);
#endif
								pTxQueue->m_iNextWb = iIndex;


								//EDRV_COUNT_TX;
								if (pTxBuffer != NULL)
								{
									 // Call Tx handler of Data link layer
									if (pTxBuffer->m_pfnTxHandler != NULL)
									{
										pTxBuffer->m_pfnTxHandler(pTxBuffer);
									}
								}
								else
								{
									EDRV_COUNT_TX_FUN;
								}
								//break;
				dma_unmap_single(&EdrvInstance_l.m_pPciDev->dev,\
								 pTxQueue->m_PktBuff[iIndex].m_DmaAddr,\
								 pTxQueue->m_PktBuff[iIndex].m_uilen, \
								 DMA_TO_DEVICE);
			}
			else
			{
				// no completed Packet to process
				//printk("No packet\n");
				break;
			}
		}
	}

Exit:
	return iHandled;
}
//---------------------------------------------------------------------------
//
// Function:     EdrvPutHwSemaphoreGeneric
//
// Description:  Release Hardware semaphore
//
// Parameters:   void
//
// Returns:      void
//
// State:
//
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
/**
\brief EdrvPutHwSemaphoreGeneric
	   Release the semaphore
\param void

\return void

*/
//------------------------------------------------------------------------------
void EdrvPutHwSemaphoreGeneric(void)
{
	DWORD dwSwsm;

	dwSwsm = EDRV_REGDW_READ(EDRV_SWSM_REG);

	dwSwsm &= ~(EDRV_SWSM_SMBI | EDRV_SWSM_SWESMBI);

	EDRV_REGDW_WRITE(EDRV_SWSM_REG, dwSwsm);
}
//------------------------------------------------------------------------------
/**
\brief EdrvGetHwSemaphore
	   Acquire the HW semaphore to access the PHY or NVM
\param void

\return void

*/
//------------------------------------------------------------------------------
INT EdrvGetHwSemaphore(void)
{
	DWORD dwSwsm;
	INT iResult = 0;
	INT iTimeout = 100;
	INT iIndex = 0;

	/* Get the FW semaphore. */
	for (iIndex = 0; iIndex < iTimeout; iIndex++) {
		dwSwsm = EDRV_REGDW_READ(EDRV_SWSM_REG);
		EDRV_REGDW_WRITE(EDRV_SWSM_REG, dwSwsm | EDRV_SWSM_SWESMBI);

		/* Semaphore acquired if bit latched */
		if (EDRV_REGDW_READ(EDRV_SWSM_REG) & EDRV_SWSM_SWESMBI)
			break;

		udelay(50);
	}

	if (iIndex == iTimeout) {
		/* Release semaphores */
		EdrvPutHwSemaphoreGeneric();
		iResult = -1;
		goto Exit;
	}

Exit:
	return iResult;
}
//------------------------------------------------------------------------------
/**
\brief EdrvPutHwSemaphore
	   Release hardware semaphore used to access the PHY or NVM
\param void

\return void

*/
//------------------------------------------------------------------------------
void EdrvPutHwSemaphore(void)
{
	DWORD dwSwsm;

	dwSwsm = EDRV_REGDW_READ(EDRV_SWSM_REG);

	dwSwsm &= ~EDRV_SWSM_SWESMBI;

	EDRV_REGDW_WRITE(EDRV_SWSM_REG, dwSwsm);
}
//------------------------------------------------------------------------------
/**
\brief EdrvAcquireSwfwSync
	   Acquire the SW/FW semaphore to access the PHY or NVM.  The mask
  	   will also specify which port we're acquiring the lock for.
\param wMask_p Specifies which semaphore to acquire

\return INT
\retval 0 no error
\retval Non-Zero reject further processing

*/
//------------------------------------------------------------------------------
INT EdrvAcquireSwfwSync(WORD wMask_p)
{
	DWORD dwSwfwSync;
	DWORD dwSwmask = wMask_p;
	DWORD dwFwmask = wMask_p << 16;
	INT   iResult = 0;
	INT   iIndex = 0, iTimeout = 200;

	while (iIndex < iTimeout) {
		if (EdrvGetHwSemaphore()) {
			iResult = -13;
			goto Exit;
		}

		dwSwfwSync = EDRV_REGDW_READ(EDRV_SW_FW_SYNC);
		if (!(dwSwfwSync & dwFwmask))
			break;

		/*
		 * Firmware currently using resource (dwFwmask)
		 */
		EdrvPutHwSemaphore();
		mdelay(5);
		iIndex++;
	}

	if (iIndex == iTimeout) {
		printk("Driver can't access resource, SW_FW_SYNC timeout.\n");
		iResult = -13;
		goto Exit;
	}

	dwSwfwSync |= dwSwmask;
	EDRV_REGDW_WRITE(EDRV_SW_FW_SYNC, dwSwfwSync);

	EdrvPutHwSemaphore();

Exit:
	return iResult;
}
//------------------------------------------------------------------------------
/**
\brief EdrvReleaseSwfwSync
	   Release the SW/FW semaphore used to access the PHY or NVM.  The mask
   	   will also specify which port we're releasing the lock for.
\param wMask_p Specifies which semaphore to release

\return void

*/
//------------------------------------------------------------------------------
void EdrvReleaseSwfwSync(WORD wMask_p)
{
	DWORD dwSwfwSync;

	while (EdrvGetHwSemaphore() != 0)
		; /* Empty */

	dwSwfwSync = EDRV_REGDW_READ(EDRV_SW_FW_SYNC);
	dwSwfwSync &= ~wMask_p;
	EDRV_REGDW_WRITE(EDRV_SW_FW_SYNC, dwSwfwSync);

	EdrvPutHwSemaphore();
}
//------------------------------------------------------------------------------
/**
\brief EdrvPtpRead
	   Reads the current SYSTIM register time in struct timespec format
\param psTime_p timespec struct to hold the current time

\return DWORD
\retval Nano seconds part of current time

*/
//------------------------------------------------------------------------------
static DWORD EdrvPtpRead(struct timespec *psTime_p)
{
	DWORD dwSec, dwNsec, dwPsec;

	/*
	 * The timestamp latches on lowest register read. For I210/I211, the
	 * lowest register is SYSTIMR. Since we only need to provide nanosecond
	 * resolution, we can ignore it.
	 */
	dwPsec = EDRV_REGDW_READ(EDRV_SYSTIMR_REG);
	dwNsec = EDRV_REGDW_READ(EDRV_SYSTIML_REG);
	dwSec = EDRV_REGDW_READ(EDRV_SYSTIMH_REG);

	psTime_p->tv_sec = dwSec;
	psTime_p->tv_nsec = dwNsec;
	return dwNsec;
}
//------------------------------------------------------------------------------
/**
\brief EdrvPtpWrite
	   Writes a value into SYSTIM register
\param psTime_p timespec struct having the time to write

\return void

*/
//------------------------------------------------------------------------------
static void EdrvPtpWrite(const struct timespec *psTime_p)
{
	/*
	 * Writing the SYSTIMR register is not necessary as it only provides
	 * sub-nanosecond resolution.
	 */
	EDRV_REGDW_WRITE(EDRV_SYSTIML_REG, psTime_p->tv_nsec);
	EDRV_REGDW_WRITE(EDRV_SYSTIMH_REG, psTime_p->tv_sec);
}
//---------------------------------------------------------------------------
//
// Function:    EdrvMacPsWrite
//
// Description: initializes one PCI device
//
// Parameters:  pPciDev             = pointer to corresponding PCI device structure
//              pId                 = PCI device ID
//
// Returns:     (int)               = error code
//
// State:
//
//---------------------------------------------------------------------------
static int EdrvMdicWrite(unsigned int iPhyreg_p, unsigned short wValue_p)
{
	unsigned int	dwRegVal = 0;
	unsigned int  dwMdicRd;


	dwRegVal &= ~(EDRV_MDIC_DATA_MASK | EDRV_MDIC_REGADD_MASK);
	dwRegVal |= wValue_p;
  dwRegVal |= (iPhyreg_p << 16);
	dwRegVal |= EDRV_MDIC_OP_WRITE;
	dwRegVal |= EDRV_MDIC_INTR_DIS;
	dwRegVal &= ~EDRV_MDIC_RD;
  printk("EdrvMdicWrite %x\n",dwRegVal);
	EDRV_REGDW_WRITE(EDRV_MDIC_REG,dwRegVal);
	// wait for completion of transfer
	do
	{
    //cpu_relax();
    udelay(50);
		dwMdicRd = EDRV_REGDW_READ(EDRV_MDIC_REG);

	}while((dwMdicRd & EDRV_MDIC_RD ) == 0); //wait PHYIDLE is set 1

	return 0;
}
//---------------------------------------------------------------------------
//
// Function:   EdrvMdicRead
//
// Description: read from PHY registers
//
// Parameters:  iMiiId_p    = PHY address
//              iPhyreg_p   = PHY register
//
// Returns:     (int)      = error code
//
// State:
//
//---------------------------------------------------------------------------
static unsigned short EdrvMdicRead(int iPhyreg_p)
{

	unsigned int		dwRegVal = 0;
	unsigned short	wValue = 0;
	unsigned int  dwMdicRd;
	dwRegVal &= ~(EDRV_MDIC_DATA_MASK | EDRV_MDIC_REGADD_MASK);
	dwRegVal |= (iPhyreg_p << 16);
	dwRegVal |= EDRV_MDIC_OP_READ;
	dwRegVal |= EDRV_MDIC_INTR_DIS;
	dwRegVal &= ~EDRV_MDIC_RD;
	printk("EdrvMdicRead %x\n",dwRegVal);
	EDRV_REGDW_WRITE(EDRV_MDIC_REG,dwRegVal);


	// wait till the value is being read

	do
	{
	   cpu_relax();
	   dwMdicRd = EDRV_REGDW_READ(EDRV_MDIC_REG);

	}while((dwMdicRd & EDRV_MDIC_RD ) == 0); //wait RD is set 1

	wValue = (dwMdicRd & EDRV_MDIC_DATA_MASK);

	return wValue;

}
//------------------------------------------------------------------------------
/**
\brief EdrvFreeTxQueueBuffer
	   Free the buffers for queue
\param pTxQueue_p Queue to be processed

\return void

*/
//------------------------------------------------------------------------------
static void EdrvFreeTxQueueBuffer(tEdrvQueue *pTxQueue_p)
{
	int iIndex;
	for(iIndex = 0 ; iIndex < EDRV_MAX_RX_DESCRIPTOR ; iIndex++)
	{
	    // set report status for all descriptor
		if(pTxQueue_p->m_PktBuff[iIndex].m_DmaAddr)
		{
			dma_unmap_single(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),\
						 pTxQueue_p->m_PktBuff[iIndex].m_DmaAddr,\
						 pTxQueue_p->m_PktBuff[iIndex].m_uilen,DMA_TO_DEVICE );
		}

	}
}
//------------------------------------------------------------------------------
/**
\brief EdrvFreeTxBuffers
	   Free all Transmit buffers
\param 	void
\return void

*/
//------------------------------------------------------------------------------
static void EdrvFreeTxBuffers(void)
{
	//tEdrvQueue *TxQueue;
	int iIndex;
	for(iIndex = 0; iIndex < EdrvInstance_l.m_TxMaxQueue; iIndex++ )
	{
		EdrvFreeTxQueueBuffer(EdrvInstance_l.m_pTxQueue[iIndex]);
	}
}
//------------------------------------------------------------------------------
/**
\brief EdrvFreeTxQueues
	   Free the memory for Tx queues
\param void

\return void

*/
//------------------------------------------------------------------------------
static void EdrvFreeTxQueues(void)
{
	tEdrvQueue *pTxQueue;
	int iIndex;

	for(iIndex = 0; iIndex < EdrvInstance_l.m_TxMaxQueue ;iIndex++)
	{
		pTxQueue = EdrvInstance_l.m_pTxQueue[iIndex];

		if(NULL == pTxQueue)
		{
			break;
		}
		if(NULL != pTxQueue->m_pbBuf)
		{
			kfree(pTxQueue->m_pbBuf);
		}

		if(NULL != pTxQueue->m_PktBuff)
		{
			kfree(pTxQueue->m_PktBuff);
		}


		if (NULL != pTxQueue->m_pDescVirt)
		{
			 dma_free_coherent(&EdrvInstance_l.m_pPciDev->dev,EDRV_TX_DESCS_SIZE,pTxQueue->m_pDescVirt,pTxQueue->m_DescDma);
			 kfree(pTxQueue);
			 EdrvInstance_l.m_pTxQueue[iIndex] = NULL;

		}

	}

	EdrvInstance_l.m_TxMaxQueue = 0;

}
//------------------------------------------------------------------------------
/**
\brief EdrvInitTxQueue
	   Initialize the Tx queue with descriptors and memory resources
\param pTxQueue_p Pointer to the queue structure to initialize

\return tEplKernel Error code

*/
//------------------------------------------------------------------------------
static tEplKernel EdrvInitTxQueue(tEdrvQueue *pTxQueue_p)
{
	tEplKernel iResult = kEplSuccessful;
	INT iDescSize;

	// Allocate Tx Descriptor
	printk("{%s}:Allocating Tx Desc %p\n",__FUNCTION__,pTxQueue_p);
	iDescSize = ALIGN(EDRV_TX_DESCS_SIZE,4096);
	pTxQueue_p->m_pDescVirt = dma_alloc_coherent(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),iDescSize,&pTxQueue_p->m_DescDma,GFP_KERNEL);
	if(NULL == pTxQueue_p->m_pDescVirt)
	{
		// TODO: iResult = // Assign the Failure case
		iResult = kEplEdrvInitError;
	    goto Exit;
	}
	printk("... Done\n");

	// Clear the descriptor memory
	memset(pTxQueue_p->m_pDescVirt,0,EDRV_TX_DESCS_SIZE);


	pTxQueue_p->m_PktBuff = kzalloc((EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvPktBuff)),GFP_KERNEL);
	if(NULL == pTxQueue_p->m_PktBuff)
	{
			iResult = kEplEdrvInitError;
			goto Exit;
	}
	pTxQueue_p->m_iNextDesc = 0;
	pTxQueue_p->m_iNextWb = 0;
#ifdef USE_MULTIPLE_QUEUE
	// Map Queue to its Vector;
	pTxQueue_p->m_Qvector = EdrvInstance_l.m_pQvector[pTxQueue_p->m_iIndex];
#endif
Exit:
	return iResult;
}
//------------------------------------------------------------------------------
/**
\brief EdrvConfigureTxQueue
	   Configure Transmit queue
\param pTxQueue_p Pointer to the queue structure to configure

\return void

*/
//------------------------------------------------------------------------------
static void EdrvConfigureTxQueue(tEdrvQueue *pTxQueue_p )
{
	QWORD 	qwTxDescDma;
	DWORD	dwReg;
	INT 	iQueue = pTxQueue_p->m_iIndex;
	INT		iIndex;
	printk("Configure Tx Queue %d",iQueue);
	// Disable the Queue
	EDRV_REGDW_WRITE(EDRV_TXDCTL(iQueue),0);

	// Program the hw with queue values
	EDRV_REGDW_READ(EDRV_STATUS_REG);
    mdelay(10);

    qwTxDescDma = pTxQueue_p->m_DescDma;

	EDRV_REGDW_WRITE(EDRV_TDLEN(iQueue),EDRV_TX_DESCS_SIZE);
	EDRV_REGDW_WRITE(EDRV_TDBAL(iQueue),(qwTxDescDma & 0x00000000ffffffffULL));
	EDRV_REGDW_WRITE(EDRV_TDBAH(iQueue),(qwTxDescDma >> 32));

	EDRV_REGDW_WRITE(EDRV_TDHEAD(iQueue),0);
	EDRV_REGDW_WRITE(EDRV_TDTAIL(iQueue),0);

	// Setup the Threshold values for queue and Enable
	dwReg = 0;
	dwReg |= (EDRV_TXDCTL_PTHRESH | (EDRV_TXDCTL_HTHRESH << 8) | (EDRV_TXDCTL_WTHRESH << 16));
	if(0 == iQueue)
	{
		dwReg |=  EDRV_TXDCTL_PRIORITY;
	}
	dwReg |= EDRV_TXDCTL_QUEUE_EN;

	EDRV_REGDW_WRITE(EDRV_TXDCTL(iQueue),dwReg);
	// Poll till queue gets enabled
	printk("Poll TXDCTL");
	for(iIndex = 0; iIndex < 3 ; iIndex++ )
	{
		if((EDRV_REGDW_READ(EDRV_TXDCTL(iQueue)) & EDRV_TXDCTL_QUEUE_EN))
		{
			break;
		}
			msleep(1);
	}
	if(iIndex == 3)
	{
		printk("...Fail\n");
	}
	else
	{
		printk("....Done\n");
	}

}
//------------------------------------------------------------------------------
/**
\brief EdrvFreeRxQueues
	   Free the memory for Rx queues
\param void

\return void

*/
//------------------------------------------------------------------------------
static void EdrvFreeRxQueues(void)
{
	tEdrvQueue *pRxQueue = NULL;
	int iIndex;


	for(iIndex = 0; iIndex < EdrvInstance_l.m_RxMaxQueue ;iIndex++)
	{

		pRxQueue = EdrvInstance_l.m_pRxQueue[iIndex];

		if(NULL == pRxQueue)
		{
			break;
		}
		if(NULL != pRxQueue->m_pbBuf)
		{
			kfree(pRxQueue->m_pbBuf);
		}

		if(NULL != pRxQueue->m_PktBuff)
		{
			kfree(pRxQueue->m_PktBuff);
		}

		if (NULL != pRxQueue->m_pDescVirt)
		{

			// pci_free_consistent(pPciDev,EDRV_TX_DESCS_SIZE,EdrvInstance_l.m_pTxDescVirt,EdrvInstance_l.m_TxDescDma);
			 dma_free_coherent(&EdrvInstance_l.m_pPciDev->dev,EDRV_TX_DESCS_SIZE,pRxQueue->m_pDescVirt,pRxQueue->m_DescDma);
			 kfree(pRxQueue);
			 EdrvInstance_l.m_pRxQueue[iIndex] = NULL;

		}

	}
	EdrvInstance_l.m_RxMaxQueue = 0;
}
//------------------------------------------------------------------------------
/**
\brief EdrvFreeRxQueueBuffer
	   Free memory allocated for Receive buffers
\param pRxQueue_p Pointer to the queue structure to process

\return void

*/
//------------------------------------------------------------------------------
static void EdrvFreeRxQueueBuffer(tEdrvQueue *pRxQueue_p)
{
	int iIndex;

	for(iIndex = 0 ; iIndex < EDRV_MAX_RX_DESCRIPTOR ; iIndex++)
	{
	    // set report status for all descriptor
		 dma_unmap_single(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),\
	       	   	   	   	 pRxQueue_p->m_PktBuff[iIndex].m_DmaAddr,\
	       	   	   	   	 EDRV_MAX_FRAME_SIZE,DMA_FROM_DEVICE );

	 }
}
//------------------------------------------------------------------------------
/**
\brief EdrvFreeRxBuffers
	   Free memory allocated for Receive buffers
\param void

\return void

*/
//------------------------------------------------------------------------------
static void EdrvFreeRxBuffers(void)
{
	//tEdrvQueue *TxQueue;
	int iIndex;
	for(iIndex = 0; iIndex < EdrvInstance_l.m_RxMaxQueue; iIndex++ )
	{
		EdrvFreeRxQueueBuffer(EdrvInstance_l.m_pRxQueue[iIndex]);
	}
}
//------------------------------------------------------------------------------
/**
\brief EdrvInitRxQueue
	   Initialize the Rx queue with descriptors and memory resources
\param pRxQueue_p Pointer to the queue structure to initialize

\return tEplKernel Error code

*/
//------------------------------------------------------------------------------
static tEplKernel EdrvInitRxQueue(tEdrvQueue *pRxQueue_p)
{
	tEplKernel iResult = kEplSuccessful;
	int iDescSize;
	// Allocate Tx Descriptor

	printk("{%s}:Allocating Rx Desc %p\n",__FUNCTION__,pRxQueue_p);
	iDescSize = ALIGN(EDRV_RX_DESCS_SIZE,4096);
	pRxQueue_p->m_pDescVirt = dma_alloc_coherent(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),\
												 iDescSize,&pRxQueue_p->m_DescDma,\
												 GFP_KERNEL);
	if(NULL == pRxQueue_p->m_pDescVirt)
	{
		iResult = kEplEdrvInitError;
	    goto Exit;
	}
	printk("... Done\n");

	// Clear the descriptor memory
	memset(pRxQueue_p->m_pDescVirt,0,EDRV_RX_DESCS_SIZE);

	pRxQueue_p->m_PktBuff = kzalloc(EDRV_MAX_RX_DESCRIPTOR * sizeof(tEdrvPktBuff),GFP_KERNEL);
	if(NULL == pRxQueue_p->m_PktBuff)
	{
		iResult = kEplEdrvInitError;
		goto Exit;
	}

	pRxQueue_p->m_pbBuf = kzalloc(EDRV_RX_BUFFER_SIZE, GFP_KERNEL);

	if(NULL ==  pRxQueue_p->m_pbBuf)
	{
			iResult = kEplEdrvInitError;
	    	goto Exit;
	}


	pRxQueue_p->m_iNextDesc = 0;
	pRxQueue_p->m_iNextWb = 0;
#ifdef USE_MULTIPLE_QUEUE
	// Map Queue to its Vector;
	pRxQueue_p->m_Qvector = EdrvInstance_l.m_pQvector[pRxQueue_p->m_iIndex];
#endif
Exit:
	return iResult;
}
//------------------------------------------------------------------------------
/**
\brief EdrvAllocRxBuffer
	   Allocate receive buffers and associate with corresponding descriptor
\param pRxQueue_p Pointer to the queue structure to allocate buffers

\return tEplKernel Error code

*/
//------------------------------------------------------------------------------
static tEplKernel EdrvAllocRxBuffer(tEdrvQueue *pRxQueue_p)
{
	int 		iIndex;
	dma_addr_t  RxDma;
	tEplKernel iResult = kEplSuccessful;
	tEdrvAdvRxDesc *RxDesc;


	for(iIndex = 0 ; iIndex < EDRV_MAX_RX_DESCRIPTOR ; iIndex++)
	{

		RxDesc = EDRV_GET_RX_DESC(pRxQueue_p,iIndex);

	    RxDma = dma_map_single(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),\
	       	   	   	   	   	  (pRxQueue_p->m_pbBuf + (iIndex * EDRV_MAX_FRAME_SIZE)),\
	       	   	   	   	  	   EDRV_MAX_FRAME_SIZE,DMA_FROM_DEVICE );
	    if (dma_mapping_error(pci_dev_to_dev(EdrvInstance_l.m_pPciDev),RxDma))
	    {
	    	iResult = kEplEdrvInitError;
	    	goto Exit;
	    }
	    pRxQueue_p->m_PktBuff[iIndex].m_DmaAddr = RxDma;
	    pRxQueue_p->m_PktBuff[iIndex].m_VirtAddr = (pRxQueue_p->m_pbBuf + (iIndex * EDRV_MAX_FRAME_SIZE));
	    RxDesc->sRead.m_le_qwBufferAddr = cpu_to_le64(RxDma);

	    EDRV_REGDW_WRITE(EDRV_RDTAIL(pRxQueue_p->m_iIndex),iIndex);
	 }
Exit:
	return iResult;
}
//------------------------------------------------------------------------------
/**
\brief EdrvConfigureRxQueue
	   Configure Receive queue
\param pRxQueue_p Pointer to the queue structure to configure

\return void

*/
//------------------------------------------------------------------------------
static void EdrvConfigureRxQueue(tEdrvQueue *pRxQueue_p )
{
	QWORD 	qwRxDescDma;
	DWORD	dwReg;
	INT 	iQueue = pRxQueue_p->m_iIndex;
	INT 	iIndex;

	printk("Configure Rx Queue %d",iQueue);
	// Disable the Queue
	EDRV_REGDW_WRITE(EDRV_RXDCTL(iQueue),0);


	EDRV_REGDW_READ(EDRV_STATUS_REG);
    mdelay(10);

    // Program the hw with queue values
    qwRxDescDma = pRxQueue_p->m_DescDma;
	EDRV_REGDW_WRITE(EDRV_RDLEN(iQueue),EDRV_RX_DESCS_SIZE);
	EDRV_REGDW_WRITE(EDRV_RDBAL(iQueue),(qwRxDescDma & 0x00000000ffffffffULL));
	EDRV_REGDW_WRITE(EDRV_RDBAH(iQueue),(qwRxDescDma >> 32));

	EDRV_REGDW_WRITE(EDRV_RDHEAD(iQueue),0);
	EDRV_REGDW_WRITE(EDRV_RDTAIL(iQueue),0);

	// Configure Split-Receive register for queue
	dwReg = EDRV_REGDW_READ(EDRV_SRRCTL(iQueue));
	//dwReg |= EDRV_SRRCTL_DROP_EN;
	dwReg |= EDRV_SRRCTL_DESCTYPE_ADV; 			// Advance mode with no packet split
	EDRV_REGDW_WRITE(EDRV_SRRCTL(iQueue),dwReg);

	// Enable the rx queue
	dwReg = 0;
	dwReg =(EDRV_RXDCTL_PTHRESH | (EDRV_RXDCTL_HTHRESH << 8) | (EDRV_RXDCTL_WTHRESH << 16));
	dwReg |= EDRV_RXDCTL_QUEUE_EN;
	EDRV_REGDW_WRITE(EDRV_RXDCTL(iQueue),dwReg);

	printk("Poll RXDCTL");
	for(iIndex = 0 ;iIndex < 3;iIndex++)
	{
		dwReg = EDRV_REGDW_READ(EDRV_RXDCTL(iQueue));
		if(( dwReg & EDRV_RXDCTL_QUEUE_EN))
		{
			break;
		}
		EDRV_REGDW_WRITE(EDRV_RXDCTL(iQueue),dwReg);
		msleep(1);
	}

	if(iIndex == 3)
	{
		printk("...Fail\n");
	}
	else
	{
		printk("....Done\n");
	}
}
#ifdef QAV_MODE
//------------------------------------------------------------------------------
/**
\brief EdrvInitQavMode
	   Configure the device for Qav mode using 1 Tx queue as SR queue with
	   Launch time logic
\param void

\return tEplKernel Error code

*/
//------------------------------------------------------------------------------
static void EdrvInitQavMode( void )
{
	DWORD	  dwTqavctrl;
	DWORD	  dwLaunchOff;
	DWORD	  dwTqavcc0 , dwTqavcc1;
	DWORD  	  dwTxpbsize, dwRxpbsize;

	// TODO: RX configuration
	// reconfigure the rx packet buffer allocation to 30k
	dwRxpbsize = EDRV_REGDW_READ(EDRV_RXPBSIZE_REG);
	dwRxpbsize &= ~EDRV_RXPBSIZE_CLEAR;
	dwRxpbsize |= EDRV_RXPBSIZE_DEF;
	EDRV_REGDW_WRITE(EDRV_RXPBSIZE_REG, dwRxpbsize);



#ifdef USE_MULTIPLE_QUEUE
	/* reconfigure the tx packet buffer allocation to 8Kb:Q0 8Kb:Q1 4Kb:Q2 & Q3 */
	dwTxpbsize = (8);
	dwTxpbsize |= (8) << EDRV_TXPBSIZE_TX1PB_SHIFT;
	dwTxpbsize |= (4) << EDRV_TXPBSIZE_TX2PB_SHIFT;
	dwTxpbsize |= (4) << EDRV_TXPBSIZE_TX3PB_SHIFT;
	EDRV_REGDW_WRITE(EDRV_TXPBSIZE_REG, dwTxpbsize);
#endif


		// DMA Configuration
		EDRV_REGDW_WRITE(EDRV_DTX_MAX_PKTSZ_REG,EDRV_MAX_FRAME_SIZE/64);

		// Configure Q0 as SR queue
		dwTqavcc0 = EDRV_TQAVCC_QUEUE_MODE_SR; /* no idle slope */
		EDRV_REGDW_WRITE(EDRV_TQAVCC(0), dwTqavcc0);




		dwTqavctrl = 0;
		dwTqavctrl = EDRV_TQAVCTRL_TXMODE | \
						EDRV_TQAVCTRL_FETCH_ARB | \
						EDRV_TQAVCTRL_TRANSTIM  | \
						EDRV_TQAVCTRL_SP_WAIT_SR| \
						EDRV_TQAVCTRL_1588_STAT_EN;

		/* default to a 5 usec prefetch delta from launch time */
		dwTqavctrl |= (5 << 5) << EDRV_TQAVCTRL_FETCH_TM_SHIFT ;

		EDRV_REGDW_WRITE(EDRV_TQAVCTRL_REG, dwTqavctrl);

		dwLaunchOff = 0;
		dwLaunchOff |= (4 << 5) << EDRV_LAUNCH_OSO_SHIFT;

		EDRV_REGDW_WRITE(EDRV_LAUNCH_OSO,dwLaunchOff);
#ifdef USE_MULTIPLE_QUEUE
		/* set Q0 to highest priority, Q3 to the lowest .. */
		EDRV_REGDW_WRITE(EDRV_TQAVARBCTRL_REG, \
						EDRV_TQAVARBCTRL_TXQPRIO(0,3) | \
						EDRV_TQAVARBCTRL_TXQPRIO(1,2) | \
						EDRV_TQAVARBCTRL_TXQPRIO(2,1) | \
						EDRV_TQAVARBCTRL_TXQPRIO(3,0) );
#endif

}
#endif
//------------------------------------------------------------------------------
/**
\brief 	EdrvInitOne
	   	initializes one PCIe device
\param 	pPciDev pointer to corresponding PCI device structure
\param	pId     PCI device ID

\return (int)
\retval 0 No error
\retval Non-Zero error code

*/
//------------------------------------------------------------------------------
static int EdrvInitOne(struct pci_dev *pPciDev,
                       const struct pci_device_id *pId)
{
	int             iResult = 0;
	tEdrvQueue 		*pQueue;
	tEdrvQVector 	*pVector;
	DWORD			dwReg;
#ifdef DISABLE_AUTONEG
	WORD			wReg;
#endif
	INT             iIndex;
	struct timespec sSysTime;
	if (EdrvInstance_l.m_pPciDev != NULL)
	{
		// Perform a sanity check to avoid insertion of module again
	    printk("%s device %s discarded\n", __FUNCTION__, pci_name(pPciDev));
	    iResult = -ENODEV;
	    goto Exit;
	}

	// Enable device
	iResult = pci_enable_device(pPciDev);
	if(0 != iResult)
	{
		goto Exit;
	}

	EdrvInstance_l.m_pPciDev = pPciDev;

	if (EdrvInstance_l.m_pPciDev == NULL)
	{
	   printk("%s pPciDev==NULL\n", __FUNCTION__);
	}

	iResult = dma_set_mask(pci_dev_to_dev(pPciDev), DMA_BIT_MASK(64));
	if ( 0 == iResult )
	{
	   dma_set_coherent_mask(pci_dev_to_dev(pPciDev), DMA_BIT_MASK(64));
	}
	else
	{
	    printk(" Using 32 bit Mask\n");
	    iResult = dma_set_mask(&pPciDev->dev, DMA_BIT_MASK(32));
	    if(0 == iResult)
	    {
	       iResult = dma_set_coherent_mask(&pPciDev->dev, DMA_BIT_MASK(32));
	       if(0 != iResult)
	       {
	           printk("[EPLi210]: No usable DMA configuration available\n");
	              goto Exit;
	       }
	    }
	}

	iResult = pci_request_regions(pPciDev,DRIVER_NAME);

	if ( 0 != iResult )
	{
		//TODO: Exit with device remove
		printk("pci_request_regions....Failed\n");
		goto Exit;

	}

	pci_set_master(pPciDev);

	EdrvInstance_l.m_pIoAddr = ioremap (pci_resource_start(pPciDev, 0), pci_resource_len(pPciDev, 0));
	if( NULL == EdrvInstance_l.m_pIoAddr)
	{
		iResult = -EIO;
		//TODO: Exit with device remove
		goto Exit;
	}
	printk("EdrvInstance_l.m_pIoAddr :0X%p\n",EdrvInstance_l.m_pIoAddr);

	// disable the interrupts
	EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR,EDRV_EIMC_CLEAR_ALL);
	EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR,~0);
	EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

	// Disable the Master
	dwReg = 0;
	dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
	dwReg |= (EDRV_CTRL_MASTER_DIS);
	EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);

	for(iIndex = EDRV_MASTER_DIS_TIMEOUT; iIndex > 0 ; iIndex--)
	{
	   if((EDRV_REGDW_READ(EDRV_STATUS_REG) & EDRV_STATUS_MASTER_EN) == 0)
	   {
	     break;
	   }
	   msleep(1);
    }

	if(0 == iIndex)
    {

	    iResult = -EIO;
        goto Exit;
	}

	// disable the interrupts
	EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR,EDRV_EIMC_CLEAR_ALL);
	EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR,~0);
	EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    EDRV_REGDW_READ(EDRV_STATUS_REG);

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    dwReg |= (EDRV_CTRL_DEV_RST);
 	EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);
 	msleep(5);

	iIndex = 0;
	while (iIndex < EDRV_AUTO_READ_DONE_TIMEOUT) {
		if (EDRV_REGDW_READ(EDRV_EECD_REG) & EDRV_EECD_AUTO_RD)
			break;
		msleep(1);
		iIndex++;
	}

	if (iIndex == EDRV_AUTO_READ_DONE_TIMEOUT) {
		printk("Auto read by HW from NVM has not completed.\n");
		goto Exit;
	}


	EDRV_REGDW_WRITE(EDRV_STATUS_REG,EDRV_STATUS_DEV_RST_SET);


	// disable the interrupts
	EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR,EDRV_EIMC_CLEAR_ALL);
	EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR,~0);
	EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);



	// Set Device configuration
	dwReg = 0;
	dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
	dwReg &= ~(EDRV_CTRL_FD | EDRV_CTRL_ILOS | EDRV_CTRL_TFCE | EDRV_CTRL_RFCE);
	dwReg |= EDRV_CTRL_SLU;
	EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);

	// Reset the phy
	//1. Acquire Phy Sw semaphore for PHY
	EdrvAcquireSwfwSync(EDRV_SWFW_PHY0_SM);

#ifdef DISABLE_AUTONEG
	//EdrvAcquireSwfwSync(EDRV_SWFW_PHY0_SM);
	wReg = EdrvMdicRead(PHY_CONTROL_REG_OFFSET);

	wReg &= ~(0x1000 | 0x0040);
	wReg |= PHY_LINK_SPEED_100;
	EdrvMdicWrite(PHY_CONTROL_REG_OFFSET,wReg);

#endif

	//2. Set Phy reset bit in CTRL register
	dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
	dwReg |= (EDRV_CTRL_PHY_RST);
	EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);
	//3. Wait for 1 ms to complete the effect
	msleep(1);
	//4. Clear the bit
	dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
	dwReg &= ~(EDRV_CTRL_PHY_RST);
	EDRV_REGDW_WRITE(EDRV_CTRL_REG,dwReg);
	msleep(10);
	//5. Release semaphore
	EdrvReleaseSwfwSync(EDRV_SWFW_PHY0_SM);

	// Get Control from hardware
	dwReg = 0;
	dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
	dwReg |= EDRV_CTRL_EXTN_DRV_LOAD;
	EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG,dwReg);



	// Set the Queue Parameters
	EdrvInstance_l.m_TxMaxQueue = EDRV_MAX_TX_QUEUES;
	EdrvInstance_l.m_RxMaxQueue = EDRV_MAX_RX_QUEUES;
#ifdef USE_MULTIPLE_QUEUE
	EdrvInstance_l.m_NumQVectors = EDRV_MAX_QUEUE_VECTOR;
#endif
	//Initialise the SYSTIM timer with current system time
	EDRV_REGDW_WRITE(EDRV_TSAUXC_REG, 0x0);
	sSysTime = ktime_to_timespec(ktime_get_real());
	EdrvPtpWrite(&sSysTime);


	// Clear the Statistic register
	//TODO: Add other required registers
	dwReg = EDRV_REGDW_READ(EDRV_STAT_TPT);
	dwReg = EDRV_REGDW_READ(EDRV_STAT_TPR);
	dwReg = EDRV_REGDW_READ(EDRV_STAT_GPRC);
	dwReg = EDRV_REGDW_READ(EDRV_STAT_BPRC);
	dwReg = EDRV_REGDW_READ(EDRV_STAT_MPRC);

	//TODO:CHeck this
	// EDRV_REGDW_WRITE(EDRV_EEER_REG,0);
	EDRV_REGDW_WRITE(EDRV_TIPG_REG,EDRV_TIPG_DEF);
	iResult = pci_enable_msi(pPciDev);
	if (iResult != 0)
	{
	   printk("%s Could not enable MSI\n", __FUNCTION__);
	}

	// Allocate Tx Buffer memory
	EdrvInstance_l.m_pbTxBuf = kzalloc(EDRV_TX_BUFFER_SIZE, GFP_KERNEL);
	if(NULL == EdrvInstance_l.m_pbTxBuf)
	{
		iResult = -ENOMEM;
		goto ExitDriver;
	}
	for(iIndex = 0 ;iIndex < EdrvInstance_l.m_TxMaxQueue ; iIndex++)
	{
	  	pQueue = kzalloc(sizeof(tEdrvQueue), GFP_KERNEL);
	  	if(NULL == pQueue)
	  	{
	  		goto ExitDriver;
	  	}
	   	pQueue->m_iIndex = iIndex;
	   	EdrvInstance_l.m_pTxQueue[iIndex] = pQueue;
	}

	for(iIndex = 0 ;iIndex < EdrvInstance_l.m_TxMaxQueue ; iIndex++)
	{
	        iResult = EdrvInitTxQueue(EdrvInstance_l.m_pTxQueue[iIndex]);
	        if( kEplSuccessful != iResult)
	        {
	        	goto ExitQueue;
	        }
	}

	// Alloc Rx Queue here

	for(iIndex = 0 ;iIndex < EdrvInstance_l.m_RxMaxQueue ; iIndex++)
	{
	   	pQueue = kzalloc(sizeof(tEdrvQueue), GFP_KERNEL);
	   	if(NULL == pQueue)
	   	{
	   		goto ExitDriver;
	   	}
	   	pQueue->m_iIndex = iIndex;
	   	EdrvInstance_l.m_pRxQueue[iIndex] = pQueue;
	 }

	 for(iIndex = 0 ;iIndex < EdrvInstance_l.m_RxMaxQueue ; iIndex++)
	 {
	    iResult = EdrvInitRxQueue(EdrvInstance_l.m_pRxQueue[iIndex]);
	    if( kEplSuccessful != iResult)
	    {
	       	goto ExitQueue;
	    }
	 }

#ifdef QAV_MODE
	 // Configure device for Qav mode
	 EdrvInitQavMode();
#endif

	 // Setup Tx Configuration

	 EDRV_REGDW_WRITE(EDRV_TXDCTL(0),0); // Disable Q0 before proceeding
	 dwReg = 0;
	 dwReg = EDRV_REGDW_READ(EDRV_TCTL_REG);
	 dwReg &= ~(EDRV_TCTL_CLEAR_CT | EDRV_TCTL_RTLC);
	 dwReg |= (/*EDRV_TCTL_CT  |*/ EDRV_TCTL_PSP);
	 EDRV_REGDW_WRITE(EDRV_TCTL_REG,dwReg);

	 // Set the default collision threshold
	 dwReg = 0;
	 dwReg = EDRV_REGDW_READ(EDRV_TCTL_EXT_REG);
	 dwReg &= ~EDRV_TCTL_EXT_COLD_CLEAR;
	 dwReg |= EDRV_TCTL_EXT_COLD;
	 EDRV_REGDW_WRITE(EDRV_TCTL_EXT_REG,dwReg);

	 // Enable the Tx
	 dwReg = 0;
	 dwReg = EDRV_REGDW_READ(EDRV_TCTL_REG);
	 dwReg |= EDRV_TCTL_EN;
	 EDRV_REGDW_WRITE(EDRV_TCTL_REG,dwReg);

	 for(iIndex = 0 ;iIndex < EdrvInstance_l.m_TxMaxQueue ; iIndex++)
	 {
	   	EdrvConfigureTxQueue(EdrvInstance_l.m_pTxQueue[iIndex]);
	 }

	 // Diasable 1st Rx Queue
	 EDRV_REGDW_WRITE(EDRV_RXDCTL(0),0);

	 dwReg = 0;
	 dwReg = EDRV_REGDW_READ(EDRV_RCTL_REG);
	 dwReg &= ~(3 << EDRV_RCTL_MO_SHIFT);
	 dwReg &= ~(3 << EDRV_RCTL_BSIZE_OFFSET);
	 dwReg &= ~EDRV_RCTL_LBM_CLEAR;
	 dwReg |= (EDRV_RCTL_SBP | EDRV_RCTL_EN | EDRV_RCTL_BAM | EDRV_RCTL_SECRC | EDRV_RCTL_LPE);
	 // Receive all packets
#ifdef PROMICIOUS_MODE
	 dwReg |= (EDRV_RCTL_UPE | EDRV_RCTL_MPE);
#endif

	 // Enable Recieve
	 EDRV_REGDW_WRITE(EDRV_RCTL_REG,dwReg);

	 // configure Rx queues
	 for(iIndex = 0 ;iIndex < EdrvInstance_l.m_RxMaxQueue ; iIndex++)
	 {
	   	EdrvConfigureRxQueue(EdrvInstance_l.m_pRxQueue[iIndex]);
	 }

	 // Allocate Rx Buffers
	 for(iIndex = 0 ;iIndex < EdrvInstance_l.m_RxMaxQueue ; iIndex++)
	 {
		 iResult = EdrvAllocRxBuffer(EdrvInstance_l.m_pRxQueue[iIndex]);
		 if( kEplSuccessful != iResult)
		 {
		   	goto ExitQueue;
		 }
	 }

	 printk("Requesting Interrupt");
	 iResult = request_irq(pPciDev->irq, TgtEthIsr, 0, DRIVER_NAME, pPciDev);
	 if (iResult != 0)
	 {
	        goto Exit;
	 }
	 printk("...Done Using MSI\n");
	 EdrvInstance_l.m_iIrq = pPciDev->irq;

	 // enable interrupts

	 EDRV_REGDW_WRITE(EDRV_INTR_MASK_SET_READ,(EDRV_INTR_ICR_TXDW | EDRV_INTR_ICR_RXDW));
	 dwReg = EDRV_REGDW_READ(EDRV_EXT_INTR_MASK_SET);
	 dwReg |= /*0x8000001F;*/(EDRV_EIMC_OTHR_EN ); // TODO:Test this
	 EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_SET,dwReg);

	 printk("%s waiting for link up...", __FUNCTION__);
	 for (iIndex = EDRV_LINK_UP_TIMEOUT; iIndex > 0; iIndex -= 100)
	 {
	     if ((EDRV_REGDW_READ(EDRV_STATUS_REG) & EDRV_STATUS_LU) )
	     {
	             printk("Link Up\n");
	             dwReg = EDRV_REGDW_READ(EDRV_STATUS_REG);
	             break;
         }
         msleep(100);
	 }

	 if (iIndex == 0)
	 {
	    printk("Link Down\n");
	    goto Exit;
	 }
	// EdrvClearGpio(1);
#ifdef DISABLE_AUTONEG
	 EdrvAcquireSwfwSync(EDRV_SWFW_PHY0_SM);
	 wReg = EdrvMdicRead(PHY_CONTROL_REG_OFFSET);
	 printk("Phy Status %x\n",wReg);
	 EdrvReleaseSwfwSync(EDRV_SWFW_PHY0_SM);
#endif
	 goto Exit;

ExitQueue:
	//TODO:Call free queues here
ExitDriver:
	//TODO: Call Disable device here
Exit:
    printk("%s finished with %d\n", __FUNCTION__, iResult);
	return iResult;

}

//------------------------------------------------------------------------------
/**
\brief 	EdrvRemoveOne
	    shuts down one PCIe device
\param 	pPciDev pointer to corresponding PCI device structure

\return void

*/
//------------------------------------------------------------------------------
static void EdrvRemoveOne(struct pci_dev *pPciDev)
{
	DWORD	 dwReg;
	INT  	 iIndex;
	WORD	 wReg;

	if(pPciDev != EdrvInstance_l.m_pPciDev)
	{
		BUG_ON(EdrvInstance_l.m_pPciDev != pPciDev);
		goto Exit;
	}

	EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR,EDRV_EIMC_CLEAR_ALL);
	EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR,~0);
	EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

	// Disable Tx
	dwReg = EDRV_REGDW_READ(EDRV_TCTL_REG);
	dwReg &= ~EDRV_TCTL_EN;
	dwReg |= EDRV_TCTL_SWFLSH;
	EDRV_REGDW_WRITE(EDRV_TCTL_REG,dwReg);

	//Disable Rx
	dwReg = EDRV_REGDW_READ(EDRV_RCTL_REG);
	dwReg &= ~EDRV_RCTL_EN;
	dwReg |= EDRV_RCTL_SWFLUSH;
	EDRV_REGDW_WRITE(EDRV_RCTL_REG,dwReg);

	EDRV_REGDW_READ(EDRV_STATUS_REG);

	usleep_range(10000, 20000);

	dwReg = EDRV_REGDW_READ(EDRV_TCTL_REG);
	dwReg &= ~EDRV_TCTL_SWFLSH;
	EDRV_REGDW_WRITE(EDRV_TCTL_REG,dwReg);

	//Disable Rx
	dwReg = EDRV_REGDW_READ(EDRV_RCTL_REG);
	dwReg &= ~EDRV_RCTL_SWFLUSH;
	EDRV_REGDW_WRITE(EDRV_RCTL_REG,dwReg);

	EdrvFreeTxBuffers();
	EdrvFreeRxBuffers();

	EdrvFreeTxQueues();
	EdrvFreeRxQueues();

	if(NULL != EdrvInstance_l.m_pbTxBuf )
	{
		kfree(EdrvInstance_l.m_pbTxBuf);
	}
	// Power down Phy
	wReg = EdrvMdicRead(PHY_I210_COPPER_SPEC);
	wReg |= PHY_I210_CS_POWER_DOWN;
	EdrvMdicWrite(PHY_I210_COPPER_SPEC,wReg);

	wReg = EdrvMdicRead(PHY_CONTROL_REG_OFFSET);
	wReg |= PHY_CONTROL_POWER_DOWN;
	EdrvMdicWrite(PHY_CONTROL_REG_OFFSET,wReg);
	msleep(1);

	// Release the control to Hardware
	dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
	dwReg &= ~EDRV_CTRL_EXTN_DRV_LOAD;
	EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG,dwReg);

	free_irq(EdrvInstance_l.m_iIrq, pPciDev);
	pci_disable_msi(pPciDev);
#ifdef USE_MULTIPLE_QUEUE
	// Free Queue vectors
	for(iIndex = 0; iIndex < EdrvInstance_l.m_NumQVectors ; iIndex++)
	{
	  	pVector = EdrvInstance_l.m_pQvector[iIndex];
	   	kfree(pVector);
	   	EdrvInstance_l.m_pQvector[iIndex] = NULL;
	}
	EdrvInstance_l.m_NumQVectors = 0;
#endif

	// unmap controller's register space
	if (EdrvInstance_l.m_pIoAddr != NULL)
	{
	    iounmap(EdrvInstance_l.m_pIoAddr);
	    EdrvInstance_l.m_pIoAddr = NULL;
	}

	// Release Memory Regions
	pci_release_regions(pPciDev);

	// disable the PCI device
	pci_disable_device(pPciDev);

	EdrvInstance_l.m_pPciDev = NULL;
Exit:
    return;
}
