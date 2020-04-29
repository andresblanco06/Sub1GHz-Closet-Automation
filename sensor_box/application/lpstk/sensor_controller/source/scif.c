/// \addtogroup module_scif_driver_setup
//@{
#include "scif.h"
#include "scif_framework.h"
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_aon_event.h)
#include DeviceFamily_constructPath(inc/hw_aon_rtc.h)
#include DeviceFamily_constructPath(inc/hw_aon_pmctl.h)
#include DeviceFamily_constructPath(inc/hw_aux_sce.h)
#include DeviceFamily_constructPath(inc/hw_aux_smph.h)
#include DeviceFamily_constructPath(inc/hw_aux_spim.h)
#include DeviceFamily_constructPath(inc/hw_aux_evctl.h)
#include DeviceFamily_constructPath(inc/hw_aux_aiodio.h)
#include DeviceFamily_constructPath(inc/hw_aux_timer01.h)
#include DeviceFamily_constructPath(inc/hw_aux_sysif.h)
#include DeviceFamily_constructPath(inc/hw_event.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_ioc.h)
#include <string.h>
#if defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#endif


// OSAL function prototypes
uint32_t scifOsalEnterCriticalSection(void);
void scifOsalLeaveCriticalSection(uint32_t key);




/// Firmware image to be uploaded to the AUX RAM
static const uint16_t pAuxRamImage[] = {
    /*0x0000*/ 0x140E, 0x0417, 0x140E, 0x0440, 0x140E, 0x044A, 0x140E, 0x0467, 0x140E, 0x0470, 0x140E, 0x0479, 0x140E, 0x0482, 0x8953, 0x9954, 
    /*0x0020*/ 0x8D29, 0xBEFD, 0x4553, 0x2554, 0xAEFE, 0x445C, 0xADB7, 0x745B, 0x545B, 0x7000, 0x7CB2, 0x68BB, 0x00A8, 0x1439, 0x68BC, 0x00AA, 
    /*0x0040*/ 0x1439, 0x68BD, 0x00AC, 0x1439, 0x78B2, 0xF801, 0xFA02, 0xBEF2, 0x78B9, 0x68BB, 0xFD0E, 0x68BD, 0xED92, 0xFD06, 0x7CB9, 0x78B8, 
    /*0x0060*/ 0xFA01, 0xBE05, 0x7002, 0x7CB8, 0x78B8, 0xFA00, 0xBEFD, 0x6440, 0x0487, 0x78B2, 0x8F1F, 0xED8F, 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 
    /*0x0080*/ 0x755B, 0x555B, 0x78B7, 0x60BF, 0xEF27, 0xE240, 0xEF27, 0x7000, 0x7CB7, 0x0487, 0x6477, 0x0000, 0x18B9, 0x9D88, 0x9C01, 0xB60E, 
    /*0x00A0*/ 0x10A6, 0xAF19, 0xAA00, 0xB60A, 0xA8FF, 0xAF39, 0xBE07, 0x0CB2, 0x8600, 0x88AA, 0x8F08, 0xFD47, 0x9DB7, 0x08B2, 0x8801, 0x8A02, 
    /*0x00C0*/ 0xBEEB, 0x254F, 0xAEFE, 0x645B, 0x445B, 0x4477, 0x0487, 0x5656, 0x655B, 0x455B, 0x0000, 0x0CB2, 0x0001, 0x0CB3, 0x1519, 0x0487, 
    /*0x00E0*/ 0x5657, 0x665B, 0x465B, 0x0000, 0x0CB2, 0x0002, 0x0CB3, 0x1519, 0x0487, 0x5658, 0x675B, 0x475B, 0x0001, 0x0CB2, 0x0004, 0x0CB3, 
    /*0x0100*/ 0x15D6, 0x0487, 0x765B, 0x565B, 0x86FF, 0x03FF, 0x0CB5, 0x645C, 0x78B4, 0x68B5, 0xED37, 0xB605, 0x0000, 0x0CB4, 0x7CBA, 0x6540, 
    /*0x0120*/ 0x0CB5, 0x78B5, 0x68B6, 0xFD0E, 0xF801, 0xE95A, 0xFD0E, 0xBE01, 0x6553, 0xBDB7, 0x700B, 0xFB96, 0x4453, 0x2454, 0xAEFE, 0xADB7, 
    /*0x0140*/ 0x6453, 0x2454, 0xA6FE, 0x7000, 0xFB96, 0xADB7, 0x0000, 0x0000, 0x00DD, 0x01BA, 0x0118, 0x01BD, 0x01AA, 0x01FE, 0x0000, 0x0000, 
    /*0x0160*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0180*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x01A0*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x500E, 0x4080, 0x68D7, 
    /*0x01C0*/ 0x70E4, 0x7E10, 0x7082, 0x0611, 0x0A0D, 0x6CD7, 0x86C3, 0x0350, 0x8801, 0x8B49, 0x8DB1, 0x5007, 0x4080, 0x68D7, 0x70F2, 0x7E10, 
    /*0x01E0*/ 0x7082, 0x0611, 0x0A0D, 0x6CD7, 0x500E, 0x4054, 0x68D7, 0x70FB, 0x7E10, 0x7082, 0x0611, 0x0A0D, 0x6CD7, 0x500F, 0x4001, 0x68D7, 
    /*0x0200*/ 0x7104, 0x7E10, 0x7082, 0x0611, 0x0A0D, 0x6CD7, 0x0098, 0x8B56, 0x655B, 0x455B, 0x7656, 0x8605, 0x01DC, 0x8B81, 0x8623, 0x0322, 
    /*0x0220*/ 0x8B7D, 0x0036, 0x8B57, 0x665B, 0x465B, 0x7657, 0x647F, 0xADB7, 0xADB7, 0x149A, 0x5656, 0x655B, 0x455B, 0x5657, 0x665B, 0x465B, 
    /*0x0240*/ 0x1000, 0x08B3, 0x8C01, 0xB677, 0x6000, 0x1622, 0x6CD7, 0x7082, 0x68D7, 0x1632, 0x6CD7, 0x7000, 0x68D7, 0x1632, 0x6CD7, 0x08D7, 
    /*0x0260*/ 0x8A00, 0xBE5F, 0x68D7, 0x161C, 0x6CD7, 0x7083, 0x68D7, 0x1632, 0x6CD7, 0x7000, 0x68D7, 0x1662, 0x6CD7, 0x8D47, 0x7000, 0x68D7, 
    /*0x0280*/ 0x1662, 0x6CD7, 0xAD47, 0x7000, 0x68D7, 0x1662, 0x6CD7, 0xBD47, 0x7001, 0x68D7, 0x1662, 0x6CD7, 0x68D7, 0x1627, 0x6CD7, 0x58D7, 
    /*0x02A0*/ 0xDA00, 0xBE3F, 0xADA0, 0xAD08, 0x00A5, 0xAB09, 0x8B0C, 0x8620, 0xA960, 0x86F6, 0xA800, 0x08D8, 0x50CC, 0xAF3D, 0xDD42, 0x68D6, 
    /*0x02C0*/ 0xDD1E, 0xDD90, 0x68BF, 0xDD2E, 0x9E02, 0x2CD6, 0x9201, 0xAD47, 0xADA0, 0xAD0B, 0x3064, 0xAB0A, 0xBB0C, 0x8620, 0xA960, 0x30C2, 
    /*0x02E0*/ 0xAF3B, 0xBD42, 0x58C1, 0xBD1D, 0xBD90, 0x58BE, 0xBD2D, 0x9E02, 0x2CC1, 0x9202, 0x8801, 0x8A0A, 0xBE02, 0x9204, 0x0000, 0x0CD8, 
    /*0x0300*/ 0x0098, 0x8B56, 0x655B, 0x455B, 0x7656, 0x8605, 0x01DC, 0x8B81, 0x8623, 0x0322, 0x8B7D, 0x0036, 0x8B57, 0x665B, 0x465B, 0x7657, 
    /*0x0320*/ 0x647F, 0x08D7, 0x8A00, 0xB606, 0x0000, 0x0CD7, 0x9210, 0x5657, 0x665B, 0x465B, 0x059F, 0x9208, 0x5656, 0x655B, 0x455B, 0x1CC0, 
    /*0x0340*/ 0x9A00, 0xB603, 0x08B4, 0x8201, 0x0CB4, 0x0004, 0x18B0, 0x8D01, 0x0CAE, 0xADB7, 0x500E, 0x4080, 0x68D7, 0x71B1, 0x7E10, 0x7082, 
    /*0x0360*/ 0x0611, 0x0A0D, 0x6CD7, 0x5656, 0x655B, 0x455B, 0x5657, 0x665B, 0x465B, 0xADB7, 0x0001, 0x0CA7, 0xADB7, 0x14A0, 0x5001, 0x86C2, 
    /*0x0380*/ 0x4210, 0x68DC, 0x71C6, 0x7E10, 0x7088, 0x0681, 0x0A0D, 0x6CDC, 0x0078, 0x8B82, 0x8623, 0x0322, 0x8B7E, 0x0035, 0x8B58, 0x675B, 
    /*0x03A0*/ 0x475B, 0x7658, 0x6480, 0x0001, 0x0CA7, 0xADB7, 0x14A0, 0x08DC, 0x8A00, 0xBE23, 0x5000, 0x68DC, 0x71E0, 0x7E10, 0x7088, 0x0690, 
    /*0x03C0*/ 0x0A0D, 0x6CDC, 0x08DC, 0x8A00, 0xBE15, 0x8D47, 0x8DA8, 0x8DAC, 0xFDA4, 0x88F5, 0x8D91, 0xFD88, 0x7CDB, 0x08DA, 0xFD28, 0xA603, 
    /*0x03E0*/ 0x08B4, 0x8202, 0x0CB4, 0x08D9, 0xFD28, 0x9E03, 0x08B4, 0x8202, 0x0CB4, 0x05FD, 0x68DC, 0x1627, 0x6CDC, 0xADB7, 0x5001, 0x86C8, 
    /*0x0400*/ 0x4010, 0x68DC, 0x8602, 0x7207, 0x7E10, 0x7088, 0x0681, 0x0A0D, 0x6CDC, 0x5658, 0x675B, 0x475B, 0xADB7, 0x0000, 0x0000, 0x0000, 
    /*0x0420*/ 0x0000, 0x0E0D, 0x5E0E, 0x1622, 0x1632, 0x7A0E, 0x1632, 0xFD44, 0x1632, 0x0A10, 0x1627, 0x8DB7, 0x53F4, 0x1650, 0x76BB, 0x1653, 
    /*0x0440*/ 0x53F4, 0x1650, 0x55BB, 0x53F8, 0x1650, 0x56BB, 0xADB7, 0x55BB, 0x53F4, 0x1650, 0x76BB, 0x1653, 0x53FD, 0x1650, 0x75BB, 0x53F5, 
    /*0x0460*/ 0x1650, 0xADB7, 0xEA00, 0xBE1B, 0xB50E, 0xFDA1, 0x8601, 0xFC00, 0xB602, 0x75BB, 0x8E02, 0x55BB, 0xFD47, 0x53FA, 0x1650, 0x76BB, 
    /*0x0480*/ 0x1653, 0x53FD, 0x1650, 0x56BB, 0x75BB, 0x53F4, 0x1650, 0x76BB, 0x1653, 0x53FA, 0x1650, 0x35BF, 0xA601, 0xE201, 0x56BB, 0xADB7, 
    /*0x04A0*/ 0xD802, 0xDEFE, 0xADB7, 0x53F6, 0x36BF, 0xAE0B, 0x36BF, 0xAE09, 0x36BF, 0xAE07, 0x36BF, 0xAE05, 0x36BF, 0xAE03, 0xD801, 0xBEF4, 
    /*0x04C0*/ 0xE202, 0xADB7, 0xEA00, 0xBE1C, 0xB50B, 0x75BB, 0x53F5, 0x1650, 0x76BB, 0x1653, 0x53FF, 0x1650, 0xFDA1, 0x35BF, 0xA601, 0xF201, 
    /*0x04E0*/ 0x56BB, 0x8601, 0xFC00, 0xB602, 0x75BB, 0x8E02, 0x55BB, 0xFD47, 0x53F9, 0x1650, 0x76BB, 0x1653, 0x53FD, 0x1650, 0x56BB, 0xF0FF, 
    /*0x0500*/ 0xADB7, 0x0E0D, 0x5E0E, 0x1622, 0x1632, 0x7A0E, 0x1632, 0xFD44, 0xFDA8, 0x1632, 0xFD44, 0xF0FF, 0x1632, 0x0A10, 0x1627, 0x8DB7, 
    /*0x0520*/ 0x0E0D, 0x5E0E, 0x7E0F, 0x1622, 0x1632, 0x7A0E, 0x1632, 0xEA00, 0xBE0B, 0x161C, 0x7A0F, 0xF201, 0x1632, 0x7000, 0x1662, 0x8D47, 
    /*0x0540*/ 0x8DA0, 0x7001, 0x1662, 0xFD08, 0x0A10, 0x1627, 0x8DB7
};


/// Look-up table that converts from AUX I/O index to MCU IOCFG offset
static const uint8_t pAuxIoIndexToMcuIocfgOffsetLut[] = {
    0, 68, 64, 60, 56, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 12, 0, 0, 0, 120, 116, 112, 108, 104, 100, 96, 92, 88, 84, 80, 76, 72
};


/** \brief Look-up table of data structure information for each task
  *
  * There is one entry per data structure (\c cfg, \c input, \c output and \c state) per task:
  * - [31:20] Data structure size (number of 16-bit words)
  * - [19:12] Buffer count (when 2+, first data structure is preceded by buffering control variables)
  * - [11:0] Address of the first data structure
  */
static const uint32_t pScifTaskDataStructInfoLut[] = {
//  cfg         input       output      state       
    0x0020117C, 0x00000000, 0x01701180, 0x002011AE, // I2C Temp and Humidity Sensor
    0x002011B2, 0x00000000, 0x001011B6, 0x001011B8  // I2C Light Sensor
};




/// Run-time logging signatures (CRC-16) for each data structure for each task
static const uint16_t pRtlTaskStructSignatures[] = {
    0x3B41, 0x0000, 0x70BA, 0xE79C, 0xA2EB, 0x0000, 0x5B30, 0xBEA0
};




// No task-specific initialization functions




// No task-specific uninitialization functions




/** \brief Performs driver setup dependent hardware initialization
  *
  * This function is called by the internal driver initialization function, \ref scifInit().
  */
static void scifDriverSetupInit(void) {

    // Select SCE clock frequency in active mode
    HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) = AON_PMCTL_AUXSCECLK_SRC_SCLK_HFDIV2;

    // Set the default power mode
    scifSetSceOpmode(AUX_SYSIF_OPMODEREQ_REQ_A);

    // Initialize task resource dependencies
    scifInitIo(24, AUXIOMODE_INPUT, 1, 0);
    scifInitIo(14, AUXIOMODE_OPEN_DRAIN_WITH_INPUT, -1, 1);
    HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[14]) |= IOC_IOCFG0_HYST_EN_M;
    scifInitIo(13, AUXIOMODE_OPEN_DRAIN_WITH_INPUT, -1, 1);
    HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[13]) |= IOC_IOCFG0_HYST_EN_M;
    HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) = (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) & (~AON_PMCTL_AUXSCECLK_PD_SRC_M)) | AON_PMCTL_AUXSCECLK_PD_SRC_SCLK_LF;
    HWREG(AON_RTC_BASE + AON_RTC_O_CTL) |= AON_RTC_CTL_RTC_4KHZ_EN;

} // scifDriverSetupInit




/** \brief Performs driver setup dependent hardware uninitialization
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifDriverSetupUninit(void) {

    // Uninitialize task resource dependencies
    scifUninitIo(24, -1);
    scifUninitIo(14, -1);
    scifUninitIo(13, -1);
    HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) = (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) & (~AON_PMCTL_AUXSCECLK_PD_SRC_M)) | AON_PMCTL_AUXSCECLK_PD_SRC_NO_CLOCK;
    HWREG(AON_RTC_BASE + AON_RTC_O_CTL) &= ~AON_RTC_CTL_RTC_4KHZ_EN;

} // scifDriverSetupUninit




/** \brief Re-initializes I/O pins used by the specified tasks
  *
  * It is possible to stop a Sensor Controller task and let the System CPU borrow and operate its I/O
  * pins. For example, the Sensor Controller can operate an SPI interface in one application state while
  * the System CPU with SSI operates the SPI interface in another application state.
  *
  * This function must be called before \ref scifExecuteTasksOnceNbl() or \ref scifStartTasksNbl() if
  * I/O pins belonging to Sensor Controller tasks have been borrowed System CPU peripherals.
  *
  * \param[in]      bvTaskIds
  *     Bit-vector of task IDs for the task I/Os to be re-initialized
  */
void scifReinitTaskIo(uint32_t bvTaskIds) {
    if (bvTaskIds & (1 << SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID)) {
        scifReinitIo(24, 1, 0);
        scifReinitIo(14, -1, 0);
        HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[14]) |= IOC_IOCFG0_HYST_EN_M;
        scifReinitIo(13, -1, 0);
        HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[13]) |= IOC_IOCFG0_HYST_EN_M;
    }
    if (bvTaskIds & (1 << SCIF_I2C_LIGHT_SENSOR_TASK_ID)) {
        scifReinitIo(14, -1, 0);
        HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[14]) |= IOC_IOCFG0_HYST_EN_M;
        scifReinitIo(13, -1, 0);
        HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[13]) |= IOC_IOCFG0_HYST_EN_M;
    }
} // scifReinitTaskIo




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_INT_DATA_T*) 0x400E0164,
    (volatile SCIF_TASK_CTRL_T*) 0x400E0172,
    (volatile uint16_t*) 0x400E014C,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    0x0008,
    24,
    scifDriverSetupInit,
    scifDriverSetupUninit,
    (volatile uint16_t*) 0x400E015C,
    (volatile uint16_t*) 0x400E0160,
    pRtlTaskStructSignatures
};




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first at the specified value of the RTC
  * and then periodically at the specified interval. The application must call this function after
  * calling \ref scifInit().
  *
  * The application must ensure that:
  * - \a tickStart is not in the past (prefer using \ref scifStartRtcTicksNow() to avoid this)
  * - \a tickPeriod is not too short. RTC ticks will be skipped silently if the Sensor Controller does
  *   not complete its tasks within a single tick interval.
  *
  * \param[in]      tickStart
  *     RTC value when the first tick is generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod) {

    // Configure RTC channel 2
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMP) = tickStart;
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMPINC) = tickPeriod;
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) |= AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M;

    // Prevent glitches to the edge detector when enabling the wake-up source
    HWREG(AUX_SYSIF_BASE + AUX_SYSIF_O_PROGWU0CFG) = AUX_SYSIF_PROGWU0CFG_WU_SRC_AON_RTC_CH2;
    HWREG(AUX_SYSIF_BASE + AUX_SYSIF_O_PROGWU0CFG) = AUX_SYSIF_PROGWU0CFG_WU_SRC_AON_RTC_CH2 | AUX_SYSIF_PROGWU0CFG_EN_M;

} // scifStartRtcTicks




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first after approximately 128 us and
  * then periodically at the specified interval. The application must call this function after calling
  * \ref scifInit().
  *
  * The application must ensure that \a tickPeriod is not too short. RTC ticks will be skipped silently
  * if the Sensor Controller does not complete its tasks within a single tick interval.
  *
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicksNow(uint32_t tickPeriod) {
    uint32_t key, sec, subsec;

    // Read the current RTC value
    key = scifOsalEnterCriticalSection();
    sec = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
    subsec = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);

    // Start RTC tick generation
    scifStartRtcTicks(((sec << 16) | (subsec >> 16)) + 8, tickPeriod);
    scifOsalLeaveCriticalSection(key);

} // scifStartRtcTicksNow




/** \brief Stops RTC-based task scheduling tick generation
  *
  * The application must call this function before calling \ref scifUninit().
  */
void scifStopRtcTicks(void) {

    // Disable RTC channel 2
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) &= ~(AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M);
    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);

    // Prevent glitches to the edge detector when disabling the wake-up source
    HWREG(AUX_SYSIF_BASE + AUX_SYSIF_O_PROGWU0CFG) = AUX_SYSIF_PROGWU0CFG_WU_SRC_AON_RTC_CH2;

} // scifStopRtcTicks


//@}


// Generated by DESKTOP-MRBABVC at 2020-04-29 11:36:32.554