#ifndef CLOSET
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
    /*0x0020*/ 0x8D29, 0xBEFD, 0x4553, 0x2554, 0xAEFE, 0x445C, 0xADB7, 0x745B, 0x545B, 0x7000, 0x7CAC, 0x68B5, 0x00A7, 0x1439, 0x68B6, 0x00A8, 
    /*0x0040*/ 0x1439, 0x68B7, 0x00A9, 0x1439, 0x78AC, 0xF801, 0xFA01, 0xBEF2, 0x78B3, 0x68B5, 0xFD0E, 0x68B7, 0xED92, 0xFD06, 0x7CB3, 0x78B2, 
    /*0x0060*/ 0xFA01, 0xBE05, 0x7002, 0x7CB2, 0x78B2, 0xFA00, 0xBEFD, 0x6440, 0x0487, 0x78AC, 0x8F1F, 0xED8F, 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 
    /*0x0080*/ 0x755B, 0x555B, 0x78B1, 0x60BF, 0xEF27, 0xE240, 0xEF27, 0x7000, 0x7CB1, 0x0487, 0x6477, 0x0000, 0x18B3, 0x9D88, 0x9C01, 0xB60E, 
    /*0x00A0*/ 0x10A6, 0xAF19, 0xAA00, 0xB60A, 0xA8FF, 0xAF39, 0xBE07, 0x0CAC, 0x8600, 0x88A8, 0x8F08, 0xFD47, 0x9DB7, 0x08AC, 0x8801, 0x8A01, 
    /*0x00C0*/ 0xBEEB, 0x254F, 0xAEFE, 0x645B, 0x445B, 0x4477, 0x0487, 0x5656, 0x655B, 0x455B, 0x0000, 0x0CAC, 0x0001, 0x0CAD, 0x152B, 0x0487, 
    /*0x00E0*/ 0x5657, 0x665B, 0x465B, 0x0000, 0x0CAC, 0x0002, 0x0CAD, 0x15C9, 0x0487, 0x5658, 0x675B, 0x475B, 0x0000, 0x0CAC, 0x0004, 0x0CAD, 
    /*0x0100*/ 0x1416, 0x0487, 0x765B, 0x565B, 0x86FF, 0x03FF, 0x0CAF, 0x645C, 0x78AE, 0x68AF, 0xED37, 0xB605, 0x0000, 0x0CAE, 0x7CB4, 0x6540, 
    /*0x0120*/ 0x0CAF, 0x78AF, 0x68B0, 0xFD0E, 0xF801, 0xE95A, 0xFD0E, 0xBE01, 0x6553, 0xBDB7, 0x700B, 0xFB96, 0x4453, 0x2454, 0xAEFE, 0xADB7, 
    /*0x0140*/ 0x6453, 0x2454, 0xA6FE, 0x7000, 0xFB96, 0xADB7, 0x0000, 0x00DB, 0x012A, 0x01DD, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 
    /*0x0160*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0180*/ 0x0000, 0x000C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x01A0*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x000A, 0x77BB, 0xFD47, 0xFD47, 0x57BB, 
    /*0x01C0*/ 0x88FF, 0xBEFA, 0x0000, 0x8B00, 0x66B3, 0x57BA, 0x000A, 0x8B03, 0x001F, 0x8B03, 0x0052, 0x8B03, 0x2408, 0xA6FE, 0x46B3, 0x77BA, 
    /*0x01E0*/ 0x86C3, 0x0350, 0x8801, 0x8B49, 0x8DB1, 0x86C3, 0x0350, 0x8801, 0x8B49, 0x8DB1, 0x66B3, 0x57BA, 0x000A, 0x8B03, 0x002A, 0x8B03, 
    /*0x0200*/ 0x0001, 0x8B03, 0x2408, 0xA6FE, 0x46B3, 0x77BA, 0x66B3, 0x57BA, 0x000A, 0x8B03, 0x002D, 0x8B03, 0x0002, 0x8B03, 0x2408, 0xA6FE, 
    /*0x0220*/ 0x46B3, 0x77BA, 0x0014, 0x8B82, 0x8623, 0x0322, 0x8B7E, 0x0035, 0x8B57, 0x665B, 0x465B, 0x7657, 0x6480, 0x0013, 0x8B56, 0x655B, 
    /*0x0240*/ 0x455B, 0x7656, 0x000B, 0x77BB, 0xFD47, 0xFD47, 0x57BB, 0x88FF, 0xBEFA, 0xADB7, 0xADB7, 0x0001, 0x77BB, 0xFD47, 0xFD47, 0x57BB, 
    /*0x0260*/ 0x88FF, 0xBEFA, 0x08DA, 0x1002, 0x77BB, 0xFD47, 0xFD47, 0x57BB, 0x98FF, 0xBEFA, 0x66B3, 0x57BA, 0x1003, 0x77BB, 0xFD47, 0xFD47, 
    /*0x0280*/ 0x57BB, 0x98FF, 0xBEFA, 0x100B, 0x9B03, 0x100E, 0x9B03, 0x4404, 0x9906, 0xAD41, 0x9D42, 0xADA0, 0x9DA8, 0x9D0A, 0x1CB9, 0x20C2, 
    /*0x02A0*/ 0x9F3A, 0x4404, 0x9906, 0xAD41, 0x9D42, 0xADA0, 0x9DA8, 0x9D0A, 0x1CBC, 0x20CA, 0x9F3A, 0x4404, 0x9906, 0xAD41, 0x9D42, 0xADA0, 
    /*0x02C0*/ 0x9DA8, 0x9D0A, 0x1CBF, 0x20D2, 0x9F3A, 0x2408, 0xA6FE, 0x46B3, 0x77BA, 0x8801, 0x8A08, 0xBE01, 0x0000, 0x0CDA, 0x1000, 0x0000, 
    /*0x02E0*/ 0x20C2, 0xAF1A, 0x9D22, 0x8801, 0x8A08, 0xAEFA, 0x1CBA, 0x1000, 0x0000, 0x20CA, 0xAF1A, 0x9D22, 0x8801, 0x8A08, 0xAEFA, 0x1CBD, 
    /*0x0300*/ 0x1000, 0x0000, 0x20D2, 0xAF1A, 0x9D22, 0x8801, 0x8A08, 0xAEFA, 0x1CC0, 0x08C1, 0x8A00, 0x9E04, 0x08C1, 0x88FF, 0x0CC1, 0x05B1, 
    /*0x0320*/ 0x08BA, 0x8D90, 0x18C0, 0x9D90, 0x2000, 0x8D29, 0x9E01, 0x2001, 0x08BD, 0x8D90, 0x3000, 0x8D29, 0x9E01, 0x3001, 0x08BB, 0xAD28, 
    /*0x0340*/ 0xB604, 0x08AE, 0x8201, 0x0CAE, 0x05AB, 0x08BE, 0xBD28, 0xB603, 0x08AE, 0x8201, 0x0CAE, 0x2CBB, 0x3CBE, 0x0004, 0x18AB, 0x8D01, 
    /*0x0360*/ 0x0CAA, 0x0014, 0x8B82, 0x8623, 0x0322, 0x8B7E, 0x0035, 0x8B57, 0x665B, 0x465B, 0x7657, 0x6480, 0x0013, 0x8B56, 0x655B, 0x455B, 
    /*0x0380*/ 0x7656, 0x0004, 0x77BB, 0xFD47, 0xFD47, 0x57BB, 0x88FF, 0xBEFA, 0xADB7, 0x000C, 0x77BB, 0xFD47, 0xFD47, 0x57BB, 0x88FF, 0xBEFA, 
    /*0x03A0*/ 0x5656, 0x655B, 0x455B, 0x0001, 0x0CB8, 0x08AE, 0x8201, 0x0CAE, 0x0004, 0x18AB, 0x8D01, 0x0CAA, 0xADB7, 0x5656, 0x655B, 0x455B, 
    /*0x03C0*/ 0x5657, 0x665B, 0x465B, 0x66B3, 0x57BA, 0x000A, 0x8B03, 0x002D, 0x8B03, 0x0000, 0x8B03, 0x2408, 0xA6FE, 0x46B3, 0x77BA, 0x000F, 
    /*0x03E0*/ 0x77BB, 0xFD47, 0xFD47, 0x57BB, 0x88FF, 0xBEFA, 0xADB7
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
    0x00000000, 0x00000000, 0x00901170, 0x01A01182  // SPI Accelerometer
};




/// Run-time logging signatures (CRC-16) for each data structure for each task
static const uint16_t pRtlTaskStructSignatures[] = {
    0x0000, 0x0000, 0x9C2C, 0x40FF
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
    scifSetSceOpmode(AUX_SYSIF_OPMODEREQ_REQ_LP);

    // Initialize task resource dependencies
    scifInitIo(19, AUXIOMODE_INPUT, -1, 0);
    scifInitIo(15, AUXIOMODE_OUTPUT | (2 << BI_AUXIOMODE_OUTPUT_DRIVE_STRENGTH), -1, 0);
    scifInitIo(7, ((AUXIOMODE_OUTPUT * (0 == 0)) + (AUXIOMODE_OPEN_DRAIN * (0 == 1))), ((0 == 1) - (0 == 0)), 1);
    HWREG(AUX_SPIM_BASE + AUX_SPIM_O_SPIMCFG) = 0 << AUX_SPIM_SPIMCFG_POL_S;
    HWREG(AUX_SPIM_BASE + AUX_SPIM_O_MOSICTL) = 0;
    HWREG(AUX_SPIM_BASE + AUX_SPIM_O_MISOCFG) = 10;
    scifInitIo(8, AUXIOMODE_PERIPH_OUTPUT_SPIM_SCLK, -1, 0);
    scifInitIo(9, AUXIOMODE_PERIPH_OUTPUT_SPIM_MOSI, -1, 0);
    scifInitIo(10, AUXIOMODE_INPUT_IDLE, -1, 0);
    HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) = (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) & (~AON_PMCTL_AUXSCECLK_PD_SRC_M)) | AON_PMCTL_AUXSCECLK_PD_SRC_SCLK_LF;
    HWREG(AON_RTC_BASE + AON_RTC_O_CTL) |= AON_RTC_CTL_RTC_4KHZ_EN;

} // scifDriverSetupInit




/** \brief Performs driver setup dependent hardware uninitialization
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifDriverSetupUninit(void) {

    // Uninitialize task resource dependencies
    scifUninitIo(19, -1);
    scifUninitIo(15, 0);
    scifUninitIo(7, 1);
    scifUninitIo(8, 0);
    scifUninitIo(9, 0);
    scifUninitIo(10, -1);
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
    if (bvTaskIds & (1 << SCIF_SPI_ACCELEROMETER_TASK_ID)) {
        scifReinitIo(19, -1, 0);
        scifReinitIo(15, -1, 2);
        scifReinitIo(7, 1, 0);
        scifReinitIo(8, -1, 0);
        scifReinitIo(9, -1, 0);
        scifReinitIo(10, -1, 0);
    }
} // scifReinitTaskIo




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_INT_DATA_T*) 0x400E0158,
    (volatile SCIF_TASK_CTRL_T*) 0x400E0166,
    (volatile uint16_t*) 0x400E014C,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    0x0000,
    24,
    scifDriverSetupInit,
    scifDriverSetupUninit,
    (volatile uint16_t*) 0x400E0154,
    (volatile uint16_t*) 0x400E0156,
    pRtlTaskStructSignatures
};




// No task-specific API available


//@}


// Generated by SANWINA0221118 at 2019-06-13 16:12:30.899
#endif
