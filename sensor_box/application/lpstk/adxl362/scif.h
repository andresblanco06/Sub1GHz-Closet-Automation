#ifdef !SCS
/** \mainpage Driver Overview
  *
  * \section section_drv_info Driver Information
  * This Sensor Controller Interface driver has been generated by the Texas Instruments Sensor Controller
  * Studio tool:
  * - <b>Project name</b>:     SPI Accelerometer for ULPSENSE
  * - <b>Project file</b>:     C:/Users/a0221118/Documents/Texas Instruments/Sensor Controller Studio/examples/spi_accelerometer_ulpsense/spi_accelerometer_ulpsense.scp
  * - <b>Code prefix</b>:      -
  * - <b>Operating system</b>: TI-RTOS
  * - <b>Tool version</b>:     2.4.0.793
  * - <b>Tool patches</b>:     None
  * - <b>Target chip</b>:      CC1352R1F3, package QFN48 7x7 RGZ, revision E (2.1)
  * - <b>Created</b>:          2019-06-13 16:12:30.899
  * - <b>Computer</b>:         SANWINA0221118
  * - <b>User</b>:             a0221118
  *
  * No user-provided resource definitions were used to generate this driver.
  *
  * No user-provided procedure definitions were used to generate this driver.
  *
  * Do not edit the generated source code files other than temporarily for debug purposes. Any
  * modifications will be overwritten by the Sensor Controller Studio when generating new output.
  *
  * \section section_drv_modules Driver Modules
  * The driver is divided into three modules:
  * - \ref module_scif_generic_interface, providing the API for:
  *     - Initializing and uninitializing the driver
  *     - Task control (for starting, stopping and executing Sensor Controller tasks)
  *     - Task data exchange (for producing input data to and consume output data from Sensor Controller
  *       tasks)
  * - \ref module_scif_driver_setup, containing:
  *     - The AUX RAM image (Sensor Controller code and data)
  *     - I/O mapping information
  *     - Task data structure information
  *     - Driver setup data, to be used in the driver initialization
  *     - Project-specific functionality
  * - \ref module_scif_osal, for flexible OS support:
  *     - Interfaces with the selected operating system
  *
  * It is possible to use output from multiple Sensor Controller Studio projects in one application. Only
  * one driver setup may be active at a time, but it is possible to switch between these setups. When
  * using this option, there is one instance of the \ref module_scif_generic_interface and
  * \ref module_scif_osal modules, and multiple instances of the \ref module_scif_driver_setup module.
  * This requires that:
  * - The outputs must be generated using the same version of Sensor Controller Studio
  * - The outputs must use the same operating system
  * - The outputs must use different source code prefixes (inserted into all globals of the
  *   \ref module_scif_driver_setup)
  *
  *
  * \section section_project_info Project Description
  * Demonstrates low-power sampling of an SPI accelerometer (ADXL362) on LaunchPad with the ULP Sense
  * BoosterPack (BOOSTXL-ULPSENSE), with simple processing.
  * 
  * The accelerometer samples autonomously at 100 Hz, and generates interrupt to the Sensor Controller.
  * The Sensor Controller reads the accelerometer and performs simple filtering and tilt detection.
  * 
  * The default sampling frequency is 100 Hz (configured accelermeter samping frequency).
  * 
  * Tilt detection is communicated to the System CPU application, which enables an LED on the LaunchPad
  * when tilt is detected for each axis.
  * 
  * Task Testing should only be used for debugging purposes, as it will not run the SPI Accelerometer
  * task at the correct rate. Use Run-Time Logging for testing, performance evaluation and tuning.
  * 
  * BOOSTXL-ULPSENSE SETUP:
  * - Mount SENSOR POWER jumper: ACC
  * - Remove all other jumpers
  * 
  * The accelerometer orientation is printed on the board.
  * 
  * See the header in the application source file ("main_tirtos.c") for further details and instructions.
  * This file is located in the source code output directory.
  *
  *
  * \subsection section_io_mapping I/O Mapping
  * Task I/O functions are mapped to the following pins:
  * - SPI Accelerometer:
  *     - <b>I: Accelerometer interrupt pin</b>: DIO30
  *     - <b>O: Debug</b>: DIO3
  *     - <b>SPI CSN: Accelerometer</b>: DIO11
  *     - <b>SPI MISO</b>: DIO8
  *     - <b>SPI MOSI</b>: DIO9
  *     - <b>SPI SCLK</b>: DIO10
  *
  *
  * \section section_task_info Task Description(s)
  * This driver supports the following task(s):
  *
  *
  * \subsection section_task_desc_spi_accelerometer SPI Accelerometer
  * Samples the ADXL362 accelerometer on the ULP Sense BoosterPack, and performs simple filtering and
  * tilt detection.
  * 
  * The accelerometer is used with default configuration (2g, 100 Hz sampling), with data ready interrupt
  * on the INT1 pin.
  * The data ready interrupt triggers the Event Handler A Code, which:
  * - Reads the values for each channel (X, Y and Z)
  * - Uses a sliding window for each channel to filter the values
  * - Performs simple tilt detection on the filtered values: abs(X) > abs(Z), and abs(Y) > abs(Z)
  * 
  * The Event Handler B Code implements data ready interrupt timeout, with error indication. The Event
  * Handler A Code restarts the timeout.
  * Timeout occurs if the data ready interrupt interval exceeds 20 ms (expected interval is 10 ms).
  * 
  * Task Testing should only be used for debugging purposes, as it will not run the SPI Accelerometer
  * task at the correct rate. Use Run-Time Logging for testing, performance evaluation and tuning.
  * 
  * BOOSTXL-ULPSENSE SETUP:
  * - Mount SENSOR POWER jumper: ACC
  * - Remove all other jumpers
  * 
  * The accelerometer orientation is printed on the board.
  *
  */




/** \addtogroup module_scif_driver_setup Driver Setup
  *
  * \section section_driver_setup_overview Overview
  *
  * This driver setup instance has been generated for:
  * - <b>Project name</b>:     SPI Accelerometer for ULPSENSE
  * - <b>Code prefix</b>:      -
  *
  * The driver setup module contains the generated output from the Sensor Controller Studio project:
  * - Location of task control and scheduling data structures in AUX RAM
  * - The AUX RAM image, and the size the image
  * - Task data structure information (location, size and buffer count)
  * - I/O pin mapping translation table
  * - Task resource initialization and uninitialization functions
  * - Hooks for run-time logging
  *
  * @{
  */
#ifndef SCIF_H
#define SCIF_H

#include <stdint.h>
#include <stdbool.h>
#include "scif_framework.h"
#include "scif_osal_tirtos.h"


/// Target chip name
#define SCIF_TARGET_CHIP_NAME_CC1352R1F3
/// Target chip package
#define SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ

/// Number of tasks implemented by this driver
#define SCIF_TASK_COUNT 1

/// SPI Accelerometer: Task ID
#define SCIF_SPI_ACCELEROMETER_TASK_ID 0


/// SPI Accelerometer: Filter control: Use halved bandwidth
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_HALF_BW 16
/// SPI Accelerometer: Filter control: Output data rate 100 Hz
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_ODR_100_HZ 3
/// SPI Accelerometer: Filter control: Output data rate 12.5 Hz
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_ODR_12P5_HZ 0
/// SPI Accelerometer: Filter control: Output data rate 200 Hz
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_ODR_200_HZ 4
/// SPI Accelerometer: Filter control: Output data rate 25 Hz
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_ODR_25_HZ 1
/// SPI Accelerometer: Filter control: Output data rate 400 Hz
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_ODR_400_HZ 5
/// SPI Accelerometer: Filter control: Output data rate 50 Hz
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_ODR_50_HZ 2
/// SPI Accelerometer: Filter control: Output data rate 800 Hz
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_ODR_800_HZ 6
/// SPI Accelerometer: Filter control: Selects range +/- 2 g
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_RANGE_2_G 0
/// SPI Accelerometer: Filter control: Selects range +/- 4 g
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_RANGE_4_G 64
/// SPI Accelerometer: Filter control: Selects range +/- 8 g
#define SCIF_SPI_ACCELEROMETER_ACCEL_FC_RANGE_8_G 128
/// SPI Accelerometer: Interrupt mapping: Activity status
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_ACT 16
/// SPI Accelerometer: Interrupt mapping: Awake status
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_AWAKE 64
/// SPI Accelerometer: Interrupt mapping: Data ready status
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_DATA_READY 1
/// SPI Accelerometer: Interrupt mapping: FIFO overrun status
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_FIFO_OVERRUN 8
/// SPI Accelerometer: Interrupt mapping: FIFO ready status
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_FIFO_READY 2
/// SPI Accelerometer: Interrupt mapping: FIFO watermark status
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_FIFO_WATERMARK 4
/// SPI Accelerometer: Interrupt mapping: Inactivity status
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_INACT 32
/// SPI Accelerometer: Interrupt mapping: Active low
#define SCIF_SPI_ACCELEROMETER_ACCEL_IM_INT_LOW 128
/// SPI Accelerometer: Power control: Selects normal operation
#define SCIF_SPI_ACCELEROMETER_ACCEL_PC_LOW_NOISE_0 0
/// SPI Accelerometer: Power control: Selects low noise operation
#define SCIF_SPI_ACCELEROMETER_ACCEL_PC_LOW_NOISE_1 16
/// SPI Accelerometer: Power control: Selects ultralow noise operation
#define SCIF_SPI_ACCELEROMETER_ACCEL_PC_LOW_NOISE_2 32
/// SPI Accelerometer: Power control: Selects standby mode
#define SCIF_SPI_ACCELEROMETER_ACCEL_PC_MEASURE_OFF 0
/// SPI Accelerometer: Power control: Selects standby mode
#define SCIF_SPI_ACCELEROMETER_ACCEL_PC_MEASURE_ON 2
/// SPI Accelerometer: Accelerometer register address: FILTER_CTL
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_FILTER_CTL 44
/// SPI Accelerometer: Accelerometer register address: INTMAP1
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_INTMAP1 42
/// SPI Accelerometer: Accelerometer register address: PARTID
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_PARTID 2
/// SPI Accelerometer: Accelerometer register address: POWER_CTL
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_POWER_CTL 45
/// SPI Accelerometer: Accelerometer register address: REVID
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_REVID 3
/// SPI Accelerometer: Accelerometer register address: SOFT_RESET
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_SOFT_RESET 31
/// SPI Accelerometer: Accelerometer register address: STATUS
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_STATUS 11
/// SPI Accelerometer: Accelerometer register address: XDATAL
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_XDATAL 14
/// SPI Accelerometer: Accelerometer register address: YDATAL
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_YDATAL 16
/// SPI Accelerometer: Accelerometer register address: ZDATAL
#define SCIF_SPI_ACCELEROMETER_ACCEL_REG_ZDATAL 18
/// SPI Accelerometer: Accelerometer read operation
#define SCIF_SPI_ACCELEROMETER_ACCEL_SPI_READ 11
/// SPI Accelerometer: Accelerometer write operation
#define SCIF_SPI_ACCELEROMETER_ACCEL_SPI_WRITE 10
/// SPI Accelerometer: Software reset: Trigger key
#define SCIF_SPI_ACCELEROMETER_ACCEL_SR_KEY 82
/// SPI Accelerometer: Size of the X, Y and Z sample windows
#define SCIF_SPI_ACCELEROMETER_XYZ_WINDOW_SIZE 8
/// SPI Accelerometer I/O mapping: Accelerometer interrupt pin
#define SCIF_SPI_ACCELEROMETER_DIO_I_ACCEL_IRQ 30
/// SPI Accelerometer I/O mapping: Debug
#define SCIF_SPI_ACCELEROMETER_DIO_O_DEBUG 3
/// SPI Accelerometer I/O mapping: Accelerometer
#define SCIF_SPI_ACCELEROMETER_DIO_SPI_CSN_ACCEL 11
/// SPI Accelerometer I/O mapping: SPI MISO
#define SCIF_SPI_ACCELEROMETER_DIO_SPI_MISO 8
/// SPI Accelerometer I/O mapping: SPI MOSI
#define SCIF_SPI_ACCELEROMETER_DIO_SPI_MOSI 9
/// SPI Accelerometer I/O mapping: SPI SCLK
#define SCIF_SPI_ACCELEROMETER_DIO_SPI_SCLK 10


// All shared data structures in AUX RAM need to be packed
#pragma pack(push, 2)


/// SPI Accelerometer: Task output data structure
typedef struct {
    uint16_t accelError; ///< Accelerometer error (no interrupt)
    int16_t  x;          ///< For observation: Latest X-axis sample
    int16_t  xFiltered;  ///< For observation: Sum of all samples in the X-axis sample window
    uint16_t xTiltDet;   ///< 1 when tilting along the X-axis is detected, otherwise 0
    int16_t  y;          ///< For observation: Latest Y-axis sample
    int16_t  yFiltered;  ///< For observation: Sum of all samples in the Y-axis sample window
    uint16_t yTiltDet;   ///< 1 when tilting along the Y-axis is detected, otherwise 0
    int16_t  z;          ///< For observation: Latest Z-axis sample
    int16_t  zFiltered;  ///< For observation: Sum of all samples in the Z-axis sample window
} SCIF_SPI_ACCELEROMETER_OUTPUT_T;


/// SPI Accelerometer: Task state structure
typedef struct {
    uint16_t initIgnoreSmpl; ///< Used to ignore the first invalid samples, and then fill the sample windows
    int16_t  pXWindow[8];    ///< X-axis sample window, used for filtering
    int16_t  pYWindow[8];    ///< Y-axis sample window, used for filtering
    int16_t  pZWindow[8];    ///< Z-axis sample window, used for filtering
    uint16_t windowPos;      ///< Sample window position
} SCIF_SPI_ACCELEROMETER_STATE_T;


/// Sensor Controller task data (configuration, input buffer(s), output buffer(s) and internal state)
typedef struct {
    struct {
        SCIF_SPI_ACCELEROMETER_OUTPUT_T output;
        SCIF_SPI_ACCELEROMETER_STATE_T state;
    } spiAccelerometer;
} SCIF_TASK_DATA_T;

/// Sensor Controller task generic control (located in AUX RAM)
#define scifTaskData    (*((volatile SCIF_TASK_DATA_T*) 0x400E0170))


// Initialized internal driver data, to be used in the call to \ref scifInit()
extern const SCIF_DATA_T scifDriverSetup;


// Restore previous struct packing setting
#pragma pack(pop)


// AUX I/O re-initialization functions
void scifReinitTaskIo(uint32_t bvTaskIds);


// No task-specific API available


#endif
//@}


// Generated by SANWINA0221118 at 2019-06-13 16:12:30.899
#endif