/* COMPILER DIRECTIVES */

/* PRIVATE COMPILE SWITCHES */

/* INCLUDE FILES */
#include "GDTypes.h"
#include "AppModules/AppModules.h"
#if (APP_MODULES_CONFIG__ENABLE_PM_02_00_GENERIC_SENSOR_CONTROLLER == GD_TRUE)
#include "GenericSensorController/GenericSensorControllerConfig.h"
#include "MagnetometerSensor_AK09973D/MagnetometerSensor_AK09973DTest.h"
#include "MagnetometerSensor_AK09973D/MagnetometerSensor_AK09973D.h"
#include "MagnetometerSensor_AK09973D/MagnetometerSensor_AK09973DConfig.h"
#if (GENERIC_SENSOR_CONTROLLER_CONFIG__COMPILE_TESTS == GD_TRUE)
#if (MAGNETOMETER_SENSOR_AK09973D_CONFIG__COMPILE_TESTS == GD_TRUE)
#include "GenericSensorController/GenericSensorController.h"
#include "GenericSensorController/GenericSensorControllerTest.h"
#include "MessagesUtility/MessagesUtility.h"
#include "UnitTestManager/UnitTestManager.h"
#include "TimerUtility/TimerUtility.h"
#include "WatchDogInterface/WatchDogInterface.h"
#include "PIOSErrorInterface/PIOSErrorInterface.h"
#include <math.h>
/* PRIVATE LITERAL DEFINITIONS */


/* PRIVATE TYPE DEFINITIONS */


/* PRIVATE CONSTANT DEFINITIONS */


/* PUBLIC CONSTANT DEFINITIONS */


/* PRIVATE VARIABLE DEFINITIONS - defined as static */

//! Test sample of one magnetometer reading for unit test, value:[-0.0010, -0.1815, -0.6193]mT
static magnetometerAk09973D_t TestRawConvertedMagReading;

//! Test sample of one magnetometer reading for unit test, raw sample value:
//!Z MSB:0XFD, Z LSB:0XCD ,Y MSB:0XFF, Y LSB:0X5B , X MSB:0XFF, X LSB:0XFF
static const magnetometerAk09973DRawData_t TestRawSample =
{
   .magneticXAxisMsb = 0XFF,
   .magneticXAxisLsb = 0XFF,
   .magneticYAxisMsb = 0XFF,
   .magneticYAxisLsb = 0X5B,
   .magneticZAxisMsb = 0XFD,
   .magneticZAxisLsb = 0XCD,
};

//! Test sample of one magnetometer reading for unit test,
//!raw sample register read value in an array.
static const uint8_t TestRawSampleInArray[MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_DATA_LENGTH +
                         MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_STATUS_BYTE_LEN +
                         MAGNETOMETER_SENSOR_AK09973D__READ_MAGNETIC_FIELD_REGISTER_PADDING0_LEN] =
                        {
                           (uint8_t)0x01,
                           (uint8_t)0xfd,
                           (uint8_t)0xcd,
                           (uint8_t)0xff,
                           (uint8_t)0x5b,
                           (uint8_t)0xff,
                           (uint8_t)0xff,
                           (uint8_t)0x00,
                        };

//! Test sample of one magnetometer reading for unit test, combined two bytes raw sample value
//! to form one magnetic field reading in one axis.
static const int16_t TestMagneticXAxis = (int16_t)0xffffU;
static const int16_t TestMagneticYAxis = (int16_t)0xff5bU;
static const int16_t TestMagneticZAxis = (int16_t)0xfdcdU;

/* PUBLIC VARIABLE DEFINITIONS */


/* PRIVATE FUNCTION DECLARATIONS */
/**
 * @brief      AK09973D Sensor register block reading.
 * @param      Channel - Channel of Generic Sensor MagnetometerSensor_AK09973D is on.
 * @param      Data - register address and value to read from
 * @param      OutputData - Destination data pointer
 * @param      Size - Destination data size
 * @return     GD_TRUE if successful, GD_FALSE if not.
 */
static uint8_t RegisterReadBlockingUnitTest(
   uint16_t Channel, const genericSensorRegister_t *Data, uint8_t *OutputData, uint16_t Size);

/* PRIVATE FUNCTION DEFINITIONS */
/*
 * See above for doco
 *
 */
static uint8_t RegisterReadBlockingUnitTest(
   uint16_t Channel, const genericSensorRegister_t *Data, uint8_t *OutputData, uint16_t Size)
{
   uint8_t retValue = GD_TRUE;
   uint8_t breakLoop = GD_FALSE;
   uint16_t instance = 0xFFFFU;
   for (uint16_t index = 0U; ((index < MAGNETOMETER_SENSOR_AK09973D_CONFIG__INSTANCE_MAX) && (breakLoop == GD_FALSE));
        index++)
   {
      if (MagnetometerSensor_AK09973DConfig__Channel_gro[index] == Channel)
      {
         instance = index;
         breakLoop = GD_TRUE;
      }
      else
      {
         // Nothing
      }
   }

   if (instance == 0xFFFFU)
   {
      retValue = GD_FALSE;
   }
   else
   {
      // initiate the sensor slave with appropriate I2CMIF driver instance and index.
      MagnetometerSensor_AK09973DConfig__Setting_g.deviceInterfaceInstance =
         (uint8_t)MagnetometerSensor_AK09973DConfig__Instance_gro[instance];
      MagnetometerSensor_AK09973DConfig__Setting_g.deviceInterfaceIndex =
         (uint8_t)MagnetometerSensor_AK09973DConfig__Index_gro[instance];
      genoError_t error;
      uint8_t timerExpired = GD_FALSE;
      uint8_t readComplete = GD_FALSE;
      genoTimer_t readTimeOutTimer;
      i2cTransactionInfo_t genericSensorTransfer;
      i2cControllerStatus_t localStatus;
      // update current GenericSensorControllerI2cController I2C driver instance
      // to this sensor's I2C instance, also update to TMP117's slaveAddress
      // for following I2C communication
      const genericSensor_t *sensor = &GenericSensors_gro[Channel];
      GenericSensorControllerI2cController.instanceNo = sensor->settings->deviceInterfaceInstance;
      GenericSensorControllerI2cController.slaveAddress =
         i2cSlaveConfig_gro[sensor->settings->deviceInterfaceIndex];

      genericSensorTransfer.controllerObject = &GenericSensorControllerI2cController;
      genericSensorTransfer.writeDataBuffer = NULL;
      genericSensorTransfer.numBytesWrite = 0U;
      genericSensorTransfer.numBytesRead = (uint16_t) Size;
      genericSensorTransfer.wordAddress = Data->registerAddress;
      genericSensorTransfer.wordAddressSize = sensor->settings->registerAddressSize;
      genericSensorTransfer.disableTimeout = GD_FALSE;
      genericSensorTransfer.enablePageWrite = GD_FALSE;
      genericSensorTransfer.priority = I2CM__MEDIUM_PRIORITY;

      #if (MAGNETOMETER_SENSOR_AK09973D_CONFIG_CONFIG__ENABLE_DEBUG == GD_TRUE)
      MessagesUtility__DebugNoTimeStamp(
         MESSAGES_UTILITY__IMPORTANT_MESSAGE,
         "Data->registerAddress:%02x, registerAddressSize  %02x, numBytesRead = %02x\r\n",
         genericSensorTransfer.wordAddress,
         genericSensorTransfer.wordAddressSize,
         genericSensorTransfer.numBytesRead);
      #endif

      PIOS_I2CMInterface_write(&genericSensorTransfer);
      // Block read with an expiry timer:
      // give > 500us (2ms is needed from experiement) expiry timer for I2C to have enough time
      // to complete read.
      // This is because, I2C read two bytes register takes > 500us, and hence I2CMIF driver
      // requires >500us before its read data compelte.
      TimerUtility__SetTimer(&readTimeOutTimer,
            (uint64_t) MAGNETOMETER_SENSOR_AK09973D__I2CMIF_READ_PROCESS_PERIOD_BEFORE_READ_DATA_READY);

      while ((timerExpired == GD_FALSE) && (readComplete == GD_FALSE))
      {
         PIOS_I2CMInterface_read_status(&GenericSensorControllerI2cController,&localStatus);
         if (localStatus.transactionStatus == I2CM__CONTROLLER_STATE_DATA_READY)
         {
            readComplete = GD_TRUE;
         }
         else
         {
            // Nothing
         }
         timerExpired = TimerUtility__HasTimerExpired(&readTimeOutTimer);
      }
      if (readComplete == GD_TRUE)
      {
         i2cReturnBufferData_t localreturn;
         localreturn.controllerObject = &GenericSensorControllerI2cController;
         localreturn.readDataBuffer = OutputData;
         localreturn.numBytesRead = (uint16_t) Size;

         PIOSErrorInterface__ClearPIOSError();
         PIOS_I2CMInterface_read(&localreturn);
         uint8_t success;
         success = PIOSErrorInterface__CheckForPIOSError();

         if (success == GD_FALSE)
         {
            success = GD_TRUE;
         }
         else
         {
            success = GD_FALSE;
         }

         if (success == GD_TRUE)
         {
            // Read was successful
         }
         else
         {
            Error__Report(&error, ERROR__DOMAIN_PIOS_MIDDLEWARE,
               APP_MODULES__PM_02_00_GENERIC_SENSOR_CONTROLLER_INDEX,
               GENERICSENSORCONTROLLER__INIT_FAIL_ERROR, ERROR__LEVEL_FAULT);
            retValue = GD_FALSE;
         }
      }
      else if (timerExpired == GD_TRUE)
      {
         // Timer has expired and read is not complete
         Error__Report(&error, ERROR__DOMAIN_PIOS_MIDDLEWARE,
            APP_MODULES__PM_02_00_GENERIC_SENSOR_CONTROLLER_INDEX,
            GENERICSENSORCONTROLLER__INIT_FAIL_ERROR, ERROR__LEVEL_FAULT);
         retValue = GD_FALSE;
      }
      else
      {
         // It is not logically possible to reach this state, however we
         // will report an error in case we have screwed this up.
         Error__Report(&error, ERROR__DOMAIN_PIOS_MIDDLEWARE,
            APP_MODULES__PM_02_00_GENERIC_SENSOR_CONTROLLER_INDEX,
            GENERICSENSORCONTROLLER__INIT_FAIL_ERROR, ERROR__LEVEL_FAULT);
         retValue = GD_FALSE;
      }
   }
   return retValue;
}
/* PUBLIC FUNCTION IMPLEMENTATIONS */

/*
 * See header for doco
 */
uint8_t MagnetometerSensor_AK09973DTest__Init_Test(uint16_t Channel)
{
   uint8_t retValue;
   uint8_t testResult;
   uint16_t instance = 0xFFFFU;
   uint8_t breakLoop = GD_FALSE;

   MessagesUtility__DebugNoTimeStamp(
      MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing MagnetometerSensor_AK09973DTest__Init_Test\r\n\r\n");

   for (uint16_t index = 0U; ((index < MAGNETOMETER_SENSOR_AK09973D_CONFIG__INSTANCE_MAX) && (breakLoop == GD_FALSE));
        index++)
   {
      if (MagnetometerSensor_AK09973DConfig__Channel_gro[index] == Channel)
      {
         instance = index;
         breakLoop = GD_TRUE;
      }
      else
      {
         // Nothing
      }
   }

   if (instance == 0xFFFFU)
   {
      testResult = UNIT_TEST_MANAGER__TEST_FAIL;
   }
   else
   {
      // Check if user configured correct hardware sensor regiseter and value to write
      uint32_t allowedInitDataReadyRegVal =
            MAGNETOMETER_SENSOR_AK09973D__DATA_READY_EVENT_ENABLE;
      uint32_t allowedInitModeRegVal =
            (uint32_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_SINGLE_MEASUREMENT <<
            MAGNETOMETER_SENSOR_AK09973D__ONE_BYTE_REGISTER_VAL_SHIFT_2_STANDARD_TWO_BYTE_REGISTER_MSB;

      uint32_t configuredInitDataReadyRegAddr =
         MagnetometerSensor_AK09973DConfig__InitRegister_gro[0U].registerAddress;
      uint32_t configuredInitModeRegAddr =
         MagnetometerSensor_AK09973DConfig__InitRegister_gro[1U].registerAddress;
      uint32_t configuredInitDataReadyRegVal =
         MagnetometerSensor_AK09973DConfig__InitRegister_gro[0U].registerValue;
      uint32_t configuredInitModeRegVal =
         MagnetometerSensor_AK09973DConfig__InitRegister_gro[1U].registerValue;

      if((configuredInitDataReadyRegAddr ==
         MAGNETOMETER_SENSOR_AK09973D__CONFIG_CONTROL1_REGISTER) &&
         (configuredInitDataReadyRegVal == allowedInitDataReadyRegVal) &&
         (configuredInitModeRegAddr ==
         MAGNETOMETER_SENSOR_AK09973D__CONFIG_CONTROL2_REGISTER) &&
         (configuredInitModeRegVal == allowedInitModeRegVal))
      {
         // Check init API error reporting
         genoError_t error;
         Error__Init(&error);
         Error__Clear();
         MessagesUtility__DebugNoTimeStamp(
            MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid channel max\r\n");
         retValue = MagnetometerSensor_AK09973D__Init(0xFFFFU);
         error = Error__Get();
         if ((retValue == GD_FALSE)&&
            (error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
         {
            retValue = GD_TRUE;
         }
         else
         {
            retValue = GD_FALSE;
         }

         // Check each sub-test case to report final test outcome
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Checking if process completed successfully\r\n");

         if(retValue == GD_TRUE)
         {
            testResult = UNIT_TEST_MANAGER__TEST_PASS;
            MessagesUtility__DebugNoTimeStamp( MESSAGES_UTILITY__IMPORTANT_MESSAGE, "init magnetometerAk09973 success!\r\n");
         }
         else
         {
            testResult = UNIT_TEST_MANAGER__TEST_FAIL;
         }

      }
      else
      {
         MessagesUtility__DebugNoTimeStamp( MESSAGES_UTILITY__IMPORTANT_MESSAGE, " "
         "invalid user configured magnetometerAk09973 init registers!\r\n");
         testResult = UNIT_TEST_MANAGER__TEST_FAIL;
      }
   }

   // Nothing to test
   return testResult;
}

/*
 * See header for doco
 */
uint8_t MagnetometerSensor_AK09973DTest__Deinit_Test(uint16_t Channel)
{
   uint8_t retValue;
   genoTimer_t timer;
   uint8_t timerExpired;
   uint8_t testResult = UNIT_TEST_MANAGER__TEST_PASS;
   timerExpired = GD_FALSE;
   uint8_t measurementRegVal[MAGNETOMETER_SENSOR_AK09973D__TOTAL_REG_SIZE_IN_READ_REGISTER_PROCESS] = {0U};
   uint16_t registerReadLen = (uint16_t)MAGNETOMETER_SENSOR_AK09973D__TOTAL_REG_SIZE_IN_READ_REGISTER_PROCESS;
   uint16_t instance = 0xFFFFU;
   uint8_t breakLoop = GD_FALSE;
   genericSensorRegister_t readMeasurementRegister =
   {
      .registerAddress = MAGNETOMETER_SENSOR_AK09973D__READ_3_AXES_MEASUREMENT_REGISTER,
      .registerValue = 0x00,
      .setOnInitialise = GD_FALSE,
   };

   MessagesUtility__DebugNoTimeStamp(
      MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing MagnetometerSensor_AK09973DTest__Deinit_Test\r\n\r\n");
   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Nothing to test\r\n");
   MessagesUtility__DebugNoTimeStamp(
      MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Verify module deinit function\r\n");

   for (uint16_t index = 0U; ((index < MAGNETOMETER_SENSOR_AK09973D_CONFIG__INSTANCE_MAX) && (breakLoop == GD_FALSE));
        index++)
   {
      if (MagnetometerSensor_AK09973DConfig__Channel_gro[index] == Channel)
      {
         instance = index;
         breakLoop = GD_TRUE;
      }
      else
      {
         // Nothing
      }
   }

   if (instance == 0xFFFFU)
   {
      retValue = GD_FALSE;
   }
   else
   {
      // Check if user configured correct hardware sensor power down mode regiseter and value to
      // write to sensor.
      uint32_t allowedDeinitResetRegVal = (uint32_t)MAGNETOMETER_SENSOR_AK09973D__SRST_SOFT_RESET_VALUE <<
            MAGNETOMETER_SENSOR_AK09973D__ONE_BYTE_REGISTER_VAL_SHIFT_2_STANDARD_TWO_BYTE_REGISTER_MSB;
      uint32_t allowedDeinitModeRegVal = (uint32_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_POWER_DOWN <<
            MAGNETOMETER_SENSOR_AK09973D__ONE_BYTE_REGISTER_VAL_SHIFT_2_STANDARD_TWO_BYTE_REGISTER_MSB;
      uint32_t configuredDeinitResetRegAddr =
         MagnetometerSensor_AK09973DConfig__DeinitRegister_gro[0U].registerAddress;
      uint32_t configuredDeinitModeRegAddr =
         MagnetometerSensor_AK09973DConfig__DeinitRegister_gro[1U].registerAddress;
      uint32_t configuredDeinitResetRegVal =
         MagnetometerSensor_AK09973DConfig__DeinitRegister_gro[0U].registerValue;
      uint32_t configuredDeinitModeRegVal =
         MagnetometerSensor_AK09973DConfig__DeinitRegister_gro[1U].registerValue;

      if((configuredDeinitResetRegAddr
       == MAGNETOMETER_SENSOR_AK09973D__SRST_SOFT_RESET_REGISTER) &&
       (configuredDeinitResetRegVal == allowedDeinitResetRegVal) &&
         (configuredDeinitModeRegAddr
       == MAGNETOMETER_SENSOR_AK09973D__CONFIG_CONTROL2_REGISTER) &&
       (configuredDeinitModeRegVal == allowedDeinitModeRegVal))
      {
         // Check if hardware sensor is in power down mode
         // This verificaiton is done assuming that if I2C is not raising any communication error
         // then Power down regiseter is written successful.
         // To check if deini API call raises I2C write Power Down mode register error,
         // if not, assume I2C configure wirte is
         // success because I2C acknowlegement bit indicates write success.
         retValue = GenericSensorController__Deinit(Channel);
         if (retValue == GD_TRUE)
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "Sampling and processing at least one sample by GenericSensorController__Process\r\n\r\n");
            WatchDogInterface__Reload();
            TimerUtility__SetTimer(&timer,
               MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);
            while (timerExpired == GD_FALSE)
            {
               timerExpired = TimerUtility__HasTimerExpired(&timer);
               GenericSensorController__Process(Channel);
            }
            // After deini API sends soft reset command, read a register to check
            // if sensor register values has been reset
            retValue = RegisterReadBlockingUnitTest(
               instance,
               &readMeasurementRegister,
               measurementRegVal, registerReadLen);
            // Check if module internal states -DeviceInfo[][] and AK09973DInterruptTriggeredCount-
            // have been reset to reset state
            // i.e. sensor still responds on I2C but, with all zero value
            if (retValue == GD_TRUE)
            {
               uint8_t statusVal= measurementRegVal[0U];
               int8_t  interruptCount = AK09973DInterruptTriggeredCount[instance];
               uint8_t companyId = DeviceInfo[instance][MAGNETOMETER_SENSOR_AK09973D__COMPANY_ID_IDX];
               uint8_t deviceId = DeviceInfo[instance][MAGNETOMETER_SENSOR_AK09973D__DEVICE_ID_IDX];
               MessagesUtility__DebugNoTimeStamp(
                  MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                  "Read sensor measurement sample register:\r\n");
               MessagesUtility__DebugNoTimeStamp(
                        MESSAGES_UTILITY__IMPORTANT_MESSAGE,  "0x%02X\r\n", statusVal);
               if ((statusVal!= (uint8_t)MAGNETOMETER_SENSOR_AK09973D__STATUS_BYTE_DEFAULT_VAL) ||
                   (interruptCount != 0) ||
                   (companyId != 0U) ||
                   (deviceId != 0U))
               {
                   retValue = GD_FALSE;
                   testResult = UNIT_TEST_MANAGER__TEST_FAIL;
               }
               else
               {
                  // first byte is correct
                  // don't care first and last byte
                  for(uint32_t i=MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_Z_AXIS_IDX;
                     i<
                     (MAGNETOMETER_SENSOR_AK09973D__TOTAL_REG_SIZE_IN_READ_REGISTER_PROCESS -
                     MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_STATUS_BYTE_LEN);
                     i++)
                  {
                     MessagesUtility__DebugNoTimeStamp(
                        MESSAGES_UTILITY__IMPORTANT_MESSAGE,  "0x%02X\r\n", measurementRegVal[i]);
                     if(measurementRegVal[i] == (uint8_t)0x00)
                     {
                        retValue = GD_TRUE;
                     }
                     else
                     {
                        retValue = GD_FALSE;
                        break;
                     }
                  }
                  if(retValue == GD_FALSE)
                  {
                     MessagesUtility__DebugNoTimeStamp( MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Deinit magnetometerAk09973 failed!\r\n");
                     testResult = UNIT_TEST_MANAGER__TEST_FAIL;
                  }
                  else
                  {
                     MessagesUtility__DebugNoTimeStamp( MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Deinit magnetometerAk09973 success!\r\n");
                  }
               }
            }
            else
            {
               retValue = GD_FALSE;
               MessagesUtility__DebugNoTimeStamp( MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Deinit magnetometerAk09973 returned false!\r\n");
               testResult = UNIT_TEST_MANAGER__TEST_FAIL;
            }
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp( MESSAGES_UTILITY__IMPORTANT_MESSAGE, " deinie API returned false!\r\n");
            testResult = UNIT_TEST_MANAGER__TEST_FAIL;
            retValue = GD_FALSE;
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp( MESSAGES_UTILITY__IMPORTANT_MESSAGE,
         "MagnetometerSensor_AK09973DConfig__DeinitRegister_gro register is configured with wrong value!\r\n");
         testResult = UNIT_TEST_MANAGER__TEST_FAIL;
         retValue = GD_FALSE;
      }
   }

   if (retValue == GD_TRUE)
   {
      genoError_t error;
      Error__Init(&error);
      Error__Clear();
      MessagesUtility__DebugNoTimeStamp(
         MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid channel max\r\n");
      retValue = MagnetometerSensor_AK09973D__Deinit(0xFFFFU);
      error = Error__Get();
      if ((retValue == GD_FALSE)&&
         (error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
      {
         retValue = GD_TRUE;
      }
      else
      {
         retValue = GD_FALSE;
      }
   }
   else
   {
      // Nothing
   }

   // Check each sub-test case to report final test outcome
   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Checking if process completed successfully\r\n");
   // Check data.
   if ((retValue == GD_TRUE) && (testResult == UNIT_TEST_MANAGER__TEST_PASS))
   {
      testResult = UNIT_TEST_MANAGER__TEST_PASS;
   }
   else
   {
      // Nothing
   }

   return testResult;
}


/*
 * See header for doco
 */
uint8_t MagnetometerSensor_AK09973DTest__Process_Test(uint16_t Channel)
{
   uint16_t instance = 0xFFFFU;
   uint8_t breakLoop = GD_FALSE;
   genoTimer_t timer;
   uint8_t timerExpired;
   timerExpired = GD_FALSE;
   genoTimer_t messageTimer;
   genoTimer_t testTimer;
   uint32_t timerCountDown;
   uint8_t testResult;

   // following RawStationaryVariationMargin
   // Magnetic filed change detection threshold, specify a value in
   // milli-Tesla for boundry value indicating a magnetic field has changed
   // between two module samples. Specify a smallest integer value (as opposed that we could use a )
   // floating point variable here)
   // using for the boundry value which is
   // 1 milli-Tesla.
   // This should detect a magnetic field change if user puts a manget close to the sensor
   // as magnetic field varies tiny bit around a stable value when there is no
   // external magenet field change
   // e.g.
   // [0.0780, -0.2441, -0.7204]milli-Tesla
   // [0.0692, -0.2431, -0.7293]milli-Tesla
   // [0.0725, -0.2540, -0.7238]milli-Tesla
   // [0.0791, -0.2508, -0.7402]milli-Tesla
   // [0.0571, -0.2408, -0.7369]milli-Tesla
   // [0.0615, -0.2563, -0.7293]milli-Tesla
   // [0.0648, -0.2486, -0.7391]milli-Tesla
   // [0.0692, -0.2431, -0.7314]milli-Tesla
   // [0.0681, -0.2496, -0.7347]milli-Tesla
   // [0.0791, -0.2441, -0.7271]milli-Tesla
   const float32_t magneticFieldVariationMargin = 1.0f;
   uint8_t retValue;


   MessagesUtility__DebugNoTimeStamp(
      MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing MagnetometerSensor_AK09973DTest__Process_Test\r\n\r\n");

   for (uint16_t index = 0U; ((index < MAGNETOMETER_SENSOR_AK09973D_CONFIG__INSTANCE_MAX) && (breakLoop == GD_FALSE));
        index++)
   {
      if (MagnetometerSensor_AK09973DConfig__Channel_gro[index] == Channel)
      {
         instance = index;
         breakLoop = GD_TRUE;
      }
      else
      {
         // Nothing
      }
   }

   if (instance == 0xFFFFU)
   {
      retValue = GD_FALSE;
   }
   else
   {
      // case: testing if magnetic sensor is functional in producing two distinct
      // sample while putting a magnet right next to the sensor
      magnetometerAk09973D_t magneticFieldReading1;
      magnetometerAk09973D_t magneticFieldReading2;
      magnetometerAk09973D_t magneticFieldSampleChange;
      // Switching mode to single to do first single mode measurement
      magnetometerAk09973DSettings_t configureSettings =
      {
         .samplingFrequency  = (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_SINGLE_MEASUREMENT,
         .samplingSensitivity = (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_HIGH_SENSITIVITY_SETTING,
      };
      retValue = MagnetometerSensor_AK09973D__Configure(
         Channel,
         &configureSettings
         );

      if(retValue == GD_TRUE)
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Sampling and processing at least one sample by GenericSensorController__Process\r\n\r\n");
         WatchDogInterface__Reload();
         TimerUtility__SetTimer(&timer,
            MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);
         while (timerExpired == GD_FALSE)
         {
            timerExpired = TimerUtility__HasTimerExpired(&timer);
            GenericSensorController__Process(Channel);
         }

         retValue = GenericSensorController__PeakLatestData(
            GENERIC_SENSOR_CONTROLLER_CONFIG__MAGNETOMETER_AK09973D_CHANNEL, &magneticFieldReading1,
            sizeof(magnetometerAk09973D_t));
         if (retValue == GD_TRUE)
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "First magnetometer converted readings: [%.4f, %.4f, %.4f]mT ",
               magneticFieldReading1.magneticXAxis,
               magneticFieldReading1.magneticYAxis,
               magneticFieldReading1.magneticZAxis);
            // setup timer to print message every second
            TimerUtility__SetTimer(&messageTimer, TEST_1_SEC_TIME_OUT);
            // setup timer for the test, wait 10 seconds for user to send response
            TimerUtility__SetTimer(&testTimer, TEST_5_SEC_TIME_OUT);
            timerCountDown = 5U;
            timerExpired = GD_FALSE;
            // clear error before the test
            Error__Clear();
            do
            {
               if (timerCountDown != 0U)
               {
                  if (TimerUtility__HasTimerExpired(&messageTimer) == GD_TRUE)
                  {
                     timerCountDown--;
                     MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                        "\r\nWaiting for user to move a magnet closer to the sensor"
                        " to create change in magnet field measurement. "
                        "Continue another sampling in %u seconds\r\n",
                        timerCountDown);
                     TimerUtility__ResetTimer(&messageTimer);
                  }
                  else
                  {
                     // do nothing
                  }
               }
               else
               {
                  // count down to 0, should be timeout
               }
               timerExpired = TimerUtility__HasTimerExpired(&testTimer);

#if ((APP_MODULES_CONFIG__ENABLE_AM_01_19_WATCH_DOG_INTERFACE == GD_TRUE) &&\
         (PIOS_CONFIG__COMPILE_WATCHDOG == GD_TRUE))
               WatchDogInterface__Reload();
#endif
             } while (timerExpired == GD_FALSE);


            // Do second single mode measurement
            timerExpired = GD_FALSE;
            retValue = MagnetometerSensor_AK09973D__Configure(
               Channel,
               &configureSettings
               );
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "Sampling and processing at least one sample by GenericSensorController__Process\r\n\r\n");
            WatchDogInterface__Reload();
            TimerUtility__SetTimer(&timer,
               MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);
            while (timerExpired == GD_FALSE)
            {
               timerExpired = TimerUtility__HasTimerExpired(&timer);
               GenericSensorController__Process(Channel);
            }

            if(retValue == GD_TRUE)
            {
               retValue = GenericSensorController__PeakLatestData(
                  GENERIC_SENSOR_CONTROLLER_CONFIG__MAGNETOMETER_AK09973D_CHANNEL, &magneticFieldReading2,
                  sizeof(magnetometerAk09973D_t));

               if (retValue == GD_TRUE)
               {
                  MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                     "Second magnetometer converted readings: [%.4f, %.4f, %.4f]mT \r\n",
                     magneticFieldReading2.magneticXAxis,
                     magneticFieldReading2.magneticYAxis,
                     magneticFieldReading2.magneticZAxis);
                  magneticFieldSampleChange.magneticXAxis =
                     fabsf(magneticFieldReading1.magneticXAxis - magneticFieldReading2.magneticXAxis);
                  magneticFieldSampleChange.magneticYAxis =
                     fabsf(magneticFieldReading1.magneticYAxis - magneticFieldReading2.magneticYAxis);
                  magneticFieldSampleChange.magneticZAxis =
                     fabsf(magneticFieldReading1.magneticZAxis - magneticFieldReading2.magneticZAxis);

                  // Only one of the magnetic field axis need to pass the magnetic field change value margin
                  // which imply a magnetic field change
                  if (magneticFieldSampleChange.magneticXAxis >
                      magneticFieldVariationMargin)
                  {
                      retValue = GD_TRUE;
                  }
                  else if (magneticFieldSampleChange.magneticYAxis >
                      magneticFieldVariationMargin)
                  {
                      retValue = GD_TRUE;
                  }
                  else if (magneticFieldSampleChange.magneticZAxis >
                      magneticFieldVariationMargin)
                  {
                     retValue = GD_TRUE;
                  }
                  else
                  {
                     MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                        "two consecutive sample "
                        "were too close in value. Please move magnet closer to mangentometer sensor (preferably right "
                        " next to the sensor) and "
                        " try running the test again. \r\n");
                     retValue = GD_FALSE;
                  }
               }
               else
               {
                  MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed to get magnetometerAk09973 Sensor Data!\r\n");
               }
            }
            else
            {
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed to configure magnetometerAk09973 "
                  "Sensor to single measurement mode!\r\n");
            }
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed to get magnetometerAk09973 Sensor Data!\r\n");
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed to configure magnetometerAk09973 "
            "Sensor to single measurement mode!\r\n");
      }
   }

   // Negative case: testing invalid channel
   if (retValue == GD_TRUE)
   {
      genoError_t error;
      Error__Init(&error);
      Error__Clear();
      MessagesUtility__DebugNoTimeStamp(
         MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid channel max\r\n");
      retValue = MagnetometerSensor_AK09973D__Process(GENERICSENSORCONTROLLER__STATE_IDLE, 0xFFFFU);
      error = Error__Get();
      if ((retValue == GD_FALSE)&&
         (error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
      {
         retValue = GD_TRUE;
      }
      else
      {
         retValue = GD_FALSE;
      }
   }
   else
   {
      // Nothing
   }

   // Inform user to remove magnet away from the sensor so that the magnet will not affect
   // next unit test cases.
   // setup timer to print message every second
   TimerUtility__SetTimer(&messageTimer, TEST_1_SEC_TIME_OUT);
   // setup timer for the test, wait 10 seconds for user to send response
   TimerUtility__SetTimer(&testTimer, TEST_3_SEC_TIME_OUT);
   timerCountDown = 3U;
   timerExpired = GD_FALSE;
   // clear error before the test
   Error__Clear();
   do
   {
      if (timerCountDown != 0U)
      {
         if (TimerUtility__HasTimerExpired(&messageTimer) == GD_TRUE)
         {
            timerCountDown--;
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "Waiting for user to move the magnet away from the sensor"
               ", so that it will not affect next unit test case."
               "Wait finish in %u seconds\r\n",
               timerCountDown);
            TimerUtility__ResetTimer(&messageTimer);
         }
         else
         {
            // do nothing
         }
      }
      else
      {
         // count down to 0, should be timeout
      }
      timerExpired = TimerUtility__HasTimerExpired(&testTimer);

#if ((APP_MODULES_CONFIG__ENABLE_AM_01_19_WATCH_DOG_INTERFACE == GD_TRUE) &&\
(PIOS_CONFIG__COMPILE_WATCHDOG == GD_TRUE))
      WatchDogInterface__Reload();
#endif
    } while (timerExpired == GD_FALSE);

   // Check each sub-test case to report final test outcome
   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Checking if process completed successfully\r\n");
   // Check outcome from sub-test above.
   if (retValue == GD_TRUE)
   {
      testResult = UNIT_TEST_MANAGER__TEST_PASS;
   }
   else
   {
      testResult = UNIT_TEST_MANAGER__TEST_FAIL;
   }

   return testResult;
}


/*
 * See header for doco
 */
uint8_t MagnetometerSensor_AK09973DTest__ProcessData_Test(uint16_t Channel)
{
   uint8_t retValue;
   uint8_t testResult;
   uint16_t instance = 0xFFFFU;
   uint8_t breakLoop = GD_FALSE;

   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Testing MagnetometerSensor_AK09973DTest__ProcessData_Test\r\n\r\n");

   for (uint16_t index = 0U;
        ((index < MAGNETOMETER_SENSOR_AK09973D_CONFIG__INSTANCE_MAX) && (breakLoop == GD_FALSE));
        index++)
   {
      if (MagnetometerSensor_AK09973DConfig__Channel_gro[index] == Channel)
      {
         instance = index;
         breakLoop = GD_TRUE;
      }
      else
      {
         // Nothing
      }
   }

   if (instance == 0xFFFFU)
   {
      retValue = GD_FALSE;
   }
   else
   {
      retValue =
         MagnetometerSensor_AK09973D__ProcessData(Channel, &TestRawSample, sizeof(TestRawSample));
      if (retValue == GD_TRUE)
      {
         magnetometerAk09973D_t MagneticFieldReading;

         retValue = GenericSensorController__PeakLatestData(
            Channel, &MagneticFieldReading, sizeof(magnetometerAk09973D_t));
         if (retValue == GD_TRUE)
         {

            // Define a tolerance for comparing floating-point numbers
            // This is required to remove MISRA C warning -> Info 777: Testing floats for equality
            // MISRA C doesn't allow float number equality comparison
            float32_t tolerance = 0.0001f; // Adjust as necessary based on your application requirements
            magnetometerAk09973D_t magneticFieldSampleChange;

            // Convert test raw sample to float32_t variable for comparison needed below.
            // Conversion from int16_t raw sample to float32_t processed expected data is done here
            TestRawConvertedMagReading.magneticXAxis = (float32_t)TestMagneticXAxis*MAGNETOMETER_SENSOR_AK09973D__HIGH_SENSITIVITY_XYZ_INCREMENT;
            TestRawConvertedMagReading.magneticYAxis = (float32_t)TestMagneticYAxis*MAGNETOMETER_SENSOR_AK09973D__HIGH_SENSITIVITY_XYZ_INCREMENT;
            TestRawConvertedMagReading.magneticZAxis = (float32_t)TestMagneticZAxis*MAGNETOMETER_SENSOR_AK09973D__HIGH_SENSITIVITY_XYZ_INCREMENT;
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "Expected Test processed data are: [%f, %f, %f]milli-Tesla\r\n",
               TestRawConvertedMagReading.magneticXAxis,
               TestRawConvertedMagReading.magneticYAxis,
               TestRawConvertedMagReading.magneticZAxis);
            magneticFieldSampleChange.magneticZAxis = fabsf(MagneticFieldReading.magneticZAxis - TestRawConvertedMagReading.magneticZAxis);
            magneticFieldSampleChange.magneticYAxis = fabsf(MagneticFieldReading.magneticYAxis - TestRawConvertedMagReading.magneticYAxis);
            magneticFieldSampleChange.magneticXAxis = fabsf(MagneticFieldReading.magneticXAxis - TestRawConvertedMagReading.magneticXAxis);
            // Effectively doing float number equality comparison to check if
            // processed data are the same as expected data
            if ((magneticFieldSampleChange.magneticXAxis < tolerance) &&
               (magneticFieldSampleChange.magneticYAxis < tolerance) &&
               (magneticFieldSampleChange.magneticZAxis < tolerance))
            {
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                  " Processed magnetic filed data from raw data matches expected \r\n");
            }
            else
            {
               // test failed
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                  "Processed magnetic filed data from raw data does not match expected\r\n");
               retValue = GD_FALSE;
            }
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "GenericSensorController__PeakLatestData failed to return latest processed data\r\n");
            retValue = GD_FALSE;
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "MagnetometerSensor_AK09973D__ProcessData failed to process data\r\n");
         retValue = GD_FALSE;
      }
   }

   // Negative case: testing invalid channel
   if (retValue == GD_TRUE)
   {
      genoError_t error;
      Error__Init(&error);
      Error__Clear();
      MessagesUtility__DebugNoTimeStamp(
         MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid channel max\r\n");
      retValue = MagnetometerSensor_AK09973D__ProcessData(0xFFFFU, &TestRawSample, sizeof(magnetometerAk09973DRawData_t));
      error = Error__Get();
      if ((retValue == GD_FALSE)&&
         (error.cause == GENERICSENSORCONTROLLER__INVALID_CHANNEL_ERROR))
      {
         retValue = GD_TRUE;
      }
      else
      {
         retValue = GD_FALSE;
      }
   }
   else
   {
      // Nothing
   }

   // Negative case: testing invalid parameter
   if (retValue == GD_TRUE)
   {
      genoError_t error;
      Error__Init(&error);
      Error__Clear();
      MessagesUtility__DebugNoTimeStamp(
         MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid data pointer\r\n");
      retValue = MagnetometerSensor_AK09973D__ProcessData(Channel, NULL, sizeof(magnetometerAk09973DRawData_t));
      error = Error__Get();
      if ((retValue == GD_FALSE)&&
         (error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
      {
         retValue = GD_TRUE;
      }
      else
      {
         retValue = GD_FALSE;
      }
   }
   else
   {
      // Nothing
   }

   // Negative case: testing invalid parameter
   if (retValue == GD_TRUE)
   {
      genoError_t error;
      Error__Init(&error);
      Error__Clear();
      MessagesUtility__DebugNoTimeStamp(
         MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid data size\r\n");
      retValue = MagnetometerSensor_AK09973D__ProcessData(Channel, &TestRawSample, 0U);
      error = Error__Get();
      if ((retValue == GD_FALSE)&&
         (error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
      {
         retValue = GD_TRUE;
      }
      else
      {
         retValue = GD_FALSE;
      }
   }
   else
   {
      // Nothing
   }

   // testing on MagnetometerSensor_AK09973D__FormatData API
   if (retValue == GD_TRUE)
   {
      magnetometerAk09973DRawData_t testOuputData;
      size_t testOuputDataSize = sizeof(testOuputData);
      uint32_t testInputDataSize =
         MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_DATA_LENGTH +
         MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_STATUS_BYTE_LEN +
         MAGNETOMETER_SENSOR_AK09973D__READ_MAGNETIC_FIELD_REGISTER_PADDING0_LEN;
      genoError_t error;

      Error__Init(&error);
      Error__Clear();

      retValue = MagnetometerSensor_AK09973D__FormatData(
         TestRawSampleInArray, testInputDataSize, &testOuputData, testOuputDataSize);

      if (retValue == GD_TRUE)
      {
          // Convert test raw sample to float32_t variable for comparison needed below.
          // Conversion from int16_t raw sample to float32_t processed expected data is done here
          if ((testOuputData.magneticXAxisMsb == TestRawSample.magneticXAxisMsb) &&
              (testOuputData.magneticXAxisLsb == TestRawSample.magneticXAxisLsb) &&
              (testOuputData.magneticYAxisMsb == TestRawSample.magneticYAxisMsb) &&
              (testOuputData.magneticYAxisLsb == TestRawSample.magneticYAxisLsb) &&
              (testOuputData.magneticZAxisMsb == TestRawSample.magneticZAxisMsb) &&
              (testOuputData.magneticZAxisLsb == TestRawSample.magneticZAxisLsb))
          {
             MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                "MagnetometerSensor_AK09973D__FormatData formatted data matches expected\r\n");
          }
          else
          {
             // test failed
             MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                "MagnetometerSensor_AK09973D__FormatData formatted data does NOT matches expected\r\n");
             retValue = GD_FALSE;
          }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "failed at MagnetometerSensor_AK09973D__FormatData call\r\n");
         retValue = GD_FALSE;
      }

      if (retValue == GD_TRUE)
      {
         // Negative case: error reporting test
         Error__Clear();
         retValue = MagnetometerSensor_AK09973D__FormatData(
            NULL, testInputDataSize, &testOuputData, testOuputDataSize);
         error = Error__Get();
         if ((retValue == GD_FALSE)&&(error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
         {
             retValue = GD_TRUE;
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "MagnetometerSensor_AK09973D__FormatData test failed\r\n");
            retValue = GD_FALSE;
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Failed at invalid input data pointer test\r\n");
      }

      if (retValue == GD_TRUE)
      {
         // Negative case: error reporting test
         Error__Clear();
         retValue = MagnetometerSensor_AK09973D__FormatData(
            TestRawSampleInArray, testInputDataSize, NULL, testOuputDataSize);
         error = Error__Get();
         if ((retValue == GD_FALSE)&&(error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
         {
             retValue = GD_TRUE;
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "MagnetometerSensor_AK09973D__FormatData test failed\r\n");
            retValue = GD_FALSE;
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Failed at invalid output data test\r\n");
      }

      if (retValue == GD_TRUE)
      {
         // Negative case: error reporting test
         Error__Clear();
         retValue = MagnetometerSensor_AK09973D__FormatData(
            TestRawSampleInArray, 0xFF, &testOuputData, testOuputDataSize);
         error = Error__Get();
         if ((retValue == GD_FALSE)&&(error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
         {
             retValue = GD_TRUE;
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "MagnetometerSensor_AK09973D__FormatData test failed\r\n");
            retValue = GD_FALSE;
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Failed at invalid input data length\r\n");
      }

      if (retValue == GD_TRUE)
      {
         // Negative case: error reporting test
         Error__Clear();
         retValue = MagnetometerSensor_AK09973D__FormatData(
            TestRawSampleInArray, testInputDataSize, &testOuputData, (size_t)0xFF);
         error = Error__Get();
         if ((retValue == GD_FALSE)&&(error.cause == GENERICSENSORCONTROLLER__PARAMETER_ERROR))
         {
             retValue = GD_TRUE;
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "MagnetometerSensor_AK09973D__FormatData test failed\r\n");
            retValue = GD_FALSE;
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Failed at invalid output data length\r\n");
      }
   }
   else
   {
      MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
         "MagnetometerSensor_AK09973D__ProcessData Unit test failed \r\n");
      retValue = GD_FALSE;
   }

   // Check each sub-test case to report final test outcome
   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Checking if process completed successfully\r\n");
   // Check data.
   if (retValue == GD_TRUE)
   {
      testResult = UNIT_TEST_MANAGER__TEST_PASS;
   }
   else
   {
      testResult = UNIT_TEST_MANAGER__TEST_FAIL;
   }
   // Nothing to test

   return testResult;
}


/*
 * See header for doco
 */
uint8_t MagnetometerSensor_AK09973DTest__Configure_Test(uint16_t Channel)
{
   uint16_t instance = 0xFFFFU;
   uint8_t breakLoop = GD_FALSE;
   genoTimer_t timer;
   uint8_t timerExpired;
   timerExpired = GD_FALSE;
   genoTimer_t messageTimer;
   genoTimer_t testTimer;
   uint32_t timerCountDown;
   uint8_t testResult;
   uint8_t retValue;
   magnetometerAk09973DSettings_t configureSettings = {
      .samplingFrequency = (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_SINGLE_MEASUREMENT,
      .samplingSensitivity =
         (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_HIGH_SENSITIVITY_SETTING,
   };
   const float32_t magneticFieldVariationMargin = 1.0f;

   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Testing MagnetometerSensor_AK09973DTest__Configure_Test\r\n\r\n");

   for (uint16_t index = 0U;
        ((index < MAGNETOMETER_SENSOR_AK09973D_CONFIG__INSTANCE_MAX) && (breakLoop == GD_FALSE));
        index++)
   {
      if (MagnetometerSensor_AK09973DConfig__Channel_gro[index] == Channel)
      {
         instance = index;
         breakLoop = GD_TRUE;
      }
      else
      {
         // Nothing
      }
   }

   if (instance == 0xFFFFU)
   {
      retValue = GD_FALSE;
   }
   else
   {
      // case: testing check there is no two consecutive samples arrived during single
      // measurement mode
      magnetometerAk09973DRawData_t sensorRawData1;
      magnetometerAk09973DRawData_t sensorRawData2;
      // Switching mode to single to do first single mode measurement
      retValue = MagnetometerSensor_AK09973D__Configure(Channel, &configureSettings);

      if (retValue == GD_TRUE)
      {
         MessagesUtility__DebugNoTimeStamp(
            MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Sampling and processing at least one sample by "
                                                 "GenericSensorController__Process\r\n\r\n");
         WatchDogInterface__Reload();
         TimerUtility__SetTimer(&timer,
            MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);
         while (timerExpired == GD_FALSE)
         {
            timerExpired = TimerUtility__HasTimerExpired(&timer);
            GenericSensorController__Process(Channel);
         }

         retValue = GenericSensorController__PeakLatestRawData(
            GENERIC_SENSOR_CONTROLLER_CONFIG__MAGNETOMETER_AK09973D_CHANNEL, &sensorRawData1,
            sizeof(magnetometerAk09973DRawData_t));

         if (retValue == GD_TRUE)
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "magnetometerAk09973 Sensor raw Sample, Z MSB:0X%02x, Z LSB:0X%02x ,"
               "Y MSB:0X%02x, Y LSB:0X%02x , X MSB:0X%02x, X LSB:0X%02x\r\n",
               sensorRawData1.magneticZAxisMsb, sensorRawData1.magneticZAxisLsb,
               sensorRawData1.magneticYAxisMsb, sensorRawData1.magneticYAxisLsb,
               sensorRawData1.magneticXAxisMsb, sensorRawData1.magneticXAxisLsb);
            // setup timer to print message every second
            TimerUtility__SetTimer(&messageTimer, TEST_1_SEC_TIME_OUT);
            // setup timer for the test, wait 10 seconds for user to send response
            TimerUtility__SetTimer(&testTimer, TEST_1_SEC_TIME_OUT);
            timerCountDown = 5U;
            timerExpired = GD_FALSE;
            // clear error before the test
            Error__Clear();
            do
            {
               if (timerCountDown != 0U)
               {
                  if (TimerUtility__HasTimerExpired(&messageTimer) == GD_TRUE)
                  {
                     timerCountDown--;
                     MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                        "Waiting 1 second before taking next single measurement sample\r\n",
                        timerCountDown);
                     TimerUtility__ResetTimer(&messageTimer);
                  }
                  else
                  {
                     // do nothing
                  }
               }
               else
               {
                  // count down to 0, should be timeout
               }
               timerExpired = TimerUtility__HasTimerExpired(&testTimer);

#if ((APP_MODULES_CONFIG__ENABLE_AM_01_19_WATCH_DOG_INTERFACE == GD_TRUE) &&                       \
     (PIOS_CONFIG__COMPILE_WATCHDOG == GD_TRUE))
               WatchDogInterface__Reload();
#endif
            } while (timerExpired == GD_FALSE);

            // continuous run another sensor sampling
            MessagesUtility__DebugNoTimeStamp(
               MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Read sensor measurement sample register.\r\n");
            uint8_t measurementRegVal
               [MAGNETOMETER_SENSOR_AK09973D__TOTAL_REG_SIZE_IN_READ_REGISTER_PROCESS] = {0U};
            uint16_t registerReadLen =
               (uint16_t)MAGNETOMETER_SENSOR_AK09973D__TOTAL_REG_SIZE_IN_READ_REGISTER_PROCESS;
            genericSensorRegister_t readMeasurementRegister = {
               .registerAddress = MAGNETOMETER_SENSOR_AK09973D__READ_3_AXES_MEASUREMENT_REGISTER,
               .registerValue = 0x00,
               .setOnInitialise = GD_FALSE,
            };
            retValue = RegisterReadBlockingUnitTest(
               instance, &readMeasurementRegister, measurementRegVal, registerReadLen);

            // i.e. sensor still responds on I2C, same as previous sampled value
            if (retValue == GD_TRUE)
            {
               uint8_t statusVal = measurementRegVal[0U];
               if (statusVal != (uint8_t)MAGNETOMETER_SENSOR_AK09973D__STATUS_BYTE_DEFAULT_VAL)
               {
                  retValue = GD_FALSE;
               }
               else
               {
                  // first byte is correct
                  sensorRawData2.magneticZAxisMsb = measurementRegVal
                     [MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_Z_AXIS_IDX];
                  sensorRawData2.magneticZAxisLsb = measurementRegVal
                     [MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_Z_AXIS_IDX+1U];
                  sensorRawData2.magneticYAxisMsb = measurementRegVal
                     [MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_Y_AXIS_IDX];
                  sensorRawData2.magneticYAxisLsb = measurementRegVal
                     [MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_Y_AXIS_IDX+1U];
                  sensorRawData2.magneticXAxisMsb = measurementRegVal
                     [MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_X_AXIS_IDX];
                  sensorRawData2.magneticXAxisLsb = measurementRegVal
                     [MAGNETOMETER_SENSOR_AK09973D__RAW_MEASUREMENT_X_AXIS_IDX+1U];

                  if ((sensorRawData1.magneticZAxisMsb== sensorRawData2.magneticZAxisMsb)&&
                     (sensorRawData1.magneticZAxisLsb == sensorRawData2.magneticZAxisLsb)&&
                     (sensorRawData1.magneticYAxisMsb == sensorRawData2.magneticYAxisMsb)&&
                     (sensorRawData1.magneticYAxisLsb == sensorRawData2.magneticYAxisLsb)&&
                     (sensorRawData1.magneticXAxisMsb == sensorRawData2.magneticXAxisMsb)&&
                     (sensorRawData1.magneticXAxisLsb == sensorRawData2.magneticXAxisLsb))
                     {
                        retValue = GD_TRUE;
                        MessagesUtility__DebugNoTimeStamp(
                           MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Two consecutive single mode measurement are equal\r\n");
                     }
                     else
                     {
                        retValue = GD_FALSE;
                     }
               }
            }
            else
            {
               retValue = GD_FALSE;
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                  "read magnetometerAk09973 sample failed!\r\n");
            }
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "GenericSensorController__PeakLatestRawData failed\r\n");
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Failed to configure magnetometerAk09973 "
            "Sensor to single measurement mode!\r\n");
      }



      if(retValue == GD_TRUE)
      {
         // case: testing if two consecutive samples arrived during
         // continuous measurement mode are different in value
         magnetometerAk09973D_t magneticFieldReading1;
         magnetometerAk09973D_t magneticFieldReading2;
         magnetometerAk09973D_t magneticFieldSampleChange;
         // Switching mode to continuous measurement to do measurement
         configureSettings.samplingFrequency  = (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_CONTIN_MEASUREMENT_10HZ;
         configureSettings.samplingSensitivity = (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_HIGH_SENSITIVITY_SETTING;

         retValue = MagnetometerSensor_AK09973D__Configure(
            Channel,
            &configureSettings
            );
         if(retValue == GD_TRUE)
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "Sampling and processing at least one sample by GenericSensorController__Process\r\n\r\n");
            WatchDogInterface__Reload();
            TimerUtility__SetTimer(&timer,
               MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);

            while (timerExpired == GD_FALSE)
            {
               timerExpired = TimerUtility__HasTimerExpired(&timer);
               GenericSensorController__Process(Channel);
            }

            retValue = GenericSensorController__PeakLatestData(
               GENERIC_SENSOR_CONTROLLER_CONFIG__MAGNETOMETER_AK09973D_CHANNEL, &magneticFieldReading1,
               sizeof(magnetometerAk09973D_t));
            if (retValue == GD_TRUE)
            {
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                  "First magnetometer converted readings: [%.4f, %.4f, %.4f]mT\r\n",
                  magneticFieldReading1.magneticXAxis,
                  magneticFieldReading1.magneticYAxis,
                  magneticFieldReading1.magneticZAxis);
               // setup timer to print message every second
               TimerUtility__SetTimer(&messageTimer, TEST_1_SEC_TIME_OUT);
               // setup timer for the test, wait 1 seconds between two different samples
               TimerUtility__SetTimer(&testTimer, TEST_1_SEC_TIME_OUT);
               timerCountDown = 1U;
               timerExpired = GD_FALSE;
               // clear error before the test
               Error__Clear();
               do
               {
                  if (timerCountDown != 0U)
                  {
                     if (TimerUtility__HasTimerExpired(&messageTimer) == GD_TRUE)
                     {
                        timerCountDown--;
                        MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                           "\r\nWaiting wait 1 seconds between two different samples"
                           " to allow change in magnet field measurement. "
                           "Continue another sampling in %u seconds\r\n",
                           timerCountDown);
                        TimerUtility__ResetTimer(&messageTimer);
                     }
                     else
                     {
                        // do nothing
                     }
                  }
                  else
                  {
                     // count down to 0, should be timeout
                  }
                  timerExpired = TimerUtility__HasTimerExpired(&testTimer);

   #if ((APP_MODULES_CONFIG__ENABLE_AM_01_19_WATCH_DOG_INTERFACE == GD_TRUE) &&\
            (PIOS_CONFIG__COMPILE_WATCHDOG == GD_TRUE))
                  WatchDogInterface__Reload();
   #endif
                } while (timerExpired == GD_FALSE);

               // Do second single mode measurement
               timerExpired = GD_FALSE;
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                  "Sampling and processing at least one sample by GenericSensorController__Process\r\n");
               WatchDogInterface__Reload();
               TimerUtility__SetTimer(&timer,
                  MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);
               while (timerExpired == GD_FALSE)
               {
                  timerExpired = TimerUtility__HasTimerExpired(&timer);
                  GenericSensorController__Process(Channel);
               }

               retValue = GenericSensorController__PeakLatestData(
                  GENERIC_SENSOR_CONTROLLER_CONFIG__MAGNETOMETER_AK09973D_CHANNEL, &magneticFieldReading2,
                  sizeof(magnetometerAk09973D_t));

               if (retValue == GD_TRUE)
               {
                  MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                     "Second magnetometer converted readings: [%.4f, %.4f, %.4f]mT \r\n",
                     magneticFieldReading2.magneticXAxis,
                     magneticFieldReading2.magneticYAxis,
                     magneticFieldReading2.magneticZAxis);
                  magneticFieldSampleChange.magneticXAxis =
                     fabsf(magneticFieldReading1.magneticXAxis - magneticFieldReading2.magneticXAxis);
                  magneticFieldSampleChange.magneticYAxis =
                     fabsf(magneticFieldReading1.magneticYAxis - magneticFieldReading2.magneticYAxis);
                  magneticFieldSampleChange.magneticZAxis =
                     fabsf(magneticFieldReading1.magneticZAxis - magneticFieldReading2.magneticZAxis);

                  // All of the magnetic field axis need to be less than the magnetic field stable margin
                  // which imply there is no magnetic field change
                  if (magneticFieldSampleChange.magneticXAxis >
                      magneticFieldVariationMargin)
                  {
                      retValue = GD_FALSE;
                  }
                  else if (magneticFieldSampleChange.magneticYAxis >
                      magneticFieldVariationMargin)
                  {
                      retValue = GD_FALSE;
                  }
                  else if (magneticFieldSampleChange.magneticZAxis >
                      magneticFieldVariationMargin)
                  {
                     retValue = GD_FALSE;
                  }
                  else
                  {
                     MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                        "Two consecutive sample "
                        "were less than margin for stable magnetic field.\r\n");
                     retValue = GD_TRUE;
                  }
               }
               else
               {
                  MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed to get second magnetometerAk09973 Sensor Data!\r\n");
               }
            }
            else
            {
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed to get first magnetometerAk09973 Sensor Data!\r\n");
            }
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed to configure magnetometerAk09973 "
               "Sensor to continuous measurement mode!\r\n");
         }
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed in testing check there is no two "
               "consecutive samples arrived during single measurement mode!\r\n");
      }


      // Negative case: testing invalid channel
      if (retValue == GD_TRUE)
      {
         genoError_t error;
         Error__Init(&error);
         Error__Clear();
         MessagesUtility__DebugNoTimeStamp(
            MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid channel max\r\n");
         retValue = MagnetometerSensor_AK09973D__Configure(0xFFFFU, &configureSettings);
         error = Error__Get();
         if ((retValue == GD_FALSE) &&
             (error.cause == GENERICSENSORCONTROLLER__INVALID_CHANNEL_ERROR))
         {
            retValue = GD_TRUE;
         }
         else
         {
            retValue = GD_FALSE;
         }
      }
      else
      {
         // Nothing
      }

      // Negative case: testing invalid configured sampling frequency
      if (retValue == GD_TRUE)
      {
         genoError_t error;
         Error__Init(&error);
         Error__Clear();
         MessagesUtility__DebugNoTimeStamp(
            MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Testing invalid configured sampling frequency\r\n");
         configureSettings.samplingFrequency =
            MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_CONTIN_MEASUREMENT_2000HZ;
         configureSettings.samplingSensitivity =
            (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_HIGH_SENSITIVITY_SETTING;
         retValue = MagnetometerSensor_AK09973D__Configure(Channel, &configureSettings);
         error = Error__Get();
         if ((retValue == GD_FALSE) && (error.cause == GENERICSENSORCONTROLLER__INVALID_INPUT))
         {
            retValue = GD_TRUE;
         }
         else
         {
            retValue = GD_FALSE;
         }
      }
      else
      {
         // Nothing
      }

      // Negative case: testing invalid configured sampling sensitivity settings
      if (retValue == GD_TRUE)
      {
         genoError_t error;
         Error__Init(&error);
         Error__Clear();
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Testing invalid configured sampling sensitivity settings\r\n");
         configureSettings.samplingFrequency =
            MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_CONTIN_MEASUREMENT_2000HZ;
         configureSettings.samplingSensitivity =
            (uint8_t)(MAGNETOMETER_SENSOR_AK09973D__CTR2_HIGH_SENSITIVITY_SETTING + 1U);
         retValue = MagnetometerSensor_AK09973D__Configure(Channel, &configureSettings);
         error = Error__Get();
         if ((retValue == GD_FALSE) && (error.cause == GENERICSENSORCONTROLLER__INVALID_INPUT))
         {
            retValue = GD_TRUE;
         }
         else
         {
            retValue = GD_FALSE;
         }
      }
      else
      {
         // Nothing
      }


      if (retValue == GD_TRUE)
      {
         timerExpired = GD_FALSE;
         // Switching mode back to single measurement
         configureSettings.samplingFrequency =
            (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_SINGLE_MEASUREMENT;
         configureSettings.samplingSensitivity =
            (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_HIGH_SENSITIVITY_SETTING;
         retValue = MagnetometerSensor_AK09973D__Configure(Channel, &configureSettings);
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Execute full cycle of GenericSensorProcess state machine to restore sensor "
            "configuration after this test case\r\n");
         WatchDogInterface__Reload();
         TimerUtility__SetTimer(&timer,
            MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);
         while (timerExpired == GD_FALSE)
         {
            timerExpired = TimerUtility__HasTimerExpired(&timer);
            GenericSensorController__Process(Channel);
         }
      }
      else
      {
         //Nothing
      }

   }

   // Check each sub-test case to report final test outcome
   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Checking if process completed successfully\r\n");
   // Check data.
   if (retValue == GD_TRUE)
   {
      testResult = UNIT_TEST_MANAGER__TEST_PASS;
   }
   else
   {
      testResult = UNIT_TEST_MANAGER__TEST_FAIL;
   }

   return testResult;
}

/*
 * See header for doco
 */
uint8_t MagnetometerSensor_AK09973DTest__ReturnSettings_Test(uint16_t Channel)
{
   uint16_t instance = 0xFFFFU;
   uint8_t breakLoop = GD_FALSE;
   uint8_t testResult;
   uint8_t retValue;
   genoTimer_t timer;
   uint8_t timerExpired = GD_FALSE;

   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Testing MagnetometerSensor_AK09973DTest__ReturnSettings_Test\r\n\r\n");

   magnetometerAk09973DSettings_t testConfigureSetting = {
      .samplingFrequency = (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_MODE_CONTIN_MEASUREMENT_100HZ,
      .samplingSensitivity =
         (uint8_t)MAGNETOMETER_SENSOR_AK09973D__CTR2_HIGH_SENSITIVITY_SETTING,
   };

   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Testing MagnetometerSensor_AK09973DTest__Configure_Test\r\n\r\n");

   for (uint16_t index = 0U;
        ((index < MAGNETOMETER_SENSOR_AK09973D_CONFIG__INSTANCE_MAX) && (breakLoop == GD_FALSE));
        index++)
   {
      if (MagnetometerSensor_AK09973DConfig__Channel_gro[index] == Channel)
      {
         instance = index;
         breakLoop = GD_TRUE;
      }
      else
      {
         // Nothing
      }
   }

   if (instance == 0xFFFFU)
   {
      retValue = GD_FALSE;
   }
   else
   {
      retValue = MagnetometerSensor_AK09973D__Configure(Channel, &testConfigureSetting);
      if(retValue == GD_FALSE)
      {
         MessagesUtility__DebugNoTimeStamp(
            MESSAGES_UTILITY__IMPORTANT_MESSAGE, "Failed configure sensor test configuraiton setting");
      }
      else
      {
         MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
            "Exexcute GenericSensorProcess to write test configuration to sensor\r\n");
         WatchDogInterface__Reload();
         TimerUtility__SetTimer(&timer,
            MAGNETOMETER_SENSOR_AK09973D_CONFIG__GENERIC_SENSOR_PROCESS_STATE_MACHINE_EXECUTION_TIME);
         while (timerExpired == GD_FALSE)
         {
            timerExpired = TimerUtility__HasTimerExpired(&timer);
            GenericSensorController__Process(Channel);
         }

         magnetometerAk09973DSettings_t resturnSettings =
         {
            .samplingFrequency  = 0U,
            .samplingSensitivity = 0U,
         };

         retValue = MagnetometerSensor_AK09973D__ReturnSettings(
            Channel,
            &resturnSettings);
         // read mode configuration register success
         MessagesUtility__DebugNoTimeStamp(
            MESSAGES_UTILITY__IMPORTANT_MESSAGE, "currentSensorSampling Freneuency:%02x, "
            "currentSensorSampling sensitivity %02x\r\n",
            resturnSettings.samplingFrequency,
            resturnSettings.samplingSensitivity
            );
         if (retValue == GD_TRUE)
         {
            if ((resturnSettings.samplingFrequency == testConfigureSetting.samplingFrequency) &&
                (resturnSettings.samplingSensitivity == testConfigureSetting.samplingSensitivity))
            {
               MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
                  "Module return settings API returned expected current sensor configuration\r\n");
            }
            else
            {
               retValue = GD_FALSE;
            }
         }
         else
         {
            MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
               "Module return settings API returned error \r\n");
         }
      }
   }

   // Check each sub-test case to report final test outcome
   MessagesUtility__DebugNoTimeStamp(MESSAGES_UTILITY__IMPORTANT_MESSAGE,
      "Checking if process completed successfully\r\n");
   // Check data.
   if (retValue == GD_TRUE)
   {
      testResult = UNIT_TEST_MANAGER__TEST_PASS;
   }
   else
   {
      testResult = UNIT_TEST_MANAGER__TEST_FAIL;
   }

   return testResult;
}

#endif //(GENERIC_SENSOR_CONTROLLER_CONFIG__COMPILE_TESTS == GD_TRUE)
#endif //(APP_MODULES_CONFIG__ENABLE_PM_03_30_MAGNETOMETER_SENSOR_AK09973D == GD_TRUE)
#endif //(APP_MODULES_CONFIG__ENABLE_PM_02_00_GENERIC_SENSOR_CONTROLLER == GD_TRUE)

