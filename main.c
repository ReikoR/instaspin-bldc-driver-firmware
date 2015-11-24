/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/proj_lab05b.c
//! \brief Adjusting the speed current controller
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB05b PROJ_LAB05b
//@{

//! \defgroup PROJ_LAB05b_OVERVIEW Project Overview
//!
//! Adjusting the supplied speed controller
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"
#include <string.h>
#include "drivers/sci/sci.h"
#include "modules/usDelay/usDelay.h"


#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#pragma CODE_SECTION(SCI_RX_ISR,"ramfuncs");
#endif

// Include header files used in the main function


// **************************************************************************
// the defines


#define LED_BLINK_FREQ_Hz   5

#define EE_READ  0x80   // 10 xxxxxx(A6-A0)
#define EE_WRITE 0x40   // 01 xxxxxx(A6-A0)
#define EE_EWEN  0x3F   // 00 11XXXX(X is DONT CARE)
#define EE_EWDS  0x00   // 00 00XXXX(X is DONT CARE)
#define EE_ERASE 0xC0   // 11 xxxxxx(A6-A0)


// **************************************************************************
// the globals

uint16_t boardId = '5';

volatile uint16_t writeData = 0b0101010101010101;
volatile uint16_t eepromReadData = 0;
volatile uint16_t writeEEPROM = 0;
volatile uint16_t timeout = 0;

volatile uint16_t voltageTooLow = 1;
_iq lowVoltageThreshold = _IQ(0.005);

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

MATH_vec3 gAdcBiasI;
MATH_vec3 gAdcBiasV;
CTRL_Handle ctrlHandle;

CLARKE_Handle   clarkeHandle_I;               //!< the handle for the current Clarke transform
CLARKE_Obj      clarke_I;                     //!< the current Clarke transform object

CLARKE_Handle   clarkeHandle_V;               //!< the handle for the voltage Clarke transform
CLARKE_Obj      clarke_V;                     //!< the voltage Clarke transform object

EST_Handle      estHandle;                    //!< the handle for the estimator

IPARK_Handle    iparkHandle;                  //!< the handle for the inverse Park transform
IPARK_Obj       ipark;                        //!< the inverse Park transform object

PARK_Handle     parkHandle;                   //!< the handle for the Park object
PARK_Obj        park;                         //!< the Park transform object

SVGEN_Handle    svgenHandle;                  //!< the handle for the space vector generator
SVGEN_Obj       svgen;                        //!< the space vector generator object

HAL_Handle halHandle;

SCI_Handle sciHandle;

USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
CTRL_Obj ctrl;				//v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

char buf[16];
char returnBuf[32];
int counter = 0;
int rxIntCounter = 0;
int commandReceived = 0;
int commandStart = 0;

int isWaitingTxFifoEmpty = 0;
int txOffDelayCount = 2; // 1 count = 66.667us, 15 counts = 1ms
int txOffDelayCounter = 0;
int txOffDelayActive = 0;
int setTxOff = 0;
int sendSpeed = 0;

//uint32_t slowCount = 30000; // 2s
//uint32_t slowCounter = 0;

void scia_init(void);
void serialWrite(char *sendData, int length);

void eepromWriteEnable();

void eepromWriteDisable();

void eepromErase(char addr);

unsigned short eepromRead(char addr);

void eepromWrite(char addr, unsigned short data);

void eepromSend(char data);

// **************************************************************************
// the functions

void main(void)
{
  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));


  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);


  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
  controller_obj = (CTRL_Obj *)ctrlHandle;
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif


  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);


  // initialize the Clarke modules
  clarkeHandle_I = CLARKE_init(&clarke_I,sizeof(clarke_I));
  clarkeHandle_V = CLARKE_init(&clarke_V,sizeof(clarke_V));


  // set the number of current sensors
  setupClarke_I(clarkeHandle_I,gUserParams.numCurrentSensors);


  // set the number of voltage sensors
  setupClarke_V(clarkeHandle_V,gUserParams.numVoltageSensors);


#ifdef FAST_ROM_V1p6
  estHandle = controller_obj->estHandle;
#else
  estHandle = ctrl.estHandle;
#endif


  // initialize the inverse Park module
  iparkHandle = IPARK_init(&ipark,sizeof(ipark));


  // initialize the Park module
  parkHandle = PARK_init(&park,sizeof(park));


  // initialize the space vector generator module
  svgenHandle = SVGEN_init(&svgen,sizeof(svgen));


  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);


  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);

  scia_init();


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif


  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();



  gMotorVars.Kp_spd = _IQ(4.0);
  gMotorVars.MaxAccel_krpmps = _IQ(10.0);
  gMotorVars.SpeedRef_krpm = _IQ(0.0);
  gMotorVars.Flag_enableSys = 1;

  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys));

    // Enable the Library internal PI.  Iq is referenced by the speed PI now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, true);

    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
    {

    	/*if (isWaitingTxFifoEmpty && SCI_getRxFifoStatus(sciHandle) == SCI_FifoStatus_Empty) {
    	//if (isWaitingTxFifoEmpty && SCI_txReady(sciHandle)) {
    		isWaitingTxFifoEmpty = 0;
    		txOffDelayActive = 1;
    	}*/

    	/*if (setTxOff) {
    		setTxOff = 0;
    		//GPIO_setLow(halHandle->gpioHandle,GPIO_Number_12);
    		AIO_setLow(halHandle->gpioHandle,AIO_Number_6);
    	}*/

    	if (commandReceived) {
    	//if (counter == 9) {
    		commandReceived = 0;

    		//SerialCommand cmd;

    		//memcpy(&cmd, buf + commandStart, 6);

    		//if (cmd.id == boardId && cmd.type == 's') {
    		//if (buf[1] == boardId && buf[2] == 's') {
    			/*long value = ((long)buf[3]) | ((long)buf[4] << 8) | ((long)buf[5] << 16) | ((long)buf[6] << 24);

				gMotorVars.SpeedRef_krpm = value;
				gMotorVars.Flag_Run_Identify = 1;*/

				/*returnBuf[0] = '<';
				returnBuf[1] = boardId;
				returnBuf[2] = 'd';

				long returnValue = gMotorVars.Speed_krpm;

				returnBuf[3] = returnValue;
				returnBuf[4] = returnValue >> 8;
				returnBuf[5] = returnValue >> 16;
				returnBuf[6] = returnValue >> 24;
				returnBuf[7] = '>';

				serialWrite(returnBuf, 8);*/

				//_IQtoa(returnBuf + 2, "%3.5f", gMotorVars.Speed_krpm);
				//int n = strlen(returnBuf);
				//returnBuf[n] = '\n';
				//serialWrite(returnBuf, n + 1);

				//long * intlocation = (long*)(&returnBuf[2]);
				//*intlocation = gMotorVars.SpeedRef_krpm;
				//*intlocation = 287392129l;
			//}

    		if (buf[0] == boardId && buf[2] == 's') {
    			buf[counter - 1] = '\0';
				gMotorVars.SpeedRef_krpm = _atoIQ(buf + 3);
				gMotorVars.Flag_Run_Identify = 1;

				/*returnBuf[0] = boardId;
				returnBuf[1] = 's';
				_IQtoa(returnBuf + 2, "%3.5f", gMotorVars.Speed_krpm);
				int n = strlen(returnBuf);
				returnBuf[n] = '\n';
				serialWrite(returnBuf, n + 1);*/
			}

    		commandStart = 0;
    		counter = 0;
    	}

    	/*if (sendSpeed) {
    		sendSpeed = 0;

			if (buf[1] == boardId && buf[2] == 's') {
				returnBuf[0] = '<';
				returnBuf[1] = boardId;
				returnBuf[2] = 'd';

				long returnValue = gMotorVars.Speed_krpm;

				returnBuf[3] = returnValue;
				returnBuf[4] = returnValue >> 8;
				returnBuf[5] = returnValue >> 16;
				returnBuf[6] = returnValue >> 24;
				returnBuf[7] = '>';

				serialWrite(returnBuf, 8);
			}
		}*/

    	/*if (SCI_txReady(sciHandle)) {
			SCI_write(sciHandle, 'a');

		}*/

		/*while (SCI_getRxFifoStatus(sciHandle)) {
			char rev_data = SCI_read(sciHandle);

			if (rev_data == '\n') {
				buf[counter] = '\0';
				counter = 0;
				//CTRL_setSpd_ref_krpm(ctrlHandle, _atoIQ(buf));
				if (buf[0] == boardId && buf[1] == 's') {
					gMotorVars.SpeedRef_krpm = _atoIQ(buf + 2);
					gMotorVars.Flag_Run_Identify = 1;

					returnBuf[0] = boardId;
					returnBuf[1] = 's';
					_IQtoa(returnBuf + 2, "%3.5f", gMotorVars.Speed_krpm);
					int n = strlen(returnBuf);
					returnBuf[n] = '\n';
					serialWrite(returnBuf, n + 1);
				}
			} else {
				buf[counter] = rev_data;
				counter++;

				if (counter == 16) {
					counter = 0;
				}
			}

			if (SCI_txReady(sciHandle)) {
				usDelay(5000);
				GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_12);
				usDelay(5000);
				SCI_write(sciHandle, rev_data);
				//SCI_putDataNonBlocking(sciHandle, rev_data);
				usDelay(5000);
				GPIO_setLow(halHandle->gpioHandle,GPIO_Number_12);
				usDelay(5000);
			}
		}*/

    	//GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_12);

		/*if (SCI_txReady(sciHandle)) {
			SCI_write(sciHandle, '3');
		}*/

      CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

      // increment counters
      gCounter_updateGlobals++;

      // enable/disable the use of motor parameters being loaded from user.h
      CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);

      // enable/disable Rs recalibration during motor startup
      EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

      // enable/disable automatic calculation of bias values
      CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


      if(CTRL_isError(ctrlHandle)) {
        // set the enable controller flag to false
        CTRL_setFlag_enableCtrl(ctrlHandle,false);

        // set the enable system flag to false
        gMotorVars.Flag_enableSys = false;

        // disable the PWM
        HAL_disablePwm(halHandle);
      } else if (voltageTooLow) {
    	  // set the enable controller flag to false
		  CTRL_setFlag_enableCtrl(ctrlHandle,false);

		  // disable the PWM
		  HAL_disablePwm(halHandle);

		  gMotorVars.Flag_Run_Identify = false;
      } else {
        // update the controller state
        bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

        // enable or disable the control
        CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

        if(flag_ctrlStateChanged)
        {
          CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

          if(ctrlState == CTRL_State_OffLine)
          {
            // enable the PWM
            HAL_enablePwm(halHandle);
          }
          else if(ctrlState == CTRL_State_OnLine)
          {
            if(gMotorVars.Flag_enableOffsetcalc == true)
            {
              uint_least16_t cnt;

              // update the ADC bias values
              HAL_updateAdcBias(halHandle);

              // record the current bias
              for(cnt=0;cnt<3;cnt++)
                gAdcBiasI.value[cnt] = HAL_getBias(halHandle,HAL_SensorType_Current,cnt);

              // record the voltage bias
              for(cnt=0;cnt<3;cnt++)
                gAdcBiasV.value[cnt] = HAL_getBias(halHandle,HAL_SensorType_Voltage,cnt);

              gMotorVars.Flag_enableOffsetcalc = false;
            }
            else
            {
              uint_least16_t cnt;

              // set the current bias
              for(cnt=0;cnt<3;cnt++)
                HAL_setBias(halHandle,HAL_SensorType_Current,cnt,gAdcBiasI.value[cnt]);

              // set the voltage bias
              for(cnt=0;cnt<3;cnt++)
                HAL_setBias(halHandle,HAL_SensorType_Voltage,cnt,gAdcBiasV.value[cnt]);
            }

            // enable the PWM
            HAL_enablePwm(halHandle);
          }
          else if(ctrlState == CTRL_State_Idle)
          {
            // disable the PWM
            HAL_disablePwm(halHandle);
            gMotorVars.Flag_Run_Identify = false;
          }

          if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
            (ctrlState > CTRL_State_Idle) &&
            (gMotorVars.CtrlVersion.minor == 6))
          {
            // call this function to fix 1p6
            USER_softwareUpdate1p6(ctrlHandle);
          }
        }
      }


      if(EST_isMotorIdentified(obj->estHandle))
      {
        // set the current ramp
        EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
        gMotorVars.Flag_MotorIdentified = true;

        // set the speed reference
        CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

        // set the speed acceleration
        CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));
        if(Flag_Latch_softwareUpdate)
        {
          Flag_Latch_softwareUpdate = false;

          USER_calcPIgains(ctrlHandle);

          // initialize the watch window kp and ki current values with pre-calculated values
          gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle,CTRL_Type_PID_Id);
          gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle,CTRL_Type_PID_Id);

          // initialize the watch window kp and ki values with pre-calculated values
          //gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle,CTRL_Type_PID_spd);
          gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle,CTRL_Type_PID_spd);
        }

      }
      else
      {
        Flag_Latch_softwareUpdate = true;

        // the estimator sets the maximum current slope during identification
        gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
      }


      // when appropriate, update the global variables
      if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
      {
        // reset the counter
        gCounter_updateGlobals = 0;

        updateGlobalVariables_motor(ctrlHandle);

        if (voltageTooLow && gMotorVars.VdcBus_kV > lowVoltageThreshold) {
        	voltageTooLow = 0;

        	// Power restored, reset to start with fresh parameters
        	// disable the PWM
			HAL_disablePwm(halHandle);

			// set the default controller parameters (Reset the control to re-identify the motor)
			CTRL_setParams(ctrlHandle,&gUserParams);
			gMotorVars.Flag_Run_Identify = false;
        } else if (!voltageTooLow && gMotorVars.VdcBus_kV < lowVoltageThreshold) {
        	voltageTooLow = 1;

        	// Power lost, disable control
			if (gMotorVars.Flag_Run_Identify) {
				// disable the PWM
				HAL_disablePwm(halHandle);

				CTRL_setFlag_enableCtrl(ctrlHandle,false);

				// set the default controller parameters (Reset the control to re-identify the motor)
				CTRL_setParams(ctrlHandle,&gUserParams);
				gMotorVars.Flag_Run_Identify = false;
			}
        }
      }


      // update Kp and Ki gains
      updateKpKiGains(ctrlHandle);

      // enable/disable the forced angle
      EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

      // enable or disable power warp
      CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
      //GPIO_setLow(halHandle->gpioHandle,GPIO_Number_19);
      HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

      HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
      //GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_19);

      //usDelay(5000);

      //if (writeEEPROM) {
    	  /*writeEEPROM = 0;

    	  GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_34);

			SPI_resetRxFifo(halHandle->drv8301Handle->spiHandle);
			SPI_enableRxFifo(halHandle->drv8301Handle->spiHandle);
			SPI_write(halHandle->drv8301Handle->spiHandle,0b1001100000000000); //Enable write

			usDelay(100);

			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34);

			usDelay(200);

			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_34);

			SPI_resetRxFifo(halHandle->drv8301Handle->spiHandle);
			SPI_enableRxFifo(halHandle->drv8301Handle->spiHandle);

			usDelay(100);

			SPI_write(halHandle->drv8301Handle->spiHandle,0xA000 | writeData >> 11);
			SPI_write(halHandle->drv8301Handle->spiHandle, writeData << 5);

			usDelay(300);

			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34);

			usDelay(20);

			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_34);

			while (!GPIO_getData(halHandle->gpioHandle, GPIO_Number_17)) {
				usDelay(10);
			}

			usDelay(10);

			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34);

			usDelay(20);

			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_34);

			timeout = 0;

			//const uint16_t data = 0;
			volatile uint16_t WaitTimeOut = 0;
			volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;

			// reset the Rx fifo pointer to zero
			SPI_resetRxFifo(halHandle->drv8301Handle->spiHandle);
			SPI_enableRxFifo(halHandle->drv8301Handle->spiHandle);

			// write the command
			SPI_write(halHandle->drv8301Handle->spiHandle,0xC000);
			SPI_write(halHandle->drv8301Handle->spiHandle,0x0000);

			WaitTimeOut = 0;

			// wait for two words to populate the RX fifo, or a wait timeout will occur
			while((RxFifoCnt < SPI_FifoStatus_2_Words) && (WaitTimeOut < 0xffff)) {
			  RxFifoCnt = SPI_getRxFifoStatus(halHandle->drv8301Handle->spiHandle);

			  if (++WaitTimeOut > 0xfffe) {
				  //halHandle->drv8301Handle->RxTimeOut = true;
				  timeout = 1;
			 }
			}

			eepromReadData = 0;

			usDelay(1);
			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34);

			//eepromReadData = (0b0000000000011111 & SPI_readEmu(halHandle->drv8301Handle->spiHandle)) << 11;
			//eepromReadData = eepromReadData | (0b1111111111100000 & SPI_readEmu(halHandle->drv8301Handle->spiHandle)) >> 5;
			eepromReadData = SPI_readEmu(halHandle->drv8301Handle->spiHandle);
			eepromReadData = SPI_readEmu(halHandle->drv8301Handle->spiHandle);*/

    	  /*writeEEPROM = 0;

    	  eepromWriteEnable();
    	  usDelay(100);
    	  eepromWrite(0x00, writeData);
    	  usDelay(100);
    	  eepromReadData = eepromRead(0x00);*/
      //}


#endif

    } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;

  } // end of for(;;) loop

} // end of main() function

void eepromWriteEnable() {
	eepromSend(EE_EWEN);
	usDelay(1);
	GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34); //_eecs=0;
}

void eepromWriteDisable() {
	eepromSend(EE_EWDS);
    usDelay(1);
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34); //_eecs=0;
}

void eepromErase(char addr) {
    eepromSend(EE_ERASE | addr);
    usDelay(1);
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34); //_eecs=0;
    /** wait busy flag clear */
    usDelay(1);     // tcs > 250ns @2.7V
    GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_34); //_eecs=1;
    usDelay(1);     // tsv < 250ns @2.7V
    //while(_eedo==0); // 0.1ms < twp < 10ms
    while (!GPIO_read(halHandle->gpioHandle,GPIO_Number_17));
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34); //_eecs=0;
}

unsigned short eepromRead(char addr) {
    unsigned short data = 0;
    signed char i = 15;
    eepromSend(EE_READ | addr);
    usDelay(1);

    for (i = 15; i >= 0; i--) {
    	GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18); //_eeck=1;
    	usDelay(1);
        GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18); //_eeck=0;
        //data = data | (_eedo<<i);
        data = data | (GPIO_read(halHandle->gpioHandle,GPIO_Number_17) << i);
        usDelay(1);
    }

    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34); //_eecs=0;
    return data;
}

void eepromWrite(char addr, unsigned short data) {
    signed char i = 15;
    eepromSend(EE_WRITE | addr);

    usDelay(1);

    for (i = 15; i >= 0; i--){
        //_eedi = (int)( (data>>i)&0x0001 );
        if ((data >> i) & 0x0001) {
        	GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_16);
        } else {
        	GPIO_setLow(halHandle->gpioHandle,GPIO_Number_16);
        }

        usDelay(1);
		GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18); //_eeck=1;
		usDelay(1);
		GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18); //_eeck=0;
    }

    usDelay(50);

    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34); //_eecs=0;
    /** wait busy flag clear */
    usDelay(1);     // tcs > 250ns @2.7V
    GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_34); //_eecs=1;
    usDelay(1);     // tsv < 250ns @2.7V
    //while(_eedo==0); // 0.1ms < twp < 10ms
    while (!GPIO_read(halHandle->gpioHandle,GPIO_Number_17));
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_34); //_eecs=0;
}

void eepromSend(char data) {
	signed char i = 7;
	GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_16); //_eedi=1;
	GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_34); //_eecs=1;     // fall is in function
	usDelay(1);
	GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18); //_eeck=1;
	usDelay(1);
	GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18); //_eeck=0;
	while(i>=0){
		//_eedi = (data>>i)&0x01;
		if ((data >> i) & 0x0001) {
			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_16);
		} else {
			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_16);
		}
		i--;
		usDelay(1);
		GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18); //_eeck=1;
		usDelay(1);
		GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18); //_eeck=0;
	}
}

interrupt void mainISR(void)
{
  _iq angle_pu = 0;
  _iq speed_pu;
  _iq speed_ref_pu = TRAJ_getIntValue(((CTRL_Obj *)ctrlHandle)->trajHandle_spd);
  _iq speed_outMax_pu = TRAJ_getIntValue(((CTRL_Obj *)ctrlHandle)->trajHandle_spdMax);

  MATH_vec2 Iab_pu;
  MATH_vec2 Vab_pu;
  MATH_vec2 Vdq_out_pu;
  MATH_vec2 Vab_out_pu;
  MATH_vec2 phasor;

  // toggle status LED
  if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }

  /*if (slowCounter++ > slowCount) {
	  slowCounter = 0;
	  AIO_toggle(halHandle->gpioHandle,AIO_Number_4);
	  AIO_toggle(halHandle->gpioHandle,AIO_Number_6);
  }*/

  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);

  {
    uint_least16_t count_isr = CTRL_getCount_isr(ctrlHandle);
    uint_least16_t numIsrTicksPerCtrlTick = CTRL_getNumIsrTicksPerCtrlTick(ctrlHandle);

    // if needed, run the controller
    if(count_isr >= numIsrTicksPerCtrlTick)
    {
      CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

      bool flag_enableSpeedCtrl;
      bool flag_enableCurrentCtrl;

      MATH_vec2 Idq_offset_pu;
      MATH_vec2 Vdq_offset_pu;


      // reset the isr count
      CTRL_resetCounter_isr(ctrlHandle);

      // run Clarke transform on current
      CLARKE_run(clarkeHandle_I,&gAdcData.I,&Iab_pu);

      // run Clarke transform on voltage
      CLARKE_run(clarkeHandle_V,&gAdcData.V,&Vab_pu);

      {
        // run the estimator
        EST_run(estHandle, \
                &Iab_pu, \
                &Vab_pu, \
                gAdcData.dcBus, \
                speed_ref_pu);

        flag_enableSpeedCtrl = EST_doSpeedCtrl(estHandle);
        flag_enableCurrentCtrl = EST_doCurrentCtrl(estHandle);
      }

      // generate the motor electrical angle
      angle_pu = EST_getAngle_pu(estHandle);
      speed_pu = EST_getFm_pu(estHandle);

      // compute the sin/cos phasor
      CTRL_computePhasor(angle_pu,&phasor);

      // set the phasor in the Park transform
      PARK_setPhasor(parkHandle,&phasor);

      // run the Park transform
      PARK_run(parkHandle,&Iab_pu,CTRL_getIdq_in_addr(ctrlHandle));

      // set the offset based on the Id trajectory
      Idq_offset_pu.value[0] = TRAJ_getIntValue(((CTRL_Obj *)ctrlHandle)->trajHandle_Id);
      Idq_offset_pu.value[1] = _IQ(0.0);

      Vdq_offset_pu.value[0] = 0;
      Vdq_offset_pu.value[1] = 0;

      CTRL_setup_user(ctrlHandle,
                      angle_pu,
                      speed_ref_pu,
                      speed_pu,
                      speed_outMax_pu,
                      &Idq_offset_pu,
                      &Vdq_offset_pu,
                      flag_enableSpeedCtrl,
                      flag_enableCurrentCtrl);

      // run the appropriate controller
      if(ctrlState == CTRL_State_OnLine)
      {
        // run the online controller
        CTRL_runPiOnly(ctrlHandle); //,&gAdcData,&gPwmData);

        // get the controller output
        CTRL_getVdq_out_pu(ctrlHandle,&Vdq_out_pu);

        // set the phasor in the inverse Park transform
        IPARK_setPhasor(iparkHandle,&phasor);

        // run the inverse Park module
        IPARK_run(iparkHandle,&Vdq_out_pu,&Vab_out_pu);

        // run the space Vector Generator (SVGEN) module
        SVGEN_run(svgenHandle,&Vab_out_pu,&(gPwmData.Tabc));
      }
      else if(ctrlState == CTRL_State_OffLine)
      {
        // run the offline controller
        CTRL_runOffLine(ctrlHandle,halHandle,&gAdcData,&gPwmData);
      }
    }
    else
    {
      // increment the isr count
      CTRL_incrCounter_isr(ctrlHandle);
    }
  }

  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);

  /*if (txOffDelayActive) {
	  if (++txOffDelayCounter == txOffDelayCount) {
		  txOffDelayCounter = 0;
		  txOffDelayActive = 0;
		  //setTxOff = 1;
		  AIO_setLow(halHandle->gpioHandle,AIO_Number_6);
	  }
  }*/

  /*if (txOffDelayActive) {
  	  txOffDelayActive = 0;
  	  AIO_setLow(halHandle->gpioHandle,AIO_Number_6);
  }*/

  /*if (txOffDelayActive) {
    			  txOffDelayActive = 0;
    			  AIO_setLow(halHandle->gpioHandle,AIO_Number_6);
    		}*/

  if (txOffDelayActive) {
  	  if (++txOffDelayCounter == txOffDelayCount) {
  		  txOffDelayCounter = 0;
  		  txOffDelayActive = 0;
  		  //setTxOff = 1;
  		  AIO_setLow(halHandle->gpioHandle,AIO_Number_6);
  	  }
    }

  if (isWaitingTxFifoEmpty && SCI_getRxFifoStatus(sciHandle) == SCI_FifoStatus_Empty) {
	//if (isWaitingTxFifoEmpty && SCI_txReady(sciHandle)) {
		isWaitingTxFifoEmpty = 0;
		txOffDelayActive = 1;
	}

  /*if (writeEEPROM) {
	  writeEEPROM = 0;

	  eepromWriteEnable();
	  usDelay(100);
	  eepromWrite(0x00, writeData);
	  usDelay(100);
	  eepromReadData = eepromRead(0x00);
  }*/

  return;
} // end of mainISR() function


void setupClarke_I(CLARKE_Handle handle,const uint_least8_t numCurrentSensors)
{
  _iq alpha_sf,beta_sf;


  // initialize the Clarke transform module for current
  if(numCurrentSensors == 3)
    {
      alpha_sf = _IQ(MATH_ONE_OVER_THREE);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
  else if(numCurrentSensors == 2)
    {
      alpha_sf = _IQ(1.0);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
  else
    {
      alpha_sf = _IQ(0.0);
      beta_sf = _IQ(0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(handle,alpha_sf,beta_sf);
  CLARKE_setNumSensors(handle,numCurrentSensors);

  return;
} // end of setupClarke_I() function


void setupClarke_V(CLARKE_Handle handle,const uint_least8_t numVoltageSensors)
{
  _iq alpha_sf,beta_sf;


  // initialize the Clarke transform module for voltage
  if(numVoltageSensors == 3)
    {
      alpha_sf = _IQ(MATH_ONE_OVER_THREE);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
 else
    {
      alpha_sf = _IQ(0.0);
      beta_sf = _IQ(0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(handle,alpha_sf,beta_sf);
  CLARKE_setNumSensors(handle,numVoltageSensors);

  return;
} // end of setupClarke_V() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
	}

  return;
} // end of updateKpKiGains() function

interrupt void SCI_RX_ISR(void) {
	rxIntCounter++;

	while (SCI_rxDataReady(sciHandle) == 1) {
		char c = SCI_read(sciHandle);

		//buf[counter] = c;
		//counter++;

		if (counter < 15) {
			//commandReceived = 1;

			switch (counter) {
			case 0:
				if (c == boardId) {
					buf[counter] = c;
					counter++;
				} else {
					counter = 0;
				}
				break;
			case 1:
				if (c == ':') {
					buf[counter] = c;
					counter++;
					sendSpeed = 1;
				} else {
					counter = 0;
				}
				break;
			case 2:
				if (c == 's') {
					buf[counter] = c;
					counter++;
					sendSpeed = 1;
				} else {
					counter = 0;
				}
				break;
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
			case 11:
			case 12:
			case 13:
			case 14:
			case 15:
				buf[counter] = c;
				counter++;

				if (c == '\n') {
					commandReceived = 1;
				}
				break;
			default:
				counter = 0;
			}
		} else {
			counter = 0;
		}
	}

	SCI_clearRxFifoOvf(sciHandle);
	SCI_clearRxFifoInt(sciHandle);

	PIE_clearInt(halHandle->pieHandle, PIE_GroupNumber_9);
}

void scia_init() {
	sciHandle = SCI_init((void *) SCIA_BASE_ADDR, sizeof(SCI_Obj));

	SCI_setNumStopBits(sciHandle, SCI_NumStopBits_One);
	SCI_disableParity(sciHandle);
	SCI_disableLoopBack(sciHandle);
	SCI_setCharLength(sciHandle, SCI_CharLength_8_Bits);
	SCI_setMode(sciHandle, SCI_Mode_IdleLine);
	//SCI_setPriority(sciHandle, SCI_Priority_FreeRun);
	//SCI_setTxDelay(sciHandle, 255);

	SCI_disableRxErrorInt(sciHandle);
	SCI_disable(sciHandle);
	SCI_disableTxWake(sciHandle);
	SCI_disableSleep(sciHandle);
	SCI_enableRx(sciHandle);
	SCI_enableRxFifo(sciHandle);
	SCI_enableTx(sciHandle);
	SCI_enableTxFifo(sciHandle);
	//SCI_enableTxFifoEnh(sciHandle);

	SCI_setBaudRate(sciHandle, 390);

	SCI_clearRxFifoOvf(sciHandle);
	SCI_clearRxFifoInt(sciHandle);

	SCI_enableRxInt(sciHandle);
	//SCI_enableTxInt(sciHandle);

	/*ENABLE_PROTECTED_REGISTER_WRITE_MODE;
	halHandle->pieHandle->SCIRXINTA = &SCI_RX_ISR;
	DISABLE_PROTECTED_REGISTER_WRITE_MODE;*/

	SCI_enable(sciHandle);

    // enable SCI interrupt
    PIE_enableInt(halHandle->pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_SCIARX);

    // enable CPU interrupt
    CPU_enableInt(halHandle->cpuHandle, CPU_IntNumber_9);
}

void serialWrite(char *sendData, int length) {
	int i = 0;

	//GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_12);
	AIO_setHigh(halHandle->gpioHandle,AIO_Number_6);

	while (i < length) {
		if (SCI_txReady(sciHandle)) {
			SCI_write(sciHandle, sendData[i]);
			i++;
		}
	}

	//GPIO_setLow(halHandle->gpioHandle,GPIO_Number_12);
	isWaitingTxFifoEmpty = 1;
}

//@} //defgroup
// end of file



