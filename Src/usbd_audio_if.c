/**
  ******************************************************************************
  * @file    USB_Device/AUDIO_Standalone/Src/usbd_audio_if.c
  * @author  MCD Application Team
  * @version jmf
  * @date    09-08-2016
  * @brief   USB Device Audio interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "usbd_audio_if.h"
#include "stm32f4_discovery_audio.h"
#include "dsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
// #define FB_RATE_DELTA (1<<12)
#define FB_RATE_DELTA 64

// F0B_rate calculation with 2 levels
#define SPK1_GAP_U2    AUDIO_INPUT_BUFF_SIZE * 6 / 8	// A half buffer up in distance	=> Speed up host a lot
#define	SPK1_GAP_U1    AUDIO_INPUT_BUFF_SIZE * 5 / 8	// A quarter buffer up in distance => Speed up host a bit
#define SPK1_GAP_NOM   AUDIO_INPUT_BUFF_SIZE * 4 / 8	// Ideal distance is half the size of linear buffer
#define SPK1_GAP_L1	   AUDIO_INPUT_BUFF_SIZE * 3 / 8  // A quarter buffer down in distance => Slow down host a bit
#define SPK1_GAP_L2	   AUDIO_INPUT_BUFF_SIZE * 2 / 8  // A half buffer down in distance => Slow down host a lot

/* Private function prototypes -----------------------------------------------*/
static int8_t Audio_Init(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t Audio_DeInit(uint32_t options);
static int8_t Audio_PlaybackCmd(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t Audio_VolumeCtl(uint8_t vol);
static int8_t Audio_MuteCtl(uint8_t cmd);
static int8_t Audio_PeriodicTC(uint8_t cmd);
static int8_t Audio_GetState(void);
void fill_buffer (int buffer);
void feedback_calculation(void);

/* Private variables ---------------------------------------------------------*/


USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops = {
  Audio_Init,
  Audio_DeInit,
  Audio_PlaybackCmd,
  Audio_VolumeCtl,
  Audio_MuteCtl,
  Audio_PeriodicTC,
  Audio_GetState,
};

/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
int16_t Audio_output_buffer[AUDIO_OUTPUT_BUFF_SIZE * 2];  // This represents two buffers in ping pong arrangement stereo samples
int16_t Audio_buffer_L[AUDIO_OUTPUT_BUFF_SIZE/2]; //one ping pong buffer, Left channel
int16_t Audio_buffer_R[AUDIO_OUTPUT_BUFF_SIZE/2]; //one ping pong buffer, Right channel
int16_t Audio_input_ring_buffer[AUDIO_INPUT_BUFF_SIZE]; // We want to work between 1/3 and 2/3 buffer, pulling AUDIO_OUTPUT_BUFF_SIZE at a time

//volatile static uint16_t old_gap = AUDIO_INPUT_BUFF_SIZE;
extern uint32_t feedback_data;

// static volatile int Audio_input_ring_buffer_length; // ## check if static relevant or not
static volatile int Audio_input_ring_buffer_start;
static volatile int Audio_input_ring_buffer_end;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
// static uint8_t Volume = 70;

int volatile next_buff = 1; //next output buffer to write

int Underrun=0;    // counter to track underruns
int Overrun=0;    // counter to track overruns

void Audio_Preparation (void)
{
	BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, AUDIO_DEFAULT_VOLUME, USBD_AUDIO_FREQ);
	
	// init filters
	initFilter();

	//while buffer filled below AUDIO_INPUT_BUFF_SIZE*2/3
	while(Audio_input_ring_buffer_end<AUDIO_INPUT_BUFF_SIZE/2);

	// fill second half of the audio_output_buffer, firts half will start with 0000s
	fill_buffer (1);

	// Size in bytes, for a complete buffer
	//BSP_AUDIO_OUT_Play((uint16_t *)&Audio_output_buffer, AUDIO_OUTPUT_BUFF_SIZE*2);
	Audio_PlaybackCmd((uint8_t *)&Audio_output_buffer, AUDIO_OUTPUT_BUFF_SIZE*4, AUDIO_CMD_START);
	next_buff = 1;

	while(1){
	    BSP_LED_On(LED6);
		while (next_buff == 1);
	  BSP_LED_Off(LED6);
	  fill_buffer (0);


	  // Wait as long as next_buff == 0, then fill second half
	  BSP_LED_On(LED6);
	  while (next_buff == 0);
	  BSP_LED_Off(LED6);
	  fill_buffer (1);

	}
}

/**
  * @brief  Initializes the AUDIO media low layer.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use 
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_Init(uint32_t  AudioFreq, uint32_t Volume, uint32_t options)
{
  // BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, AudioFreq);
  return 0;
}

/**
  * @brief  De-Initializes the AUDIO media low layer.      
  * @param  options: Reserved for future use
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_DeInit(uint32_t options)
{
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
  return 0;
}

/**
  * @brief  Handles AUDIO command.        
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_PlaybackCmd(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  
  int i =0;
  uint16_t * pbuf_uint16 = (uint16_t *)pbuf;

  switch(cmd)
  {
  //## to monitor if the function can be looping to wait for data, and at the same time manage the CMD_DATA_IN on interrupts
  case AUDIO_CMD_START:
	  BSP_AUDIO_OUT_Play((uint16_t *)pbuf, size);
    break;

  //## check if this one is really usefull
  case AUDIO_CMD_PLAY:
    BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)pbuf, size);
    break;

  //USB frame received
  case AUDIO_CMD_DATA_OUT:
    // Copy buffer to input circular buffer
    // If the buffer can be filled without overrun then duplicate tha data in in the ring buffer
    // size is in bytes but buffer is uint16, so the loop is on size/2

	  //?? issue with Audio_input_ring_buffer_length which is not interrupt free +> attempt without, relying on the
	  // Asynch feature
	  //if (Audio_input_ring_buffer_length + size/2 <=  AUDIO_INPUT_BUFF_SIZE){
      for (i = 0; i < size/2; ++i) {
        Audio_input_ring_buffer[Audio_input_ring_buffer_end] = (uint16_t) *pbuf_uint16++;
        Audio_input_ring_buffer_end = (Audio_input_ring_buffer_end + 1 >= AUDIO_INPUT_BUFF_SIZE) ? 0 : Audio_input_ring_buffer_end + 1;
      }
      //## pb of race condition between main() and interrupts
      //Audio_input_ring_buffer_length = Audio_input_ring_buffer_length + size/2;
    //} else {
    // there an over run, we discard the frame, don't fill the ring buffer to help catch up, and increase the overrun counter
	//Overrun++;
    //}
    break;

  }
  return 0;
}

/**
  * @brief  Controls AUDIO Volume.             
  * @param  vol: Volume level (0..100)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_VolumeCtl(uint8_t vol)
{
  BSP_AUDIO_OUT_SetVolume(vol);
  return 0;
}

/**
  * @brief  Controls AUDIO Mute.              
  * @param  cmd: Command opcode
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_MuteCtl(uint8_t cmd)
{
  BSP_AUDIO_OUT_SetMute(cmd);
  return 0;
}

/**
  * @brief  Audio_PeriodicTC              
  * @param  cmd: Command opcode
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_PeriodicTC(uint8_t cmd)
{
  return 0;
}

/**
  * @brief  Gets AUDIO State.              
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_GetState(void)
{
  return 0;
}

/**
  * @brief  Manages the DMA full Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
	//## Verif the syntax for the buffer : *2 to have a complete buffer
	Audio_PlaybackCmd((uint8_t *)&Audio_output_buffer, AUDIO_OUTPUT_BUFF_SIZE*2, AUDIO_CMD_PLAY);
	next_buff = 1;
	feedback_calculation();
	// USBD_AUDIO_Sync(&USBD_Device, AUDIO_OFFSET_FULL);
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
	next_buff = 0;
	feedback_calculation();
	// USBD_AUDIO_Sync(&USBD_Device, AUDIO_OFFSET_HALF);
}


void fill_buffer (int buffer) // buffer=0 for first half of the buffer buffer = 1 for second half
{
	uint16_t i = 0;

	// If there in no underrun: ie the ring buffer has more than the needed samples to fill the next AUDIO_OUTPUT_BUFFER
	// then we prepare the next buffer

	//?? issue with Audio_input_ring_buffer_length which is not interrupt free +> attempt without, relying on the
	// Asynch feature
	//if (Audio_input_ring_buffer_length > AUDIO_OUTPUT_BUFF_SIZE){
		for(i=0; i<AUDIO_OUTPUT_BUFF_SIZE/2; i++){
			Audio_buffer_L[i]= Audio_input_ring_buffer[Audio_input_ring_buffer_start];
			Audio_buffer_R[i]= Audio_input_ring_buffer[Audio_input_ring_buffer_start+1];

			// move the read pointer of Audio_Input_ring_buffer
			Audio_input_ring_buffer_start = (Audio_input_ring_buffer_start + 2 >= AUDIO_INPUT_BUFF_SIZE) ? 0 : Audio_input_ring_buffer_start + 2;
			// Audio_input_ring_buffer_length = Audio_input_ring_buffer_length - 2;
		}

		//if user button pressed, we apply the DSP
		//## add the init of the pushbutton
		if (BSP_PB_GetState(BUTTON_KEY)== 1) {
				BSP_LED_On(LED3);
				dsp((int16_t*)&Audio_buffer_L[0], AUDIO_OUTPUT_BUFF_SIZE/2, 0);
				dsp((int16_t*)&Audio_buffer_R[0], AUDIO_OUTPUT_BUFF_SIZE/2, 1);
		} else {
			BSP_LED_Off(LED3);
		}

		// Build stereo Audio_output_buffer from Audio_buffer_L and Audio_buffer_R, filling the requested
		// ping pong buffer: first half offset 0 or second half offset AUDIO_OUTPUT_BUFF_SIZE
		for(i=0; i<AUDIO_OUTPUT_BUFF_SIZE/2; i++){
			 Audio_output_buffer[AUDIO_OUTPUT_BUFF_SIZE*buffer+2*i]= Audio_buffer_L[i]; /*Left Channel*/
			 Audio_output_buffer[AUDIO_OUTPUT_BUFF_SIZE*buffer+2*i + 1]= Audio_buffer_R[i]; /*Right Channel*/
		}
	//}
		// If there in an underrun, we don't read the buffer to recover
	//else {
		// We toggle LED3 to signal the error
		// We increment the underrun counter
		// The previous data will be used and create a glitch
		//BSP_LED_Toggle(LED3);
		//Underrun = Underrun + 1;
	//}

}

void feedback_calculation(void)
{
	volatile uint16_t gap = 0;
	gap= Audio_input_ring_buffer_end - Audio_input_ring_buffer_start;
	if ((int16_t)gap < 0) {gap += AUDIO_INPUT_BUFF_SIZE;}

	//feedbacktable[0]= 0x66;
		        //feedbacktable[1]= 0x06;
		        //feedbacktable[2]= 0x0C;
		        //feedbacktable[0]= 0x00;
		        //feedbacktable[1]= 0x00;
		        //feedbacktable[2]= 0x0C;
		        //feedbacktable[0]= 0x9A;
		        //feedbacktable[1]= 0xF9;
		        //feedbacktable[2]= 0x0B;

	if 		(gap < SPK1_GAP_L2) { 		// gap < inner lower bound => 1*FB_RATE_DELTA
				BSP_LED_On(LED3);
				BSP_LED_Off(LED4);
				BSP_LED_Off(LED5);
				//BSP_LED_Off(LED6);
				}
			else if (gap < SPK1_GAP_L1) { 			// gap < outer lower bound => 2*FB_RATE_DELTA
				feedback_data =0x0C0666;
				//BSP_LED_Off(LED3);
				BSP_LED_On(LED4);
				BSP_LED_Off(LED5);
				//BSP_LED_Off(LED6);
			}

			else if (gap > SPK1_GAP_U2) { 		// gap < inner lower bound => 1*FB_RATE_DELTA
				//BSP_LED_Off(LED3);
				BSP_LED_Off(LED4);
				BSP_LED_Off(LED5);
				BSP_LED_On(LED6);
			}
			else if (gap > SPK1_GAP_U1) { 		// gap < inner lower bound => 1*FB_RATE_DELTA
				feedback_data = 0x0BF99A;
				//feedback_data = 0x0BE000;
				//BSP_LED_Off(LED3);
				BSP_LED_Off(LED4);
				BSP_LED_On(LED5);
				//BSP_LED_Off(LED6);
			}

			else {		// Go back to indicating feedback system on module LEDs
				//BSP_LED_Off(LED3);
				BSP_LED_Off(LED4);
				BSP_LED_Off(LED5);
				//BSP_LED_Off(LED6);
			}

	/*if (gap < old_gap) {
		if (gap < SPK1_GAP_L2) { 			// gap < outer lower bound => 2*FB_RATE_DELTA
			feedback_data += 2*FB_RATE_DELTA;
			old_gap = gap;
			BSP_LED_On(LED6);
		}
		else if (gap < SPK1_GAP_L1) { 		// gap < inner lower bound => 1*FB_RATE_DELTA
			feedback_data += FB_RATE_DELTA;
			old_gap = gap;
			BSP_LED_On(LED4);
		}
		else {		// Go back to indicating feedback system on module LEDs
			BSP_LED_Off(LED3);
			BSP_LED_Off(LED4);
			BSP_LED_Off(LED5);
			BSP_LED_Off(LED6);
		}
	}
	else if (gap > old_gap) {
		if (gap > SPK1_GAP_U2) { 			// gap > outer upper bound => 2*FB_RATE_DELTA
			feedback_data -= 2*FB_RATE_DELTA;
			old_gap = gap;
			BSP_LED_On(LED3);
		}
		else if (gap > SPK1_GAP_U1) { 		// gap > inner upper bound => 1*FB_RATE_DELTA
			feedback_data -= FB_RATE_DELTA;
			old_gap = gap;
			BSP_LED_On(LED5);
		}
		else {		// Go back to indicating feedback system on module LEDs
			BSP_LED_Off(LED3);
			BSP_LED_Off(LED4);
			BSP_LED_Off(LED5);
			BSP_LED_Off(LED6);
		}
	}
	else {		// Go back to indicating feedback system on module LEDs
		BSP_LED_Off(LED3);
		BSP_LED_Off(LED4);
		BSP_LED_Off(LED5);
		BSP_LED_Off(LED6);
	}*/
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
