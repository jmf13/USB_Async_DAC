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
typedef enum
{
  PLAYER_STARTED = 1,
  PLAYER_STOPPING,
  PLAYER_STOPPED
}PLAYER_STATE_TypeDef;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
//## #define FB_RATE_DELTA (1<<12)
//## #define FB_RATE_DELTA 64


/* Private function prototypes -----------------------------------------------*/
static int8_t Audio_Init(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t Audio_DeInit(uint32_t options);
static int8_t Audio_PlaybackCmd(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t Audio_VolumeCtl(uint8_t vol);
static int8_t Audio_MuteCtl(uint8_t cmd);
static int8_t Audio_PeriodicTC(uint8_t cmd);
static int8_t Audio_GetState(void);

void fill_buffer (int buffer, uint8_t *pbuf, uint32_t size);

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

// State of the player
PLAYER_STATE_TypeDef player_state;

/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
int16_t Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE * 2];  // This represents two buffers in ping pong arrangement stereo samples
int16_t Audio_buffer_L[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Left channel
int16_t Audio_buffer_R[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Right channel

extern USBD_HandleTypeDef USBD_Device;
//?? Be careful: the USBDbuffer is bytes and not int16_t
// int16_t Audio_input_ring_buffer[AUDIO_INPUT_BUFF_SIZE]; // We want to work between 1/3 and 2/3 buffer, pulling AUDIO_OUTPUT_BUFF_SIZE at a time


//?? No need anymore of Audio_input_ring_buffer_length, replaced by buffer in USBD-Audio 
// static volatile int Audio_input_ring_buffer_length; // ## check if static relevant or not
//static volatile int Audio_input_ring_buffer_start;
//static volatile int Audio_input_ring_buffer_end;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
//?? check if volume usefull
// static uint8_t Volume = 70;

// variables to keep as global variables the information about the buffer 
// pulled from the usbd_audio

uint8_t* pbuf_input;
uint32_t buf_input_size;

// Size of the output buffer
uint32_t Audio_output_buffer_size =0;

//variable for next output buffer to write - 2 means no buffer to fill (wait state)
int volatile next_buff = 2; 

// Endless loop that fills the buffers when needed
// this procedure is called by main() once initialization is finished
void Audio_Loop (void)
{
	
	while(1){

	  switch(next_buff) {
  
  	    case 0:
	      fill_buffer (0, pbuf_input, buf_input_size);
	      break;

	    case 1:
	      fill_buffer (1, pbuf_input, buf_input_size);
	      break;
    
	    case 2:
	      // do nothing
	      break;
  	  }
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

  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, AUDIO_DEFAULT_VOLUME, USBD_AUDIO_FREQ);

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

  switch(cmd)
  {
  //Called by usbd_audio when the input buffer is ready, to start the music playing
  case AUDIO_CMD_START:
	    // init filters
	  	initFilter();

		// fills first half of the output buffer with zeros, playing will start with half buffer of zeros
		for(i=0; i<AUDIO_OUTPUT_BUF_SIZE; i++){
			 Audio_output_buffer[i]= 0;
		}
		

	  	// fill second half of the audio_output_buffer, first half will start with 0000s
	  	fill_buffer (1, pbuf, size);

	  	// Size in bytes, for a complete buffer
	  	BSP_AUDIO_OUT_Play((uint16_t *)&Audio_output_buffer, AUDIO_OUTPUT_BUF_SIZE*2);
		// we then wait for a DMA event of DMA half transferred 
	  	next_buff = 2;
    break;


  // called by USBD_Audio in the pull_data callback => we save the infor srelated to the buffer ready: pointer and size 
  case AUDIO_CMD_PLAY:
    pbuf_input = pbuf;
    buf_input_size = size;
    break;

  }
  return 0;
}

/**
  * @brief  Controls AUDIO Volume.             
  * @param  vol: Volume level (0..100)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */

//?? Who sets the volume?
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

 switch(player_state)
  {
    case PLAYER_STARTED:
	//start playing the prepared buffer
	//?? make sure that Audio_output_buffer_size is well calculated
	BSP_AUDIO_OUT_ChangeBuffer((uint16_t *) &Audio_output_buffer,  AUDIO_OUTPUT_BUF_SIZE*2);
	//Pull_Data from USBD_Audio - note that data are provided by USBD_Audio using the Audio_PlaybackCmd callback
	USBD_AUDIO_DataPull (&USBD_Device);	
	next_buff = 1;  
      break;
    //?? See how to fill buffer with zeors to complement the incomplete buffer
    case PLAYER_STOPPING:
        //Start playing the prepared buffer before being stopped
	BSP_AUDIO_OUT_ChangeBuffer((uint16_t *) &Audio_output_buffer, Audio_output_buffer_size);
	
	//we don't pull data, set next_buffer to no filling, and stop the player
	next_buff = 2;

	//?? See how to fill buffer with zeors to complement the incomplete buffer
	player_state = PLAYER_STOPPED;
      break;

    case PLAYER_STOPPED:
      // We do nothing
      break;
   }
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
switch(player_state)
  {
    case PLAYER_STARTED:
	//Pull_Data from USBD_Audio - note that data are provided by USBD_Audio using the Audio_PlaybackCmd callback
	USBD_AUDIO_DataPull (&USBD_Device);
	next_buff = 0;	  
      break;

    case PLAYER_STOPPING:
      // We do nothing
      break;

    case PLAYER_STOPPED:
      // We do nothing
      break;
   }
	
}

//?? Adapt to use pbuf and size parameters
void fill_buffer (int buffer, uint8_t *pbuf, uint32_t size) // buffer=0 for first half of the buffer buffer = 1 for second half
{
	uint16_t i = 0;

	// If there in no underrun: ie the ring buffer has more than the needed samples to fill the next AUDIO_OUTPUT_BUFFER
	// then we prepare the next buffer

	//?? issue with Audio_input_ring_buffer_length which is not interrupt free +> attempt without, relying on the
	// Asynch feature
	//if (Audio_input_ring_buffer_length > AUDIO_OUTPUT_BUFF_SIZE){
		for(i=0; i<AUDIO_OUTPUT_BUF_SIZE/2; i++){
			Audio_buffer_L[i]= pbuf[i];
			Audio_buffer_R[i]= pbuf[i+1];
		}

		//if user button pressed, we apply the DSP
		//## add the init of the pushbutton
		if (BSP_PB_GetState(BUTTON_KEY)== 1) {
				BSP_LED_On(LED3);
				dsp((int16_t*)&Audio_buffer_L[0], AUDIO_OUTPUT_BUF_SIZE/2, 0);
				dsp((int16_t*)&Audio_buffer_R[0], AUDIO_OUTPUT_BUF_SIZE/2, 1);
		} else {
			BSP_LED_Off(LED3);
		}

		// Build stereo Audio_output_buffer from Audio_buffer_L and Audio_buffer_R, filling the requested
		// ping pong buffer: first half offset 0 or second half offset AUDIO_OUTPUT_BUFF_SIZE
		for(i=0; i<AUDIO_OUTPUT_BUF_SIZE/2; i++){
			 Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE*buffer+2*i]= Audio_buffer_L[i]; /*Left Channel*/
			 Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE*buffer+2*i + 1]= Audio_buffer_R[i]; /*Right Channel*/
		}

		Audio_output_buffer_size = Audio_output_buffer_size + size;
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




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
