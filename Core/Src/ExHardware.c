#include "ExHardware.h"
#include "main.h"

/**
 * usd in backlight function
 **/
extern TIM_HandleTypeDef htim3;

uint8_t backlight = 0xFF;
//---------------------------------


void Backlight_Control ( BACKLIGHT_CONTROL BL_Ctrl )
{
	switch( BL_Ctrl )
	{
		//==============================================
		case BL_ON:
			
//			Set_Backlight_Duty( backlight );
			HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_1 );
			
			break;
		//==============================================
		case BL_OFF:
		default:
			
			HAL_TIM_PWM_Stop( &htim3, TIM_CHANNEL_1 );
			Set_Backlight_Duty( 0 );
			
			break;
		//==============================================
	}
}

void Set_Backlight_Duty ( uint8_t d )
{
	#define MaxTrueDuty 100
	#define MinTrueDuty 0

	TIM_OC_InitTypeDef sConfigOC = {0};

	d = ( d > 100 ) ? 100 : d;
	
	if( d == 100 )
	  goto set_duty;
	
	d = ((MaxTrueDuty - MinTrueDuty) * d ) / 100 + MinTrueDuty;
	
	set_duty:

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;

	sConfigOC.Pulse = d;

	HAL_TIM_PWM_Stop( &htim3, TIM_CHANNEL_1 );
	HAL_TIM_PWM_ConfigChannel( &htim3, &sConfigOC, TIM_CHANNEL_1 );

	// Enable PWM
	HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_1 );
}

uint8_t Get_Backlight_Duty ( void )
{
	return backlight;
}
