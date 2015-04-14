#include "motor_pwm.h"

//   Set the parameter of the MOTOR module  ( you can check this in the schematics file given ) 
static MOTOR_PWM_STRUCT motor_pwm = {
	{TIM_Channel_1, GPIO_Pin_6, GPIO_Pin_2, ENABLE, TIM_OC1Init},
	{TIM_Channel_2, GPIO_Pin_7, GPIO_Pin_3, ENABLE, TIM_OC2Init},
	{TIM_Channel_3, GPIO_Pin_8, GPIO_Pin_4, ENABLE, TIM_OC3Init},
	{TIM_Channel_4, GPIO_Pin_9, GPIO_Pin_5, ENABLE, TIM_OC4Init}
};

/**
  * @brief  Inintialization of timer and GPIO for motors
  * @param  None
  * @retval None
  */
void motor_init(void) {
	
	GPIO_InitTypeDef  MOTOR_MAG_GPIO_InitStructure;   // this part defines the variable we use 
	GPIO_InitTypeDef  MOTOR_DIR_GPIO_InitStructure;         
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      // TimeBase is for timer setting   > refer to P. 344 of library
	TIM_OCInitTypeDef  TIM_OCInitStructure;             // OC is for channel setting within a timer  > refer to P. 342 of library
	u8 motor_id;

	RCC_APB1PeriphClockCmd(MOTOR_TIM_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(MOTOR_DIR_GPIO_RCC | MOTOR_MAG_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);

	MOTOR_MAG_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	MOTOR_MAG_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	MOTOR_DIR_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	MOTOR_DIR_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);            // this part feeds the parameter we set above

	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // counter will count up (from 0 to FFFF)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;       //timer clock = dead-time and sampling clock 	
	TIM_TimeBaseStructure.TIM_Prescaler = 2;                         //clk=72M/(2+1)=24MHz, interval=41.67ns
	TIM_TimeBaseStructure.TIM_Period = 1000;	                        //pulse cycle=1/24M*1000=41.67us
	TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);       // this part feeds the parameter we set above

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         //set "high" to be effective output
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	             //produce output when counter < CCR
	TIM_OCInitStructure.TIM_Pulse = 0;                               // this part sets the initial CCR value
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    // this part enable the output

	TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);
	TIM_Cmd(MOTOR_TIM, ENABLE);	

	for (motor_id = 0; motor_id < 6; motor_id++) {                                    
		if (motor_pwm[motor_id].state == DISABLE)
			continue;
		MOTOR_MAG_GPIO_InitStructure.GPIO_Pin |= motor_pwm[motor_id].motor_mag_pin;
		MOTOR_DIR_GPIO_InitStructure.GPIO_Pin |= motor_pwm[motor_id].motor_dir_pin;
		motor_pwm[motor_id].oc_init_function(MOTOR_TIM, &TIM_OCInitStructure);      // feeds the OCInit
	}										
	GPIO_Init(MOTOR_MAG_PORT, &MOTOR_MAG_GPIO_InitStructure);	
	GPIO_Init(MOTOR_DIR_PORT, &MOTOR_DIR_GPIO_InitStructure);	
}

/**
  * @brief  Controlling the PWM for motors
  * @param  motor_id: Port of Motor to be used (MOTOR1, MOTOR2, MOTOR3, MOTOR4)
  * @param  ccr_val: CCR Value from -1000 to 1000
  * @retval None
  */
void motor_control(u8 motor_id , s16 ccr_val) {	            // input the motor_id you change and the corresponding value
												                // 1 = 0.1%,  1000 = 100%     determine
    if(ccr_val < -1000 || ccr_val > 1000)                          // this part limit your input for ccr_val 
		ccr_val = 0;	
	
	if (ccr_val > 0)                                            // the +ve or -ve represent 2 direction of rotation
         GPIO_SetBits(MOTOR_DIR_PORT, motor_pwm[motor_id].motor_dir_pin );
	else {
		GPIO_ResetBits(MOTOR_DIR_PORT, motor_pwm[motor_id].motor_dir_pin );
		ccr_val = -ccr_val;
	}

    if (motor_id == MOTOR1 )                                      // this part feed your ccr value into the register
		TIM_SetCompare1(MOTOR_TIM, ccr_val );      				  // Compare1 is for channel 1 
	else if (motor_id == MOTOR2 )
		TIM_SetCompare2(MOTOR_TIM, ccr_val );					  // Compare2 is for channel 2 
	else if ( motor_id == MOTOR3 )
		TIM_SetCompare3(MOTOR_TIM, ccr_val );					  // Compare3 is for channel 3 
	else if ( motor_id == MOTOR4 )
		TIM_SetCompare4(MOTOR_TIM, ccr_val );				 	 // Compare4 is for channel 4 
}
