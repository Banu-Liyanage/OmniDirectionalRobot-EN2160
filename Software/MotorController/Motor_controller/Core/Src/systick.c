/*
 * systick.c
 *
 *  Created on: May 21, 2025
 *      Author: PANKAJA
 */


#include "systick.h"
#include "encoders.h"

//void CheckEncoderCounts(void){
// Geet the difference and add that to the previous values
//		if (getForwardLeftEncoderCounts() > 31000 || getForwardRightEncoderCounts() > 31000
//				|| getForwardLeftEncoderCounts() > 31000 || getForwardRightEncoderCounts() > 31000
//				|| getForwardLeftEncoderCounts() < -31000 || getForwardRightEncoderCounts() < -31000
//				|| getForwardLeftEncoderCounts() < -31000 || getForwardRightEncoderCounts() < -31000) {
//			int16_t difference = getRightEncoderCounts() - getLeftEncoderCounts();
//			resetEncodersinSystick();
//			TIM1->CNT = (int16_t) difference;
//		}
//}


void SysTickFunction(void) {
	/*
	 * Anything in this function body will be executed every millisecond.
	 * Call you PID update function here.
	 */
	//--------------------------------------------------------------------
		//update_Encoder_Data();

	//--------------------------------------------------------------------


}
