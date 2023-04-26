#include "functions.h"
extern int16_t speedRef_L[1000]; //10s at 100Hz
extern int16_t speedRef_R[1000]; //10s at 100Hz

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

void generateStep(uint16_t t0, uint16_t value0, uint16_t t1, uint16_t value1, uint16_t samples, uint16_t  data[])
{
	/* Generate a 2 step function : from 0 to t0 => data=0 / from t0 to t1 => data=value0 / from t1 to samples => data=value1
	*/
	for (int i=0; i<samples ; i++)
	{
		if (i<t0)
		{
			data[i]=0; //default
		}
		if (i>=t0 && i<t1)
		{
			data[i]=value0; //value0 at t0
		}
		if (i>=t1)
		{
			data[i]=value1; //value1 at t1
		}
	}
}

void readHall_L(struct hall *hall)
{
	hall->h1=HAL_GPIO_ReadPin(H1_L_GPIO_Port, H1_L_Pin);
	hall->h2=HAL_GPIO_ReadPin(H2_L_GPIO_Port, H2_L_Pin);
	hall->h3=HAL_GPIO_ReadPin(H3_L_GPIO_Port, H3_L_Pin);
}

void readHall_R(struct hall *hall)
{
	hall->h1=HAL_GPIO_ReadPin(H1_R_GPIO_Port, H1_R_Pin);
	hall->h2=HAL_GPIO_ReadPin(H2_R_GPIO_Port, H2_R_Pin);
	hall->h3=HAL_GPIO_ReadPin(H3_R_GPIO_Port, H3_R_Pin);
}

void decodeHall(struct hall *hall,struct PWM *PWM)
{
    /*
    Commutation sequence for MAXON EC45 BLDC MOTOR
    */

	hall->h123= (hall->h1<<2) | (hall->h2<<1) | hall->h3; //for motor sense reading

	if (PWM->sense) //clockwise
	{
		PWM->aH= (hall->h1)&&(!hall->h2);	PWM->aL= (!hall->h1)&&(hall->h2);	PWM->bH= (hall->h2)&&(!hall->h3);	PWM->bL= (!hall->h2)&&(hall->h3);	PWM->cH= (!hall->h1)&&(hall->h3);	PWM->cL= (hall->h1)&&(!hall->h3);
	}
	else //anti-clockwise
	{
		PWM->aH= (!hall->h1)&&(hall->h2);	PWM->aL= (hall->h1)&&(!hall->h2);	PWM->bH= (!hall->h2)&&(hall->h3);	PWM->bL= (hall->h2)&&(!hall->h3);	PWM->cH= (hall->h1)&&(!hall->h3);	PWM->cL= (!hall->h1)&&(hall->h3);
	}

	if (hall->h123 != hall->prev_h123){ //for sense sign reading
		switch(hall->h123) // 101 -> 100 -> 110 -> 010 -> 011 -> 001 clockwise
		{
			case 1:
				if(hall->prev_h123 == 3)	(hall->tickS)++;	//clockwise
				else	(hall->tickS)--;
				break;				//anti clockwise
			case 2:
				if(hall->prev_h123 == 6)	(hall->tickS)++;
				else	(hall->tickS)--;
				break;
			case 3:
				if(hall->prev_h123 == 2)  	(hall->tickS)++;
				else	(hall->tickS)--;
				break;
			case 4:
				if(hall->prev_h123 == 5) 	(hall->tickS)++;
				else	(hall->tickS)--;
				break;
			case 5:
				if(hall->prev_h123 == 1) 	(hall->tickS)++;
				else	(hall->tickS)--;
				break;
			case 6:
				if(hall->prev_h123 == 4) 	(hall->tickS)++;
				else	(hall->tickS)--;
				break;
			default:
				break;
		}
	}
	hall->prev_h123 = hall->h123;
}

void updateOutput_L(struct PWM PWM)
{
	if (PWM.aH)	TIM1->CCER |= TIM_CCER_CC1E; 	//CC1E=1
	else	TIM1->CCER &= ~(TIM_CCER_CC1E); 	//CC1E=0

	if (PWM.aL)	TIM1->CCER |= TIM_CCER_CC1NE; 	//CC1NE=1
	else	TIM1->CCER &= ~(TIM_CCER_CC1NE); 	//CC1NE=0

	if (PWM.bH)	TIM1->CCER |= TIM_CCER_CC2E;
	else	TIM1->CCER &= ~(TIM_CCER_CC2E);

	if (PWM.bL)	TIM1->CCER |= TIM_CCER_CC2NE;
	else	TIM1->CCER &= ~(TIM_CCER_CC2NE);

	if (PWM.cH)	TIM1->CCER |= TIM_CCER_CC3E;
	else	TIM1->CCER &= ~(TIM_CCER_CC3E);

	if (PWM.cL)	TIM1->CCER |= TIM_CCER_CC3NE;
	else	TIM1->CCER &= ~(TIM_CCER_CC3NE);
}

void updateOutput_R(struct PWM PWM)
{
	if (PWM.aH)	TIM8->CCER |= TIM_CCER_CC1E; 	//CC1E=1
	else	TIM8->CCER &= ~(TIM_CCER_CC1E); 	//CC1E=0

	if (PWM.aL)	TIM8->CCER |= TIM_CCER_CC1NE; 	//CC1NE=1
	else	TIM8->CCER &= ~(TIM_CCER_CC1NE); 	//CC1NE=0

	if (PWM.bH)	TIM8->CCER |= TIM_CCER_CC2E;
	else	TIM8->CCER &= ~(TIM_CCER_CC2E);

	if (PWM.bL)	TIM8->CCER |= TIM_CCER_CC2NE;
	else	TIM8->CCER &= ~(TIM_CCER_CC2NE);

	if (PWM.cH)	TIM8->CCER |= TIM_CCER_CC3E;
	else	TIM8->CCER &= ~(TIM_CCER_CC3E);

	if (PWM.cL)	TIM8->CCER |= TIM_CCER_CC3NE;
	else	TIM8->CCER &= ~(TIM_CCER_CC3NE);
}

int16_t calculateSpeed(struct hall *hall)
{
	/*
	 * Calculate motor speed using hall effect sensor
	 * called every 10ms (100Hz)
	 * return measurement_L.speed in tick/s
	 * resolution : 1/p mechanical turn  -> 48ticks for 1turn
	 */
	if (hall->tickS > 240)
		return (int16_t) MAX_SPEED;
	else if (hall->tickS < -240)
		return (int16_t) MIN_SPEED;
	else
		return 100*hall->tickS;
}

void writeCommand_L(struct PWM *PWM, struct dataSpeed *data)
{
	if ((data->cmdsat >= 0) && (data->cmdsat <=DC_MAX))			//clockwise
	{
		PWM->sense=1;	TIM1->CCR1=data->cmdsat;	TIM1->CCR2=data->cmdsat;	TIM1->CCR3=data->cmdsat;
	}
	else if ((data->cmdsat >= DC_MIN) && (data->cmdsat < 0))	//anti clockwise
	{
		PWM->sense=0;	TIM1->CCR1=-data->cmdsat;	TIM1->CCR2=-data->cmdsat;	TIM1->CCR3=-data->cmdsat;
	}
	else														//error state --> PWM off
	{
		PWM->sense=1;	TIM1->CCR1=0;	TIM1->CCR2=0;	TIM1->CCR3=0;
	}
}

void writeCommand_R(struct PWM *PWM, struct dataSpeed *data)
{
	if ((data->cmdsat >= 0) && (data->cmdsat <=DC_MAX))			//clockwise
	{
		PWM->sense=1;	TIM8->CCR1=data->cmdsat;	TIM8->CCR2=data->cmdsat;	TIM8->CCR3=data->cmdsat;
	}
	else if ((data->cmdsat >= DC_MIN) && (data->cmdsat < 0))	//anti clockwise
	{
		PWM->sense=0;	TIM8->CCR1=-data->cmdsat;	TIM8->CCR2=-data->cmdsat;	TIM8->CCR3=-data->cmdsat;
	}
	else														//error state --> PWM off
	{
		PWM->sense=1;	TIM8->CCR1=0;	TIM8->CCR2=0;	TIM8->CCR3=0;
	}
}

void PIDS_Prop(struct PID *Prop, struct dataSpeed *data_L,struct dataSpeed *data_R, struct hall *hall_L, struct hall *hall_R){

	//read speed
	data_L->speed=calculateSpeed(hall_L);	hall_L->tickS=0;
	data_R->speed=calculateSpeed(hall_R);	hall_R->tickS=0;

	//Left
	data_L->error = data_L->speed_ref - data_L->speed;
	data_L->cmd = (int16_t)((float)data_L->error)*Prop->Kp;

	//Right
	data_R->error = data_R->speed_ref - data_R->speed;
	data_R->cmd = (int16_t)((float)data_R->error)*Prop->Kp;

	//Saturation left
	if (data_L->cmd < DC_MIN)	data_L->cmdsat = (int16_t) DC_MIN;
	else if (data_L->cmd > DC_MAX)	data_L->cmdsat = (int16_t) DC_MAX;
	else	data_L->cmdsat = data_L->cmd;

	//Saturation right
	if (data_R->cmd < DC_MIN)	data_R->cmdsat = (int16_t) DC_MIN;
	else if (data_R->cmd > DC_MAX)	data_R->cmdsat = (int16_t) DC_MAX;
	else	data_R->cmdsat = data_R->cmd;
}

void PIDS_PI(struct PID *PI, struct dataSpeed *data_L,struct dataSpeed *data_R, struct hall *hall_L, struct hall *hall_R){

	//read speed
	data_L->speed=calculateSpeed(hall_L);	hall_L->tickS=0;
	data_R->speed=calculateSpeed(hall_R);	hall_R->tickS=0;

	//Left
	data_L->error = data_L->speed_ref - data_L->speed;
	data_L->cmd = (int16_t)(data_L->prev_cmd + PI->a0*((float)data_L->error) - PI->a1*((float)data_L->prev_error)); //PI
	//FIXME : PI windup

	//Right
	data_R->error = data_R->speed_ref - data_R->speed;
	data_R->cmd = (int16_t)(data_R->prev_cmd + PI->a0*((float)data_R->error) - PI->a1*((float)data_R->prev_error)); //PI
	//FIXME : PI windup

	//Saturation left
	if (data_L->cmd < DC_MIN)	data_L->cmdsat = (int16_t) DC_MIN;
	else if (data_L->cmd > DC_MAX)	data_L->cmdsat = (int16_t) DC_MAX;
	else	data_L->cmdsat = data_L->cmd;

	//Saturation right
	if (data_R->cmd < DC_MIN)	data_R->cmdsat = (int16_t) DC_MIN;
	else if (data_R->cmd > DC_MAX)	data_R->cmdsat = (int16_t) DC_MAX;
	else	data_R->cmdsat = data_R->cmd;
}

bool parseurSpeed(float distance, int16_t angle ,uint16_t speed_ref)
{
	uint16_t maxInitL = 1500,  maxInitR = 3000;
	float fin = -0.15;

	uint16_t minL = 1500, minR = 2500; // si non nul "recule"
	float dif_dur = 0;


	for (uint16_t j=0; j<1000 ; j++)    //init speed consigne
	{
		speedRef_L[j]=0;  speedRef_R[j]=0;
	}

	if(distance != 0)
	{
		float durL = 1.;
		float durR = 1.;
		uint16_t nb_fin = 0;
		uint16_t t100Hz_dist = 0;
		if (distance > 0)
		{
			t100Hz_dist = (uint16_t) 480*distance/((float) speed_ref); //error flag if overflow

			nb_fin = (uint16_t)(fin*t100Hz_dist);
			if (dif_dur < 0){
				durL = nb_fin*(1+dif_dur);
			}
			else if (dif_dur > 0){
				durR = nb_fin*(1-dif_dur);
			}
		}
		else if (distance < 0){
			t100Hz_dist = (uint16_t) -distance/((float) speed_ref); //error flag if overflow

			nb_fin = (uint16_t)(fin*t100Hz_dist);
			if (dif_dur < 0){
				durL = nb_fin*(1+dif_dur);
			}
			else if (dif_dur > 0){
				durR = nb_fin*(1-dif_dur);
			}
		}


		if(t100Hz_dist > 1000-2-nb_fin)  return 1; //error flag if overflow

    	// distance > 0 -> L - R +
		// distance < 0 -> L + R -

		if (distance > 0){
			for (uint16_t j=0; j<2; j++) 							// pick
			{
				speedRef_L[j] = -maxInitL; speedRef_R[j] = maxInitR;		// temps haut
			}
			for (uint16_t j=2; j<t100Hz_dist-nb_fin-1; j++)
			{
				speedRef_L[j] = -speed_ref; speedRef_R[j] = speed_ref;
			}
			for (uint16_t j = t100Hz_dist-nb_fin-1; j < t100Hz_dist-1; j++) // deceleration
			{
				float t = j/(t100Hz_dist-nb_fin-1);
				// Left
				if (t < durL){
					speedRef_L[j] = minL;
				}
				else {
					speedRef_L[j] = 0;
				}
				// Right
				if (t < durR){
					speedRef_R[j] = -minR;
				}
				else {
					speedRef_R[j] = 0;
				}
			}
			for (uint16_t j=t100Hz_dist-1; j<t100Hz_dist; j++)
			{
				speedRef_L[j]=0; speedRef_R[j]=0;
			}
		}
		else // distance < 0 // L + R -
		{
			for (uint16_t j=0; j<2; j++) 							// pick
			{
				speedRef_L[j] = maxInitL; speedRef_R[j] = -maxInitR;		// temps haut
			}
			for (uint16_t j=2; j<t100Hz_dist-nb_fin-1; j++)
			{
				speedRef_L[j] = speed_ref; speedRef_R[j] = -speed_ref;
			}
			for (uint16_t j = t100Hz_dist-nb_fin-1; j < t100Hz_dist-1; j++) // deceleration
			{
				float t = j/(t100Hz_dist-nb_fin-1);
				// Left
				if (t < durL){
					speedRef_L[j] = (int)(speed_ref + (minL - speed_ref)*t/(durL));
				}
				else {
					speedRef_L[j] = -minL;
				}
				// Right
				if (t < durR){
					speedRef_R[j] = -(int)(speed_ref + (minR - speed_ref)*t/(durR));
				}
				else {
					speedRef_R[j] = minR;
				}
			}
			for (uint16_t j=t100Hz_dist-1; j<t100Hz_dist; j++)
			{
				speedRef_L[j]=0; speedRef_R[j]=0;
			}
		}
	}
  return 0;  //OK
}

void generateSpeed()
{
	for(int i=0; i<1000;i++)
	  {
		  if(i<=100)
		  {
			  speedRef_L[i]=-500;
			  speedRef_R[i]=500;
		  }
		  if(i>100 && i<400) //1s a 4s
		  {
			  speedRef_L[i]=-1000;
			  speedRef_R[i]=1000;
		  }
		  else
		  {
			  speedRef_L[i]=0;
			  speedRef_R[i]=0;
		  }
	  }
}
