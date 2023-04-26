/*
 * functions.h
 *
 *  Created on: 15 mai 2022
 *      Author: Vincent Quattrin
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>

typedef enum {false, true} bool; //and not stdbool.h library

// !!WARNING : IF CHANGED DEFINE, UPDATE MAIN.h FILE !!
#define P 8
#define FREQ_TIM7 10000
#define MAX_SPEED 20000
#define MIN_SPEED -20000
#define DC_MIN -85
#define DC_MAX 85



struct hall
{
	bool h1;
	bool h2;
	bool h3;
	char h123;
	char prev_h123;	//to determine sense of rotation

	int16_t tickS; 	//to calculate speed
	int32_t tickP;	//to calculate position
};


struct PWM
{
	//channels enable bits
	 bool aH;
	 bool aL;
	 bool bH;
	 bool bL;
	 bool cH;
	 bool cL;
	 bool sense; //0=motor turns anti-clockwise 1=motor turns clockwise
};

struct dataSpeed{
	 int16_t speed_ref;
	 int16_t speed;

	 int16_t error;
	 int16_t prev_error;

	 int16_t cmd;
	 int16_t prev_cmd;
	 int16_t cmdsat;
	 int16_t prev_cmdsat;
};

struct dataPosition{
	 int16_t pos_ref;
	 int16_t pos;

	 int16_t error;
	 int16_t prev_error;

	 int16_t setSpeed_ref;
	 int16_t prev_setSpeed_ref;
	 int16_t setSpeed_refsat;
	 int16_t prev_setSpeed_refsat;
};


struct PID{
	const float Kp;
	const float w_i;
	const float Te;

	//integration coeff : PI(z) = setSpeed_ref(z)/error(z) = Kp*(a0*z - 1)/z-1 = Kp*(a0-a1*z^{-1})/(1-z^{-1})
	const float a0;
	const float a1;
};


int _write(int file, char *ptr, int len);

void generateStep(uint16_t t0, uint16_t value0, uint16_t t1, uint16_t value1, uint16_t samples, uint16_t  data[]);

void readHall_L(struct hall *hall);
void readHall_R(struct hall *hall);

void decodeHall(struct hall *hall,struct PWM *PWM);

void commutation(struct hall *hall_L,struct hall *hall_R,struct PWM *PWM_L,struct PWM *PWM_R);

void updateOutput_L(struct PWM PWM);
void updateOutput_R(struct PWM PWM);

int16_t calculateSpeed(struct hall *hall);

void writeCommand_L(struct PWM *PWM, struct dataSpeed *data);
void writeCommand_R(struct PWM *PWM, struct dataSpeed *data);

void PIDS_Prop(struct PID *Prop, struct dataSpeed *data_L,struct dataSpeed *data_R, struct hall *hall_L, struct hall *hall_R); //Speed P corrector
void PIDS_PI(struct PID *Prop, struct dataSpeed *data_L,struct dataSpeed *data_R, struct hall *hall_L, struct hall *hall_R); //Speed PI corrector

volatile bool parseurSpeed(float distance, int16_t angle, uint16_t speed_ref);

void generateSpeed();
#endif /* INC_FUNCTIONS_H_ */
