/***************************************************************************
============================================================================
				UVic MECH 458
				FINAL PROJECT
			      Sorting Apparatus
============================================================================
***************************************************************************/

// ========== Include Libraries ============================================
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"
#include "LinkedQueue.h"

// ========== Function Initialization ======================================
void timer0();
void timer3();
void mTimer();
void DCmotor();
int initStepper();
int rotate();
void stepAccel();
void stepDecel();
void sort();
void display();
void pauseDisplay();
void rampdownDisplay();

/*
============================================================================
			Global Variables
============================================================================
*/

// ========== ADC Variables ================================================
volatile unsigned int ADC_result = 0;
volatile unsigned int ADC_min;
volatile unsigned int ADC_count = 0;

// ========== Counting Variables ===========================================
volatile unsigned int Alm = 0;
volatile unsigned int Stl = 0;
volatile unsigned int Wht = 0;
volatile unsigned int Blk = 0;
volatile unsigned int sortedAlm = 0;
volatile unsigned int sortedStl = 0;
volatile unsigned int sortedWht = 0;
volatile unsigned int sortedBlk = 0;

// ========== Stepper Variables ============================================
volatile unsigned int curr_position = 0;
volatile unsigned int belt = 0;
volatile unsigned int tray;
volatile unsigned int half = 100;
volatile unsigned int quart = 50;
volatile unsigned int minDelay = 6;
volatile unsigned int initDelay = 12;
volatile unsigned int decelDelay = 12;

// ========== State Variables and Timing ===================================
volatile unsigned int exitFlag = 0;
volatile unsigned int pauseFlag = 1;
volatile unsigned int temp;
volatile unsigned int millis = 0;

// ========== Stepper Array ================================================
char stepper_position[] = {0b00110110,0b00101110,0b00101101, 0b00110101};  	// Revised array values for dual-phase mode

// ========== Linked List Variables=========================================
link *head;
link *tail;
link *newLink;
link *rtnLink;
link *endhead;
link *endtail;
link *endLink;
link *endrtnLink;

// ========== DC Motor Speed ===============================================
int PWM	= 150;

/*
============================================================================
			Main Program
============================================================================
*/

int main(int argc,char*argv[])
{
// ========== Configure Maximum Frequency to 8 MHz =========================
CLKPR = (1 << CLKPCE); 						// Equivalent to CLKPR = 0b10000000
CLKPR = 0;

// ========== Disable all interrupts =======================================
cli();

// ========== Configure mTimer and Timer0 ==================================
TCCR1B |=_BV(CS11);						 	// mTimer prescaler 8
TCCR3B |=_BV(CS31);							// Timer3 8 prescale
timer0();									// Initialize timer0

// ========== Configure Interrupt 0 (Pause) ================================
EIMSK  |= _BV(INT0);                 		// Eternal Interrupt Mask Register - enable INT0
EICRA  |= _BV(ISC01);                		// External Interrupt Control Register A - falling edge interrupt

// ========== Configure Interrupt 1 (Ramp-Down) ============================
EIMSK  |= _BV(INT1);                 		// Eternal Interrupt Mask Register - enable INT0
EICRA  |= _BV(ISC11);                		// External Interrupt Control Register A - falling edge interrupt

// ========== Configure Interrupt 2 (Optical/RL) ===========================
EIMSK  |= _BV(INT2);                        // Eternal Interrupt Mask Register - enable INT2
EICRA  |= (_BV(ISC21) | _BV(ISC20));        // External Interrupt Control Register A - rising edge interrupt

// ========== Configure Interrupt 3 (EX Gate) ==============================
EIMSK  |= _BV(INT3);                        // Eternal Interrupt Mask Register - enable INT3
EICRA  |= _BV(ISC31);					    // External Interrupt Control Register A - falling edge interrupt

// ========== Configure ADC ================================================
ADCSRA |= _BV(ADEN);                        // ADEN - enable ADC
ADCSRA |= _BV(ADIE);                        // ADIE - enable interrupt of ADC
ADMUX  |= _BV(MUX0);						// MUX0 - Write one for PF1 of ADC
ADMUX  |= _BV(REFS0);    				    // REFS0 - Write one for voltage ref section - AVcc with external capacitor on on AREF pin; ADLAR=0

// ========== Configure Input and Output Ports =============================
DDRA = 0b10111111;							// Home/Stepper Motor
DDRB = 0b11111111;							// DC motor
DDRC = 0b11111111;							// LED/LCD
DDRD = 0b00000000;							// Interrupts (0 = Pause; 1 = Ramp-Down; 2 = OR; 3 = EX)
DDRF = 0b00000000;							// ADC

// ========== Initialize the LCD Screen ====================================
InitLCD(LS_BLINK|LS_ULINE);
LCDClear();

// ========== Globally enable interrupts ===================================
sei();                             			// Sets the Global Enable for all interrupts

// ========== Initialize and Configure Motors ==============================
initStepper();
mTimer(1000);
DCmotor(1);
half = 100 - (decelDelay - minDelay) - (initDelay - minDelay);
quart = half - 50;

// ========== Interrupt and Sensor Polling - Endless loop ==================
    while (1)
    {
		//display(); 						// For calibration purposes, uncomment to display calibration values
		if (exitFlag == 1)
		{
			initLink(&endLink);
			temp = head->e.itemCode;
			enqueue(&endhead, &endtail, &endLink);
			endhead->e.itemCode = temp;	
			dequeue(&head, &tail, &rtnLink);
			free(rtnLink);
			sort();
			DCmotor(1);
			exitFlag = 0;
		}
        
        if (millis > 10000)
        {
            rampdownDisplay();
        }
        
		else if (pauseFlag == 0)
		{
			pauseDisplay();
		}
		
	}
return(0);
}/*main*/


/*
============================================================================
				ISR
============================================================================
*/

// ========== Button1 (Pause) Interrupt ====================================
ISR(INT0_vect)
{
	if(pauseFlag == 1){
		DCmotor(0);
		LCDClear();
		mTimer(100);
		pauseFlag = 0;
	}
	else if(pauseFlag == 0){
		DCmotor(1);
		LCDClear();
		mTimer(100);
		pauseFlag = 1;
	}
}

// ========== Button2 (Ramp-Down) Interrupt  ===============================
ISR(INT1_vect)
{
    timer3();
}

// ========== Optical (Reflective) Sensor Interrupt ========================
ISR(INT2_vect)
{
	ADC_min = 1025;
	ADC_count = 0;
	belt++;
	ADCSRA |= _BV(ADSC);
	initLink(&newLink);
    enqueue(&head, &tail, &newLink);
}

// ========== EX Gate Interrupt ============================================
ISR(INT3_vect)
{
	DCmotor(0);	
	exitFlag = 1;
	belt--;
}

ISR(TIMER3_COMPA_vect)
{
    millis++;
}

// ========== ADC Interrupt ================================================
ISR(ADC_vect)
{
    ADC_result = ADC;
	if(ADC_min > ADC){
       	ADC_min = ADC;
   	}

	if((PIND & 0b00000100) == 0b00000000){
			
		ADC_count++;
	
		if(0 < ADC_min && ADC_min <= 60)  	     		// Aluminum
	    {
			newLink->e.itemCode = 1;
	        Alm++;
	    }
	    else if(60 < ADC_min && ADC_min <= 750) 		// Steel
	    {
			newLink->e.itemCode = 2;
	        Stl++;
	    }
	    else if(750 < ADC_min && ADC_min <= 890) 	 	// White
	    {
			newLink->e.itemCode = 3;
	        Wht++;
	    }
	    else if(ADC_min > 890)                  	 	// Black
	    {
			newLink->e.itemCode = 4;
	        Blk++;
		}
	}

	else
	{
		ADCSRA |= _BV(ADSC);
	}
}

// ========== BadISR Interrupt =============================================
ISR(BADISR_vect)
{
	DCmotor(0);
	LCDClear();
	LCDWriteStringXY(0,0,"BadISR");
	mTimer(5000);
}


/*
============================================================================
			Linked List
============================================================================
*/

/***************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/
void setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}/*setup*/

/***************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}/*initLink*/

/***************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail		
*  of the queue accordingly				
*  INPUT: the head and tail pointers, and a pointer to the new link that was created 
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		*h = *nL;
		*t = *nL;
	}/* else */
	return;
}/*enqueue*/

/***************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink' 
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h,link **t, link **deQueuedLink){
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (*h != NULL){
		*h = (*h)->next;
	}/*if*/
	
	/* If it is the last element */
	if (*h == NULL){
		*t = NULL; 		// Point the tail to NULL (This is the CORRECTION; added this line)
	}
	return;
}/*dequeue*/

/***************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/

/***************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
	
	/* Last but not least set the tail to NULL */
	*t = NULL;		

	return;
}/*clearQueue*/


/***************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	if (*h != NULL){
        return(0);
    }
    else{
        return(1);  // return(*h == NULL);
	}
}/*isEmpty*/


/***************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
int size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	int 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/
	
	return(numElements);
}/*size*/


/*
============================================================================
			Program Functions
============================================================================
*/

// ========== timer0 =======================================================
void timer0(){
    TCCR0A |=_BV(WGM00) | _BV(WGM01) | _BV(COM0A1); // Step 1 - This will set the WGM0 and WGM1 bits
                                                    // Step 3 - Set Compare Match Output mode to clear on match and set output compare when timer reaches TOP
    TCCR0B |=_BV(CS01);                             // Step 4 - Set the Clock Select bits for prescaling 8
	TCCR0B |=_BV(CS00);								// Step 4 - Set the Clock Select bits for prescaling 64 - Keep CS01

	OCR0A = PWM;									// DCmotor Speed 0 < Speed < 255 // Step 5 - Set Output compare Register for 128 cycles = 1ms
return;
}/*timer0*/


void timer3(){

    TCCR3B |=_BV(WGM32);    /* This will set the WGM bits to 0100. timer resets on its own from WGM setting */
    OCR3A = 0x03e8;         /* Set Output compare Register for 1000 cycles = 1ms*/
    TCNT3 = 0x0000;         /* Set the initial value of the Timer Counter to 0x0000*/
    TIMSK3 = TIMSK3 | 0b00000010;
    TIFR3 |=_BV(OCF3A);     /* Reset the flag and clear the timer interrupt flag and begin timer*/

    return;
}/*timer3*/

// ========== mTimer =======================================================
void mTimer(int count){
    int i = 0;              /* used to keep track of the loop */

    TCCR1B |=_BV(WGM12);    /* This will set the WGM bits to 0100. timer resets on its own from WGM setting */
    OCR1A = 0x03e8;         /* Set Output compare Register for 1000 cycles = 1ms*/
    TCNT1 = 0x0000;         /* Set the initial value of the Timer Counter to 0x0000*/
    TIFR1 |=_BV(OCF1A);     /* Reset the flag and clear the timer interrupt flag and begin timer*/
    while(i<count){                         /*Poll the timer to determine when the timer has reached 0x03e8*/
        if((TIFR1 & 0x02) == 0x02){

            TIFR1|=_BV(OCF1A);              /* Clear the interrupt flag by writing a ONE to the bit*/
            i++;                            /* Increment the loop counter*/
            }                               /* End of timer for loop, counts 1000 oscilations for 1 ms of time*/
            
        }                                   /* End of while loop */
    return;
}/*mTimer*/

// ========== DCmotor ======================================================
void DCmotor(int DCrun){
	if (DCrun == 1){
        PORTB = 0b00000001;					// Run conveyor belt
	}
	else if (DCrun == 0){
		PORTB = 0b00000000; 				// Send 0 to brake
	}
    return;
}/*DCmotor*/

int initStepper(){
	// Warm-up motor for 50 steps
	for(int i=0; i<50;i++){
		curr_position++;
		
		if(curr_position>3){
			curr_position = 0;
		}//end if
		
		PORTA = stepper_position[curr_position]; 	// Store control signal order
		
		mTimer(12);
	}//end for
	
	while((PINA & 0b01000000) == 0b01000000){
		curr_position++;
		
		if(curr_position>3){
			curr_position = 0;
		}//end if
		
		PORTA = stepper_position[curr_position]; 	// Store control signal order
		
		mTimer(12);
	
	}//end while
	tray = 4;
	return (curr_position);
}/*initStepper*/

// ========== rotate =======================================================
int rotate(int dir, int step, int delay){
	int j = curr_position; 							// j is local variable, curr_position is global
	
	// Clockwise (dir == 1)
	if(dir == 1){
		for(int i=0; i<step; i++){
			j++; 									//Increment
			if(j>3){
				j = 0; 								//if reached end of array, return to initial step
			}//end if
			PORTA = stepper_position[j]; 			//Store control signal order
			mTimer(delay);
		}//end for
	}//end if
	
	// CCW (dir == 0)
	if(dir == 0){
		for(int i=0; i<step; i++){
			j--; 									//Decrement
			if(j<0){
				j = 3; 								//if reached end of array, return to initial step
			}//end if
			PORTA = stepper_position[j]; 			//Store control signal order
			mTimer(delay);
		}//end for
	}//end if
	curr_position = j; 								//Store local variable j into global variable current
return 0;
}/*rotate*/

// ========== stepAccel ====================================================
void stepAccel(int dir){
  int delay = initDelay;
    while(delay>minDelay){
        rotate(dir,1,delay);
        delay--;
    }
return;
}/*stepAccel*/

// ========== stepDecel ====================================================
void stepDecel(int dir){
    int delay = minDelay;
    while (delay<decelDelay){   
        rotate(dir,1,delay);
        delay++;
    }
return;
}/*stepDecel*/

// ========== Sort =========================================================
void sort(){
	int type = endhead->e.itemCode;
	switch(type)
	{
	case(1):								// Aluminum (Tray = 1)
		Alm--;
		sortedAlm++;
		if (tray == 1){
		mTimer(40);
		break;
		}
		else if (tray == 2){
		stepAccel(1);	
		rotate(1,half,minDelay);
		stepDecel(1);
		mTimer(40);
		tray = 1;
		break;
		}
		else if (tray == 3){
		stepAccel(0);	
		rotate(0,quart,minDelay);
		stepDecel(0);
		mTimer(20);
		tray = 1;
		break;
		}
		else // tray == 4
		stepAccel(1);
		rotate(1,quart,minDelay);
		stepDecel(1);
		mTimer(20);
		tray = 1;
		break;
	case(2):								// Steel (Tray = 2)
		Stl--;
		sortedStl++;
		if (tray == 1){
		stepAccel(1);
		rotate(1,half,minDelay);
		stepDecel(1);
		mTimer(40);
		tray = 2;
		break;
		}
		else if (tray == 2){
		mTimer(40);
		break;
		}
		else if (tray == 3){
		stepAccel(1);
		rotate(1,quart,minDelay);
		stepDecel(1);
		mTimer(20);
		tray = 2;
		break;
		}
		else // tray == 4
		stepAccel(0);
		rotate(0,quart,minDelay);
        stepDecel(0);
		mTimer(20);
		tray = 2;
		break;
	case(3):								// White (Tray = 3)
		Wht--;
		sortedWht++;
		if (tray == 1){
        stepAccel(1);
		rotate(1,quart,minDelay);
		stepDecel(1);
		mTimer(20);
		tray = 3;
		break;
		}
		else if (tray == 2){
		stepAccel(0);
		rotate(0,quart,minDelay);
		stepDecel(0);
		mTimer(20);
		tray = 3;
		break;
		}
		else if (tray == 3){
		mTimer(40);
		break;
		}
		else // tray == 4
		stepAccel(1);
		rotate(1,half,minDelay);
		stepDecel(1);
		mTimer(40);
		tray = 3;
		break;
	case(4):								// Black (Tray = 4)
		Blk--;
		sortedBlk++;
		if (tray == 1){
		stepAccel(0);
		rotate(0,quart,minDelay);
		stepDecel(0);
		mTimer(20);
		tray = 4;
		break;
		}
		else if (tray == 2){
		stepAccel(1);
		rotate(1,quart,minDelay);
		stepDecel(1);
		mTimer(20);
		tray = 4;
		break;
		}
		else if (tray == 3){	
		stepAccel(1);
		rotate(1,half,minDelay);
        stepDecel(1);
		mTimer(40);
		tray = 4;
		break;
		}
		else // tray == 4
		mTimer(40);
		break;
    }
	dequeue(&endhead, &endtail, &endrtnLink);
	free(endrtnLink);
    return;
}/*sort*/

// ========== display ======================================================
void display()
{
	LCDWriteStringXY(0,0,"A S W B ");
	LCDWriteIntXY(0,1,sortedAlm,1);
	LCDWriteStringXY(2,1,"  ");
	LCDWriteIntXY(2,1,sortedStl,1);
	LCDWriteStringXY(6,1,"  ");
	LCDWriteIntXY(4,1,sortedWht,1);
	LCDWriteStringXY(10,1,"  ");
	LCDWriteIntXY(6,1,sortedBlk,1);
	LCDWriteStringXY(14,1,"  ");
	LCDWriteIntXY(10,1,ADC_min,5);
return;
}/*display*/

void pauseDisplay()
{
	LCDWriteStringXY(0,0," A  S  W  B Belt");
	LCDWriteIntXY(0,1,sortedAlm,2);
	LCDWriteStringXY(2,1," ");
	LCDWriteIntXY(3,1,sortedStl,2);
	LCDWriteStringXY(5,1," ");
	LCDWriteIntXY(6,1,sortedWht,2);
	LCDWriteStringXY(8,1," ");
	LCDWriteIntXY(9,1,sortedBlk,2);
	LCDWriteStringXY(14,1," ");
	LCDWriteIntXY(14,1,belt,2);
return;
}/*pauseDisplay*/

void rampdownDisplay()
{
    mTimer(1000);
    DCmotor(0);
    cli();
    LCDClear();
    while(1)
    {
        LCDClear();
        LCDWriteStringXY(0,0,"Sorted          ");
        mTimer(3000);
        LCDClear();
        LCDWriteStringXY(0,0,"Alm Stl Wht Blk ");
        LCDWriteIntXY(0,1,sortedAlm,2);
        LCDWriteStringXY(2,1,"  ");
        LCDWriteIntXY(4,1,sortedStl,2);
        LCDWriteStringXY(6,1,"  ");
        LCDWriteIntXY(8,1,sortedWht,2);
        LCDWriteStringXY(10,1,"  ");
        LCDWriteIntXY(12,1,sortedBlk,2);
        LCDWriteStringXY(14,1,"  ");
        mTimer(5000);
    }
    return;
}/*rampdownDisplay*/
