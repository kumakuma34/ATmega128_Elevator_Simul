#include "includes.h"
#include <avr/io.h>	
#include <util/delay.h>

#define F_CPU	16000000UL	// CPU frequency = 16 Mhz
#define  N_TASKS        4
#define ON 1
#define OFF 0
#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE            
#define UCHAR unsigned char // UCHAR 정의
#define LED PD4
OS_STK       TaskStk[N_TASKS][TASK_STK_SIZE];
//OS_STK          Start[TASK_STK_SIZE];
OS_EVENT     *Mbox;//mailbox
OS_EVENT	*MsgQueue;//msg queue
OS_EVENT     *sem;//semaphore
OS_EVENT *sem1;
OS_EVENT *sem2;
OS_FLAG_GRP *f_grp;//Event Flag
volatile unsigned char fnd_sel[4] = { 0x01,0x02,0x04,0x08 };
volatile unsigned char num[8] = { 0x01,0x02,0x04,0x08,0x10, 0x20, 0x40, 0x80 };
volatile unsigned char digit[11] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x6f, 0x37 };
volatile int state = OFF;
volatile int stop = 1;

void  LedTask();
void StartTask(void *data);
void InitI2C();
int ReadTemperature(void);
void TemperatureTask(void *data);//온도 값을 읽어오는 Task
void FndDisplayTask(void *data);
void moveAfter(void *pdata);//사용자가 탄 후 엘레베이터의 위치를 조정하는 Task

void *Msq_Table[2];//message queue의 결과를 담을 Table

int elevate1, elevate2;//엘레베이터 1의 위치와 엘레베이터 2의 위치.
int toLocation1, toLocation2;//엘레베이터 1이 가야하는 위치와 엘레베이터 2가 가야하는위치
int up1 = -1;//0이면 1번 엘레베이터가 내려감, 0이면 올라감.
int up2 = -1;//0이면 2번 엘레베이터가 내려감, 0이면 올라감
int move1 = 0;
int move2 = 0;//1이번 엘레베이터를 움직여야 하는 상황, 0이라면 움직이지 않아도 되는 상황
int buzzerdone1 = 0;//0이면 도착후 버저를 아직 울리지 않음. 1이면 이미 울림., 1번 엘레베이터
int buzzerdone2 = 0;//0이면 도착후 버저를 아직 울리지 않음. 1이면 이미 울림. 2번 엘레베이터
int alert = 0;//0이번 위험 경보가 울리지 않는 상황, 1이면 위험 경보가 울리는 상황
INT8U err;
INT8U now_location = 3;//사용자가 현재 있는 위치. 임의로 지정
int TCN_val[8] = { 17,43,66,77,97,114,129,137 };
int main(void)
{
	OSInit();

	OS_ENTER_CRITICAL();
	TCCR0 = 0x03;
	TIMSK = 0x01;

	TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
	OS_EXIT_CRITICAL();
	DDRC = 0xff;
	DDRB = 0x10;
	DDRG = 0x0f;
	DDRA = 0xff;
	DDRE = 0xcf;
	EICRB = 0x0A;
	EIMSK = 0x30;
	TCNT0 = TCN_val[7];
	SREG |= 1 << 7;
	PORTG = 0x0f;

	sem = OSSemCreate(1);
	Mbox = OSMboxCreate(0);
	MsgQueue = OSQCreate((void*)Msq_Table, 2);
	f_grp = OSFlagCreate(0x00, &err);

	//각종 Event 객체들 초기화

	OSTaskCreate(FndDisplayTask, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 0);
	OSTaskCreate(StartTask, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 1);
	OSTaskCreate(TemperatureTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(moveAfter, (void *)0, (void *)&TaskStk[3][TASK_STK_SIZE - 1], 3);
	//Task 만들기

	OSStart();
	return 0;
}
ISR(TIMER0_OVF_vect)
{
	if (stop == 0)
	{
		if (state == ON) {
			PORTB = 0x00;
			state = OFF;
		}
		else {
			PORTB = 0x10;
			state = ON;
		}
		TCNT0 = TCN_val[5];
	}
	if (alert == 1)
	{
		if (state == ON) {
			PORTB = 0x00;
			state = OFF;
		}
		else {
			PORTB = 0x10;
			state = ON;
		}
		TCNT0 = TCN_val[7];
	}
}
ISR(INT4_vect)//up 스위치가 눌린 경우. 상향 엘레베이터를 호출한 상황 시뮬레이션. 
{
	int diff1, diff2;//각각 두 엘레베이터와 현재 사용자와이 거리 차이.
	int which;// 어떤 엘레베이터를 움직일 것인지 선택
	diff1 = now_location - elevate1;
	diff2 = now_location - elevate2;
	if (now_location < elevate1)
		up1 = 0;
	else if (now_location == elevate1)
		up1 = -1;
	else
		up1 = 1;
}
ISR(INT5_vect)//up 스위치가 눌린 경우. 상향 엘레베이터를 호출한 상황 시뮬레이션. 
{
	int diff1, diff2;//각각 두 엘레베이터와 현재 사용자와이 거리 차이.
	int which;// 어떤 엘레베이터를 움직일 것인지 선택
	diff1 = now_location - elevate1;
	diff2 = now_location - elevate2;
	if (now_location < elevate2)
		up2 = 0;
	else
		up2 = 1;
}
void InitI2C()
{
	PORTD = 3;                   // For Pull-up override value
	SFIOR &= ~(1 << PUD);          // PUD
	TWSR = 0;                   // TWPS0 = 0, TWPS1 = 0
	TWBR = 32;                  // for 100  K Hz bus clock
	TWCR = _BV(TWEA) | _BV(TWEN);   // TWEA = Ack pulse is generated
}
int ReadTemperature(void)
{
	int value;

	TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);   // START 전송
	while (!(TWCR & _BV(TWINT)));            // ACK를 기다림

	TWDR = 0x98 + 1;                      //TEMP_I2C_ADDR + 1 > SLA+R 준비, R=1
	TWCR = _BV(TWINT) | _BV(TWEN);            // SLA+R 전송
	while (!(TWCR & _BV(TWINT)));            // ACK를 기다림

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);   // 1st DATA 준비
	while (!(TWCR & _BV(TWINT)));            // ACK를 기다림

											 //온도센서는 16bit 기준으로 값을 가져오므로
											 //8비트씩 2번을 받아야 한다.
	value = TWDR << 8;                     // 1 byte DATA 수신
	TWCR = _BV(TWINT) | _BV(TWEN);             // SLA+R 전송
	while (!(TWCR & _BV(TWINT)));            // ACK를 기다림

	value |= TWDR;                        // 1 byte DATA 수신
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);   // STOP 전송

	value >>= 8;

	TIMSK = (value >= 33) ? TIMSK | _BV(TOIE2) : TIMSK & ~_BV(TOIE2);

	return value;
}
void moveAfter(void *pdata)//사용자가 탄 후 엘레베이터의 위치를 조정하는 Task
{
	int a, b;
	while (1)
	{
		if (move1 == 1)
		{
			a = rand() % 20;
			toLocation1 = a;
			if (toLocation1 - now_location > 0)
				up1 = 1;
			else
				up1 = 0;
			move1 = 0;
			OSFlagPost(f_grp, 0x01, OS_FLAG_SET, &err);
		}//1번 엘레베이터가 가야할 위치를 랜덤하게 만든 후 FLagPost를 해준다.
		if (move2 == 1)
		{
			b = rand() % 20;
			toLocation2 = b;
			if (toLocation2 - now_location > 0)
				up2 = 1;
			else
				up2 = 0;
			move2 = 0;
			OSFlagPost(f_grp, 0x02, OS_FLAG_SET, &err);
		}//2번 엘레베이터가 가야 할위치를 랜덤하게 설정 한 후 FlagPost를 해준다.
	}
}
void TemperatureTask(void *data)
{
	int   value;
	INT8U err;
	//data = data;
	InitI2C();
	while (1)
	{
		if(move1 == 1)
			OSFlagPend(f_grp, 0x01, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &err);
		if(move2 == 1)
			OSFlagPend(f_grp, 0x02, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &err);

		LedTask();//현재 층 수를 LED에 표시해준다.
		OSSemPend(sem, 0, &err);
		value = ReadTemperature();//온도값을 읽는다
		OSSemPost(sem);
		OSMboxPost(Mbox, (void*)&value);//읽은 온도값을 MailBox에 Post한다.
	}
}

void StartTask(void *data)
{
	int i;
	int temperature;
	volatile int count = 0;
	INT8U err;
	int random_a = rand() % 20; // 0~19
	int random_b = rand() % 20;//set two elevator's location randomly
	elevate1 = random_a;
	elevate2 = random_b;
	data = data;
	//InitI2C();
	while (1) {
		//LedTask();
		temperature = *(int *)OSMboxPend(Mbox, 0, &err);//MailBox에 담겨져 있는 온도 값을 읽어온다.
		if (temperature <= 28)//화재경보가 울리지 않는 겨웅이다.
		{
			alert = 0;
			OSQPost(MsgQueue, (void*)&elevate1);
			OSQPost(MsgQueue, (void*)&elevate2);//message queue에 각 엘레베이터의위치 post
			if (up1 == 1)
			{
				elevate1++;
				if (elevate1 == now_location || toLocation1 == elevate1)
					up1 = -1;
			}
			else if (up1 == 0)
			{
				elevate1--;
				if (elevate1 == now_location || toLocation1 == elevate1)
					up1 = -1;
			}

			if (up2 == 1)
			{
				elevate2++;
				if (elevate2 == now_location || toLocation2 == elevate2)
					up2 = -1;
			}
			else if (up2 == 0)
			{
				elevate2--;
				if (elevate2 == now_location || toLocation2 == elevate2)
					up1 = -1;
			}
			if ((now_location == elevate1 && buzzerdone1 == 0))
			{
				up1 = -1;
				stop = 0;
				_delay_ms(300);
				buzzerdone1 = 1;
				move1 = 1;//사용자가 탑승하였고, 1번 엘레베이터가 움직여야 한다는 것을 나타낸다.
			}
			if ((now_location == elevate2 && buzzerdone2 == 0))
			{
				up2 = -1;
				stop = 0;
				_delay_ms(300);
				buzzerdone2 = 1;
				move2 = 1;//사용자가 탑승하엿고 2번 엘레베이터가 움직여야 한다는 것을 나타낸다.
			}
			stop = 1;
		}
		else//온도가 높아 화재경보가 발생해야 하는 상황
		{
			int temp1 = 0;
			int temp2 = -1;
			OSQPost(MsgQueue, (void*)&temp1);
			OSQPost(MsgQueue, (void*)&temp2);
			//FndDisplayTask(-1);//화재경보가 발생 한 후에는 FND에 NO NO 표시
			alert = 1;
			_delay_ms(100);
		}

	}
}
void move(int which)// which : 어떤 엘레베이터를 움직일 것인지에 대한 선택. 
{
	int i;
	if (which == 1)//첫번째 엘레베이터를 움직이는 경우
	{
		while (now_location != elevate1)
		{
			if (elevate1 - now_location > 0)
			{
				elevate1--;
				FndDisplayTask(elevate1 * 100 + elevate2);
				_delay_ms(100);
			}
			else
			{
				elevate1++;
				FndDisplayTask(elevate1 * 100 + elevate2);
				_delay_ms(100);
			}
		}


	}
	else if (which == 2)//두 번째 엘레베이터를 움직이는 경우. 
	{
		while (now_location != elevate2)
		{
			if (elevate2 - now_location > 0)
			{
				elevate2--;
				FndDisplayTask(elevate1 * 100 + elevate2);
				_delay_ms(100);
			}
			else
			{
				elevate2++;
				FndDisplayTask(elevate1 * 100 + elevate2);
				_delay_ms(100);
			}
		}
	}


}
void LedTask()
{
	// LED Task
	int up = 1;
	int i, value;
	DDRA = 0xff;
	value = 0;
	value = 0;
	for (i = 0; i < now_location; i++)
	{
		value += num[i];
	}

	PORTA = value;
	_delay_ms(20);

}//사용자의 층 수를 LED로 표시한다.
void FndDisplayTask(void *data)
{
	int cnt;
	int i, fnd[4], temp;
	int locate1, locate2;//elevate1과 elevate2의 위치.
	volatile int count;
	INT8U err;
	while (1)
	{
		locate1 = *(int*)OSQPend(MsgQueue, 0, &err);
		locate2 = *(int*)OSQPend(MsgQueue, 0, &err);//elevate의 위치를 전달 받아서 FND에 출력하기
		count = locate1 * 100 + locate2;
		if (count == -1)
		{
			fnd[3] = 10;
			fnd[2] = 0;
			fnd[1] = 10;
			fnd[0] = 0;
		}//화재 경보시 NONO를출력한다
		else
		{
			fnd[3] = (count / 1000) % 10; // 천자리
			fnd[2] = (count / 100) % 10; // 백자리
			fnd[1] = (count / 10) % 10; // 십자리
			fnd[0] = count % 10; // 일자리
		}
		for (cnt = 0; cnt < 100; cnt++)
		{
			for (i = 0; i < 4; i++)
			{
				// 출력할 데이터
				if (i == 2)
				{
					temp = digit[fnd[i]] + 128; // fnd 2번은 dot을 찍어야하므로 0x80을 더해준다.
					PORTC = temp;
				}
				else
					PORTC = digit[fnd[i]];

				// 출력할 fnd 번호
				PORTG = fnd_sel[i];

				_delay_ms(2);
			}

		}
	}
	

}