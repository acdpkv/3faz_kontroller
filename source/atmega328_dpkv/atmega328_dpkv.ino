const byte acceleratorPin = 0; // Входной аналоговый пин для регулировки частоты (0-7)
const byte damperPin = 1; // Входной аналоговый пин для регулировки скорости нарастания (0-7)
const byte amplitudePin = 2; // Входной аналоговый пин для добавления амплитуды (0-7)

const byte reversePin = 8; // Входной пин реверса, когда вход никуда не подключен или подключен к плюсу, то вперёд, когда вход подключен к минусу/GND, то реверс (2-13)

const byte allowOutPin = 13; // Логический выход (когда фазные выходы включены - на этом выходе 5В, когда нет - 0В)


// Аналоговые значения ниже: 0 - это 5В, 1023 - это 0В
const int minAccel = 850;	// Значение, меньше которого на выходах 0В; например, для 0.5В при питании 5В: 1023-0.5/5*1023 = 921
const int midAccel = 512;	// Значение, для выравнивания логарифмической зависимости регулировки частоты, используется вместе с задержкой midTime ниже, тем самым меняя среднее значение midAccel и/или среднюю задержку midTime можно регулировать зависимость напряжения от частоты; 2.5В - 512
const int maxAccel = 164;	// Значение, больше которого на выходах максимальная частота; например, для 4.2В при питании 5В: 1023-4.2/5*1023 = 164


// Задержка между переключениями частей синусоиды / частота (1-32767), рассчитывается по формуле: ЗАДЕРЖКА = 33333 / НЕОБХОДИМАЯ_ЧАСТОТА - 318(число из таблицы "Задержка (мкс)")
const int minTime = 10000;	// 10000 - это ~3.23Гц, чем меньше число, тем больше частота, и наоборот
const int midTime = 237;	// 237 - это ~60Гц, то есть при выбранных выше midAccel 2.5В, будет 60 Гц, остальные значения рассчитываются меджу ними автоматически
const int maxTime = 15;		// 15 - это ~100Гц, также чем меньше число, тем больше частота, и наоборот

const int maxTimeReverse = 15; // Максимальная частота заднего хода, 1015 - это ~25Гц


// Аналоговые значения ниже: 1023 - это 5В, 0 - это 0В
const int maxDamper = 512;	// Значение, меньше которого происходит повышение частоты, а больше - понижение; например 2В: 2/5*1023 = 410


const int delayReverse = 1000;	// Задержка между переключением направления, ~1000мс=1сек.

const int addAmplitude = 10; // Добавление амплитуды, чем больше число, тем меньше добавка; например 0.5В добавит при числе 10: 102/10=~10%; при 20: 102/20=~5% (1-1023)


const float startAmplitude = 10.0;	// Начальная амплитуда для рассчитанной частоты

const float incAmplitude = 0.08;	// Скорость нарастания амплитуды до соответствующей по частоте, чем больше число, тем быстрее нарастание (число нарастания в % на каждую часть синусоиды, тем самым если умножить число на 30, получится увеличение в % за один период, то есть 0.30 * 30 = 9% за один период)




/*
Номера пинов с фазами не изменяются:

Синусоида [1] фазы - пин 6
Синусоида [2] фазы - пин 9
Синусоида [3] фазы - пин 11

Энкодер - пин 2
*/






const int midAccelM1 = midAccel - 1;
const int midTimeM1 = midTime - 1;


const byte fp1[30] = {128,155,180,203,223,239,255,255,255,255,239,223,203,180,155,128,101,76,53,33,17,0,0,0,0,17,33,53,76,101};
const byte fp2[30] = {239,223,203,180,155,128,101,76,53,33,17,0,0,0,0,17,33,53,76,101,128,155,180,203,223,239,255,255,255,255};
const byte fp3[30] = {17,0,0,0,0,17,33,53,76,101,128,155,180,203,223,239,255,255,255,255,239,223,203,180,155,128,101,76,53,33};





byte j = 0;
bool enabled = false;
float dFreq = minTime;	// Начальная задержка между переключениями частей синусоиды, то есть число начальной частоты; когда скорость нарастания частоты не мгновенная, как числа выше 15-1400 (100Гц-15Гц), то есть если здесь 1400 - minTime, то с нажатой педалью газа в пол и включении питания в этот момент, нарастание будет начинаться с 15 Гц, а если поставить 15, то сразу будет 100 Гц
bool prevDirection = true;
int microsecond = minTime;


uint8_t analog_ref = DEFAULT;
int acceleratorValue = 0;
int damperValue = 1023;
int amplitudeValue = 0;
byte aPin = damperPin;


float damperAmplitude = startAmplitude;
bool damperBoost = true;


volatile unsigned int pulses;

void counter()
{
	pulses++;
}


void setup()
{
	pinMode(acceleratorPin + 14, INPUT);
	pinMode(damperPin + 14, INPUT);
	pinMode(amplitudePin + 14, INPUT);
	
	pinMode(allowOutPin, OUTPUT);
	digitalWrite(allowOutPin, LOW);
	
	ADCSRA = 0;
	ADCSRB = 0;
	ADMUX |= (1 << REFS0);
	analog_ref = ADMUX;
	ADMUX |= aPin;
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADSC);

	TCCR0A = (TCCR0A & 0xe0) | 1;
	TCCR0B = (TCCR0B & 0xf8) | 2;
	TCCR1B = (TCCR1B & 0xf8) | 2;
	TCCR2B = (TCCR2B & 0xf8) | 2;
	TIMSK0 = 0;
	TIMSK1 = 0;
	TIMSK2 = 0;

	pinMode(6, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(11, OUTPUT);

	pinMode(reversePin, INPUT_PULLUP);

	prevDirection = digitalRead(reversePin);
	
	pinMode(2, INPUT);
}

void loop()
{
	if (enabled)
	{
		int rFreq = 33333/(dFreq+318);	// 318 - число из таблицы "Задержка (мкс)"
		
		int ampFactor = 0;
		
		
		//------------------------------
		
		if (rFreq > 60)	// Частота больше 60 Гц, значит 60-100 Гц = 40-90%
			ampFactor = mapi(rFreq,   60, 100   ,   40, 90   );
		else if (rFreq > 12)	// Частота больше 12 Гц, значит 12-60 Гц = 18-40%
			ampFactor = mapi(rFreq,   12, 60   ,   18, 40   );
		else	// Частота менее 12 Гц, значит 3-12 Гц = 10-18%
			ampFactor = mapi(rFreq,   3, 12   ,   10, 18   );
		
		//------------------------------
		
		
		ampFactor += amplitudeValue/addAmplitude;
		
		
		if(damperBoost)
		{
			if(damperAmplitude < ampFactor)
			{
				ampFactor = damperAmplitude;
				
				damperAmplitude += incAmplitude;
			}
			else
			{
				damperBoost = false;
			}
		}
		
		
		if(ampFactor > 100)
		{
			ampFactor = 100;
		}
		else if(ampFactor < 1)
		{
			ampFactor = 1;
		}
		
		float amplitudeFactor = ampFactor/100.0;
		
		byte _OCR2A = (((int)fp1[j]-128)*amplitudeFactor)+128;
		byte _OCR1A = (((int)fp2[j]-128)*amplitudeFactor)+128;
		byte _OCR0A = (((int)fp3[j]-128)*amplitudeFactor)+128;
		
		OCR2A = _OCR2A;
		OCR1A = _OCR1A;
		OCR0A = _OCR0A;
		
		TCCR2A |= (1 << COM2A1);
		TCCR1A |= (1 << COM1A1);
		TCCR0A |= (1 << COM0A1);
		
		digitalWrite(allowOutPin, HIGH);
	}
	else
	{
		digitalWrite(allowOutPin, LOW);
		
		digitalWrite(6, LOW);
		digitalWrite(9, LOW);
		digitalWrite(11, LOW);
		
		if (aPin == damperPin)
		{
			pulses=0;
			
			attachInterrupt(0, counter, CHANGE);
			
			for (unsigned int l = 0; l < 125; l++)
			{
				delayMicroseconds(1000);
			}
			
			detachInterrupt(0);
			
			
			if(pulses == 0)
			{
				dFreq = minTime;
			}
			else
			{
				dFreq = 33333/(pulses*8.0/6.0)-318;
				
				if(dFreq > minTime)
					dFreq = minTime;
				else if(dFreq < maxTime)
					dFreq = maxTime;
			}
		}
	}
	
	
	bool nowDirection = digitalRead(reversePin);

	if (nowDirection != prevDirection)
	{
		digitalWrite(allowOutPin, LOW);
		
		digitalWrite(6, LOW);
		digitalWrite(9, LOW);
		digitalWrite(11, LOW);
		
		for (unsigned int l = 0; l < delayReverse; l++)
		{
			delayMicroseconds(1000);
		}
		
		acceleratorValue = 0;
		enabled = false;
		
		if (!bit_is_set(ADCSRA, ADSC))
		{
			ADCSRA |= (1 << ADSC);
		}
		
		dFreq = minTime;
		
		prevDirection = nowDirection;
	}

	
	if (prevDirection)
	{
		if(j == 29)
		{
			j = 0;
		}
		else
		{
			j++;
		}
	}
	else
	{
		if(j == 0)
		{
			j = 29;
		}
		else
		{
			j--;
		}
	}



	if (!bit_is_set(ADCSRA, ADSC))
	{
		int result = ADCL | (ADCH << 8);

		if (aPin == damperPin)
		{
			damperValue = result;
			aPin = acceleratorPin;
		}
		else if (aPin == acceleratorPin)
		{
			acceleratorValue = result;
			aPin = amplitudePin;
		}
		else
		{
			amplitudeValue = result;
			aPin = damperPin;
		}

		ADMUX = analog_ref | aPin;
		ADCSRA |= (1 << ADSC);
	}


	int analog = 1023 - acceleratorValue;

	if (analog > minAccel)
	{
		if(enabled)
		{
			damperAmplitude = startAmplitude;
			damperBoost = true;
		}
		
		enabled = false;
	}
	else
	{
		enabled = true;

		if (analog < maxAccel)
			analog = maxAccel;



		// Логарифмическая зависимость частоты от аналогового значения, то есть чем больше напряжение, тем быстрее рост частоты (сейчас закомментирована и используется более линейная ниже), при 2.5В было ~29Гц
		//microsecond = map(analog, maxAccel, minAccel, maxTime, minTime);
		
		
		// Сглаживание логарифмической зависимости в линейную
		if (analog < midAccel)
			microsecond = map(analog, maxAccel, midAccelM1, maxTime, midTimeM1);
		else
			microsecond = map(analog, midAccel, minAccel, midTime, minTime);
		
		
		if(!prevDirection)
		{
			if (microsecond < maxTimeReverse)
			{
				microsecond = maxTimeReverse;
			}
		}
		
		
		float dFreq2 = sq(dFreq);
		float dFreq3 = dFreq2 * dFreq;
		
		
		
		if (microsecond > dFreq)
		{
			dFreq += 2E-10 * dFreq3 + 2E-07 * dFreq2 + 6E-05 * dFreq + 0.0069;	// 7 Гц - Формула понижения частоты при отпускании педали газа
			
			
			if (dFreq > microsecond)
			{
				dFreq = microsecond;
			}
		}
		else if (damperValue < maxDamper)
		{
			dFreq -= 6E-10 * dFreq3 + 6E-07 * dFreq2 + 0.0002 * dFreq + 0.0208;	// 20 Гц - Формула нарастания частоты при нажатии на педаль газа
			
			
			if (dFreq < microsecond)
			{
				dFreq = microsecond;
			}
		}
		else
		{
			dFreq += 3E-10 * dFreq3 + 3E-07 * dFreq2 + 9E-05 * dFreq + 0.01;	// 10 Гц - Формула понижения частоты при превышении напряжения maxDamper
			
			
			if (dFreq > minTime)
			{
				dFreq = minTime;
			}
		}
		

		delayMicroseconds(dFreq);
	}
}


int mapi(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
