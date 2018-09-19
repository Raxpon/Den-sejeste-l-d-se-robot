// pragma bruges til at g�re vores kode mere l�sbar og struktureret. Det g�r
// s� man kan kollapse koden i Visual Studio. N�r koden bliver compiled ser compileren
// bort fra pragma ligesom den g�r ved kommentarer
#pragma region Libraries, Objects and ENUMs

// bibliotek
#include "MeMegaPi.h"

// Motorer UDEN encoder (bruges til manuel styring)
MeMegaPiDCMotor mL(PORT1B);     // DC-motor til venstre larvefodsmotor
MeMegaPiDCMotor mR(PORT2B);     // DC-motor til h�jre larvefodsmotor
MeMegaPiDCMotor mArm(PORT3B);   // DC-motor til arm
MeMegaPiDCMotor mClamp(PORT4B); // DC-motor til klo

// Motorer MED encoder (bruges til automatisk program)
MeEncoderOnBoard EmL(SLOT_1);   // Encodermotor til venstre larvefodsmotor
MeEncoderOnBoard EmR(SLOT_2);   // Encodermotor til h�jre larvefodsmotor
MeEncoderOnBoard EmArm(SLOT_3); // Encodermotor til arm

// De andre sensorer p� robotten TODO: ANGIV PORTE
MeUltrasonicSensor USSensor;    // Ultralydssensor
MeGyro Gyro;                    // Gyroskop og accelerometer
MeLineFollower LFollower;       // Line follower

// ENUM s� det er nemmere at l�se koden vi bruger til automatisk program
enum AUTOSTATE { OPGAVE_1, OPGAVE_2, OPGAVE_3, OPGAVE_4, OPGAVE_5, OPGAVE_6, NOTHING };

AUTOSTATE aState; // vores enum bruges til automatisk program
int timer = 0;
int drivenLenght = 0;

#pragma endregion
void isr_process_encoder1(void)
{
	if (digitalRead(EmR.getPortB()) == 0)
	{
		EmR.pulsePosMinus();
	}
	else
	{
		EmR.pulsePosPlus();;
	}
}

void isr_process_encoder2(void)
{
	if (digitalRead(EmL.getPortB()) == 0)
	{
		EmL.pulsePosMinus();
	}
	else
	{
		EmL.pulsePosPlus();
	}
}
void isr_process_encoder3(void)
{
	if (digitalRead(EmArm.getPortB()) == 0)
	{
		EmArm.pulsePosMinus();
	}
	else
	{
		EmArm.pulsePosPlus();
	}
}
#pragma region Classes and Functions

// siger vi har en function som er uddybbet l�ngere nede
void AutomaticMode();
class DataTransfer;

// class, som har funktioner til robottens sensorer: ultralydssensor, gyro, linefollower
class Sensor 
{
private:
public:
};

Sensor sensor; // bruges til at tilg� funktioner vedr�rende �vrige sensorer p� robotten

// class, som har funktionerne, der bruges, n�r robotten er i manuel tilstand
// dvs. DC-motorene UDEN encoder
class ManualControl 
{
private:
    uint8_t vent = 200; // bruges n�r der bruges delay, dvs. vent 200 milisekunder
    uint8_t fart = 150; // angiver hastighed til kloen og armen.

    // funktion til at stoppe motorene
    void Stop() 
	{
        mL.stop();
        mR.stop();
        mArm.stop();
        mClamp.stop();
    }
public:
    // funktion til at styre larvef�ddernes hastighed
    // mLSpeed = venstre motor hastighed
    // mRSpeed = h�jre motor hastighed
    void Drive(byte mLSpeed, byte mRSpeed) 
	{

        // konverterer fra unsigned int (byte) til signed int
        int8_t MLSpeed = mLSpeed - 127;
        int8_t MRSpeed = mRSpeed + 127;

        // fordi robotten ikke skal k�re motoren hvis den f�r data fra appen som er t�t p� 0.
        // L�gger 150 til eller fra, fordi det er minimum hastighed for at robotten
        // overhovedet vil bev�ge sig. MLSpeed og MRSpeed kan h�jest v�re p� 80 eller -80 pga.
        // appens opbygning.
        if (MLSpeed < -25) 
		{
            mL.run(MLSpeed - 150);
        }
        else if (MLSpeed > 25) 
		{
            mL.run(MLSpeed + 150);
        }

        if (MRSpeed < -25) 
		{
            mR.run(MRSpeed + 150);
        }
        else if (MRSpeed > 25) 
		{
            mR.run(MRSpeed - 150);
        }
        //DataTransfer::TransmitBuffer[0] = 4;
        //DataTransfer::TransmitBuffer[1] = MLSpeed;
        //DataTransfer::TransmitBuffer[2] = MRSpeed;
        //DataTransfer::Send(3);

        delay(vent);
        Stop();
    }

    // funktion til at styre armens hastighed
    // saenkArm = hvis den er lig 0, s� skal armen h�ves, hvis den er lig 1 skal armen s�nkes
    void Swing(byte saenkArm) 
	{
        if (saenkArm == 0) 
		{
            mArm.run(fart);
        }
        else if (saenkArm == 1) 
		{
            mArm.run(-fart);
        }
        delay(vent);
        Stop();
    }

    // funktion til at styre kloens hastighed
    // lukClamp = hvis den er lig 0, s� skal kloen �bnes, hvis den er lig 1 skal kloen lukkes
    void Squash(byte lukClamp) 
	{
        if (lukClamp == 0) 
		{
            mClamp.run(fart);
        }
        else if (lukClamp == 1) 
		{

            mClamp.run(-fart);
        }
        delay(vent);
        Stop();
    }
};

ManualControl manuel; // class med k�refunktioner til larvef�dderne, armen og kloen UDEN encoder

// class, som har funktioner vedr�rende dataoverf�rsler igennem Bluetooth
class DataTransfer 
{
private:
    byte ReceiveDataBuffer[16]; // modtage data fra Appen

    // bruges til at "l�se" data fra appen og g�re bestemte ting
    // nofData = antallet af fundne data
    void Decode(byte nofData) 
	{
        for (uint8_t i = 0; i < nofData; i++) 
		{
            //TransmitBuffer[i] = ReceiveDataBuffer[i];

            switch (ReceiveDataBuffer[i])
            {
            case 0:
                manuel.Drive(ReceiveDataBuffer[++i], ReceiveDataBuffer[2]);
                break;
            case 1:
                manuel.Squash(ReceiveDataBuffer[++i]);
                break;
            case 2:
                manuel.Swing(ReceiveDataBuffer[++i]);
                break;
            case  3:
                AutomaticMode();
                break;
            default:
                break;
            }
        }
        //Send(nofData);
    }
public:
    byte static TransmitBuffer[16];    // sende data til Appen

    // funktion der konstant bliver k�rt og har mulighed for at modtage data fra appen
    void isReceived() 
	{
        //byte readData = 0; // l�ser den nuv�rende data
        byte nof_data = 0; // antallet af fundne data
        uint8_t cnt = 0; // looper igennem hver byte modtaget

        if (Serial3.available() != (int)0)
        {
            Serial.println("Data recieved");

            while (Serial3.available() != (int)0)
            {
                Serial.println(Serial3.available());

                //readData = Serial3.read();
                //ReceiveDataBuffer[cnt] = readData;
                ReceiveDataBuffer[cnt] = Serial3.read();   // hvorfor g�r man ikke det her?

                cnt++;
                nof_data++;

                delay(1);
            }

            Decode(nof_data);
        }
    }

    // bruges til at sende data til appen
    // nofData = antallet af data
    //void static Send(int nofData) {
    //    for (int i = 0; i < nofData; i++) {
    //        Serial3.write(TransmitBuffer[i]);
    //    }
    //    delay(1);
    //}
};
DataTransfer data;      // class med dataoverf�rselsfunktioner

//class, som styrer decodermotorer p� larvef�dder
class eFremdrift 
{
	private:
	public:
		void drive(float lenght, int motorspeed)
		{
			float tid = 45 * lenght * 60 / (6.3 * PI * motorspeed);
			EmL.moveTo(-2.2 * 360 * (lenght + drivenLenght) / (6.3 * PI), motorspeed);
			EmR.moveTo(2.2 * 360 * (lenght + drivenLenght) / (6.3 * PI), motorspeed);

			for (timer = 0; timer < tid; timer++) 
			{
				Serial.println(timer);
				Serial.println(drivenLenght);
				EmR.loop();
				EmL.loop();
				delay(100);
			}
			timer = 0;
			drivenLenght = lenght + drivenLenght;

		}
};

eFremdrift fremdrift;   // class med funktioner til encodermotor p� larvef�dder

// class med funktioner omhandlende decodermotor til armen
class eArm 
{
    // i denne class skal der v�re funktioner til armen (obviously lol)
private:
public:
	void drivearm(float lenght, int motorspeed)
	{
		EmArm.moveTo(-2.2 * 360 * lenght / (6.3*PI), motorspeed);
	}
};

eArm arm;               // class med funktioner til encoderarmen

void AutomaticMode() 
{
    while (true) 
	{
        switch (aState) 
		{
        case OPGAVE_1:
			 if (timer != 0) 
			{
				timer = 0;
			}
			for(timer= 0;timer < 40;timer++) 
			{
				EmR.loop();
				EmL.loop();
				eFremdrift.drive(43, 130);
				delay(100);

			}
			
            aState = OPGAVE_2;
            break;
        case OPGAVE_2:

            aState = OPGAVE_3;
            break;
        case OPGAVE_3:

            aState = OPGAVE_4;
            break;
        case OPGAVE_4:

            aState = OPGAVE_5;
            break;
        case OPGAVE_5:

            aState = OPGAVE_6;
            break;
        case OPGAVE_6:

            aState = NOTHING;
            break;
        }
        if (aState == NOTHING) 
		{
            aState = OPGAVE_1;
            break;
        }
        if (Serial3.read() == 3)
            break;
    }
}

#pragma endregion

#pragma region Setup()- and Loop()-functions

// The setup() function runs once each time the micro-controller starts
void setup() 
{
	attachInterrupt(EmR.getIntNum(), isr_process_encoder1, RISING);
	attachInterrupt(EmL.getIntNum(), isr_process_encoder2, RISING);
	attachInterrupt(EmArm.getIntNum(), isr_process_encoder3, RISING);
	Serial.begin(115200);

	TCCR1A = _BV(WGM10);
	TCCR1B = _BV(CS11) | _BV(WGM12);

	TCCR2A = _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS21);

	EmR.setPulse(7);
	EmL.setPulse(7);
	EmArm.setPulse(7);
	EmR.setRatio(26.9);
	EmL.setRatio(26.9);
	EmArm.setRatio(26.9);
	EmR.setPosPid(1.8, 0, 1.2);
	EmL.setPosPid(1.8, 0, 1.2);
	EmArm.setPosPid(1.8, 0, 1.2);
	EmR.setSpeedPid(0.18, 0, 0);
	EmL.setSpeedPid(0.18, 0, 0);
	EmArm.setSpeedPid(0.18, 0, 0);

    aState = OPGAVE_1;
    Serial.begin(115200);
    Serial3.begin(115200);
    //Serial3.write(0);
}

// Add the main program code into the continuous loop() function
void loop() {
	EmR.loop();
	EmL.loop();
	EmArm.loop();
	data.isReceived();
    delay(100);

}
