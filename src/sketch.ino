#include <Arduino.h>
#include <SPI.h>
#include <EtherCard.h>
#include <stdlib.h>
#include <stdarg.h>

// ------------------------------------------------------
// ENABLE/DISABLE FUNCTIONS
// ------------------------------------------------------
//#define DEBUG
//#define TEMPERATURE
//#define WATCHDOG

#ifdef TEMPERATURE
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#ifdef WATCHDOG
#include <avr/wdt.h>
#endif

// ------------------------------------------------------
// Network Configuration
// ------------------------------------------------------
byte 			mac[] = 		{ 0xDE, 0xAD, 0x12, 0x12, 0x12, 0x12 };
byte 			ip[] = 			{ 192, 168, 2, 12 };
byte 			gateway[] = 		{ 192, 168, 2, 1 };
byte 			subnet[] = 		{ 255, 255, 255, 0 };

int 			pos;
#define 		kBUFFER_SIZE 		500
byte 			Ethernet::buffer[kBUFFER_SIZE];
static BufferFiller	bfill;

#define 		kBUFFER_SIZE_AUX	400
char			aux[kBUFFER_SIZE_AUX];


// ------------------------------------------------------
// Energy Comsumption Control
// ------------------------------------------------------
float			average = 		0.0;
int			num_samples=		100;
#define 		k0_AMPS_VALUE 		(1023/2)   //ACS712 value to 0 amps
#define			kPIN_SENSOR 		A0	   //Current sensor pin
int			power_usage = 		0;	   //Power usage


// ------------------------------------------------------
// Control de reles
// ------------------------------------------------------
#define 		kN_DEVICES		3	   //Number of devices we want to control
#define			kOFF			1	   //Relay board states: L=ON, H=OFF
#define			kON			0
#define			kMAX_WORKING_HOURS	5	   //Max working time in hours for any device
unsigned long		device_power_on_time[kN_DEVICES]= {0,0,0};
int			device_pin[kN_DEVICES]		= {7,1,2};            // Relay pins
int			device_state[kN_DEVICES]	= {kOFF,kOFF,kOFF};   // Default device states


// ------------------------------------------------------
// Temperatura
// ------------------------------------------------------
#ifdef TEMPERATURE
#define 		PIN_ONEWIRE 9
OneWire 		oneWire(PIN_ONEWIRE);
DallasTemperature 	temp(&oneWire);
DeviceAddress 		temp_sensor_address = {0x28, 0xA6, 0x0A, 0xEA, 0x03, 0x00, 0x00, 0xFB};
float			last_correct_temperature = 20.0;
#endif




// ------------------------------------------------------
// INITIALIZING
// ------------------------------------------------------
void setup()
{
#ifdef WATCHDOG
  wdt_disable();
#endif

  power_usage=0;
  pinMode(kPIN_SENSOR,INPUT);

  for(int i=0;i<kN_DEVICES;i++)
  {
     device_power_on_time[i]=0;
     device_state[i]=kOFF;
     pinMode(device_pin[i],OUTPUT);
     digitalWrite(device_pin[i],device_state[i]);
  }

  pos=0;
  memset(aux,0,kBUFFER_SIZE_AUX/sizeof(char));

#ifdef TEMPERATURE
  temp.begin();
  temp.setResolution(temp_sensor_address,12);
#endif

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Initializing jEV System");
  Serial.print(" MAC: "); Serial.print(mac[0],HEX); Serial.print(mac[1],HEX); Serial.print(mac[2],HEX); Serial.print(mac[3],HEX); Serial.print(mac[4],HEX); Serial.println(mac[5],HEX);
  Serial.print(" IP: "); Serial.print(ip[0]); Serial.print("."); Serial.print(ip[1]); Serial.print("."); Serial.print(ip[2]); Serial.print("."); Serial.println(ip[3]);
  Serial.print(" GW: "); Serial.print(gateway[0]); Serial.print("."); Serial.print(gateway[1]); Serial.print("."); Serial.print(gateway[2]); Serial.print("."); Serial.println(gateway[3]);
  Serial.print(" MASK: "); Serial.print(subnet[0]); Serial.print("."); Serial.print(subnet[1]); Serial.print("."); Serial.print(subnet[2]); Serial.print("."); Serial.println(subnet[3]);
#endif

  if (!ether.begin(sizeof Ethernet::buffer, mac, 10))
  {
    debug(" * Failed to access Ethernet controller");
    while(1);
  }
  debug(" * Ethernet controller initialized");

  if (!ether.staticSetup(ip))
  {
#ifdef DEBUG
    Serial.println(" * Failed to set IP address");
    ether.printIp("   IP: ",ether.myip);
    ether.printIp("   GW: ",ether.gwip);
    ether.printIp("   DNS: ",ether.dnsip);
#endif
  }

#ifdef WATCHDOG
  wdt_enable(WDTO_8S);
#endif
}



// ------------------------------------------------------
// MAIN PROGRAM
// ------------------------------------------------------
void loop()
{
  bool encendidas=false;

#ifdef WATCHDOG
  wdt_reset();
#endif

  // A) Check for HTTP received command
  word len = ether.packetReceive();
  word pos = ether.packetLoop(len);
  if(pos)
  {
    bfill = ether.tcpOffset();
    char* data = (char *) Ethernet::buffer + pos;

    process_client_request(data);
    return_status_to_client();
  }

  // B) Check max working time for each device (1 h = 3600000 ms)
  for(int i=0;i<kN_DEVICES;i++)
  {
     if(device_power_on_time[i]!=0 && abs(millis()-device_power_on_time[i])>=kMAX_WORKING_HOURS*3600000)
       power_off_device(i);
  }
}


void process_client_request(char* data)
{
  bool salir=false;

  if(data==NULL)
     return;

  int size=strlen(data);
  for(int i=0,j=0;!salir && j<size;j++)
  {
     if(j<size-1 && data[j]=='\r' && data[j+1]=='\n')
     {
       salir=true;
       data[j]=0;

       if(strstr(data,"soft_reset")!=NULL)
         soft_reset();

       if(strstr(data,"ev1_on")!=NULL)
         power_on_device(0);

       if(strstr(data,"ev1_off")!=NULL)
         power_off_device(0);

       if(strstr(data,"ev2_on")!=NULL)
         power_on_device(1);

       if(strstr(data,"ev2_off")!=NULL)
         power_off_device(1);

       if(strstr(data,"ev3_on")!=NULL)
         power_on_device(2);

       if(strstr(data,"ev3_off")!=NULL)
         power_off_device(2);

       if(strstr(data,"all_on")!=NULL)
         for(int i=0;i<kN_DEVICES;i++)
           power_on_device(i);

       if(strstr(data,"all_off")!=NULL)
         for(int i=0;i<kN_DEVICES;i++)
           power_off_device(i);
     }
  }
}


void return_status_to_client()
{
  double power,real_power,current,watts;
  double atemp;
  long vcc;
  char a[7][20];
  unsigned long t=millis();

  // 1a. Calculamos las tres medias de consumo
  average = 0;
  for(int i = 0; i < num_samples; i++)
  {
    average = average + analogRead(kPIN_SENSOR);
    delay(2);
  }

  power=average/num_samples;
  real_power=abs(k0_AMPS_VALUE-power);
  current=abs(0.0264*(power-k0_AMPS_VALUE));
  watts=abs(0.0264*(power-k0_AMPS_VALUE)*230);
  atemp=readInternalTemp();
  vcc=readVcc();

  memset(aux, 0, kBUFFER_SIZE_AUX);
  snprintf(aux,kBUFFER_SIZE_AUX,"HTTP/1.1 200 OK\r\n"
                      "Content-Type: text/html\r\n"
//#ifdef DEBUG
                      "Refresh: 15\r\n\r\n"
//#endif
                      "<html>\r\n"
                      "powerADC:%s\r\n<br/>\r\n"
                      "power:%s\r\n<br/>\r\n"
                      "current:%s\r\n<br/>\r\n"
                      "watts:%s\r\n<br/>\r\n"
                      "atemp:%s\r\n<br/>\r\n"
                      "avcc:%d\r\n<br/>\r\n"
                      "millis:%lu\r\b<br/>\r\n"
#ifdef TEMPERATURE
                      "tempDS:%s\r\n<br/>\r\n"
#endif
                      ,
           dtostrf(power,2,3,a[0]),dtostrf(real_power,2,3,a[1]),dtostrf(current,2,3,a[2]),dtostrf(watts,2,3,a[3]),
           dtostrf(atemp,2,3,a[5]),(int)vcc,t
#ifdef TEMPERATURE
           ,dtostrf(readTempSensor(temp_sensor_address),2,3,a[4])
#endif
  );


  for(int i=0;i<kN_DEVICES;i++)
  {
    snprintf(aux,kBUFFER_SIZE_AUX,"%s"
                      "estado_dispositivo%d:%d\r\n<br/>\r\n",
             aux,i+1,device_state[i]);
  }
  snprintf(aux,kBUFFER_SIZE_AUX,"%s</html>",aux);

  bfill.emit_raw(aux,strlen(aux));
  ether.httpServerReply(bfill.position());
}



// ------------------------------------------------------
// POWER OFF SPECIFIED DEVICE
// ------------------------------------------------------
void power_off_device(int device_number)
{
   if(device_number>=0 && device_number<kN_DEVICES)
   {
     device_state[device_number]=kOFF;
     digitalWrite(device_pin[device_number],kOFF);
     device_power_on_time[device_number]=0;

     debug2("Powering off device: ",device_number);
   }
}


// ------------------------------------------------------
// POWER ON SPECIFIED DEVICE
// ------------------------------------------------------
void power_on_device(int device_number)
{
   if(device_number>=0 && device_number<kN_DEVICES)
   {
     device_state[device_number]=kON;
     digitalWrite(device_pin[device_number],kON);
     device_power_on_time[device_number]=millis();

     debug2("Powering on device: ",device_number);
   }
}



// ------------------------------------------------------
// Returns ATmega328 temperature (ºmC)
// ------------------------------------------------------
double readInternalTemp()
{
  unsigned int wADC;
  double t;

  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  ADCSRA |= _BV(ADEN);
  delay(20); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  wADC = ADCW;

  t = (wADC - 324.31 ) / 1.22;

  return t;
}


// ------------------------------------------------------
// Returns ATmega328 voltage (mV)
// ------------------------------------------------------
long readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}



// ------------------------------------------------------
// Returns DS18B20 sensor temperature
// ------------------------------------------------------
#ifdef TEMPERATURE
double readTempSensor(DeviceAddress sensor_address)
{
  int i=10;  //ten samples
  int j=0;
  float value=-127;

  while((value<=-55 || value>=85) && i>0)
  {
     i--; 

     cli();
     temp.requestTemperatures();
     value=temp.getTempC(sensor_address)/1;
     sei();
     delay(30);
  }

  if(value==-127 || value==85)
  {
     value=last_correct_temperature;
  }
  last_correct_temperature=value;

  return((double)value);
}
#endif


// ------------------------------------------------------
// Show debug message on serial line
// ------------------------------------------------------
void debug(char* message)
{
#ifdef DEBUG
  if(message!=NULL && message[0]!=0)
     Serial.println(message);
#endif
}

void debug2(char* message, int value)
{
#ifdef DEBUG
  if(message!=NULL && message[0]!=0)
  {
     Serial.print(message);
     Serial.println(value);
  }
#endif
}


// ------------------------------------------------------
// Restarts program from beginning but does not reset
// the peripherals and registers
// ------------------------------------------------------
void soft_reset()
{
/*
#ifdef WATCHDOG
  //wdt_enable(WDTO_15MS);
  while(1) { };
#endif
*/
  asm volatile ("  jmp 0");
}
