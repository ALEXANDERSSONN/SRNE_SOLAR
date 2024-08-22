#include <Arduino.h>
#include "wiring_private.h"
#include <ModbusMaster.h>

Uart LTEserial(&sercom3, 7, 6, SERCOM_RX_PAD_3, UART_TX_PAD_2);  // Create the new UART instance assigning it to pin 7 and 6
Uart modbus_iso(&sercom0, 3, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);

#define MAX485_DE 4
#define WORK_DE 5

ModbusMaster node;

uint32_t last1, last2, last3, last4;
uint8_t result, qty, i;
uint16_t start_register;
bool incoming = false;
bool first_config = true;
String message, chk, str;
String rssi;
float temp, humid;

/*----- CONFIG NODE ID HERE ---------*/

String nid = "0001";  //01

/*----- CONFIG NODE ID HERE ---------*/

void setup() {

  Serial.begin(9600);

  LTEserial.begin(115200);
  pinPeripheral(7, PIO_SERCOM_ALT);  //Assign RX function to pin 7
  pinPeripheral(6, PIO_SERCOM_ALT);  //Assign TX function to pin 6

  modbus_iso.begin(9600);
  pinPeripheral(3, PIO_SERCOM);
  pinPeripheral(2, PIO_SERCOM);

  pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);
  pinMode(17, OUTPUT);
  //digitalWrite(17, HIGH);

  //QUECTEL_PCIE_TX
  pinMode(A1, OUTPUT);     //SIM7600_ENA TX Pin (U9)
  digitalWrite(A1, LOW);  //SIM7600_ENA set Active High

  //QUECTEL_PCIE_RX
  pinMode(A5, OUTPUT);    //QUECTEL/SIM7600_ENA RX pin (U10 PIN19)
  digitalWrite(A5, LOW);  //QUECTEL/SIM7600_ENA set Active Low

  //Disable SIM7600,Neoway
  pinMode(A6, OUTPUT);     //NERO_ENA U10 PIN 1
  digitalWrite(A6, HIGH);  //NERO_ENA Active Low

  //During uplink
  pinMode(WORK_DE, OUTPUT);
  digitalWrite(WORK_DE, LOW);

  //Hard reset EC25
  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);

  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {

  //Uplink payload
  if (millis() - last1 >= 10000) {
    last1 = millis();
    digitalWrite(WORK_DE, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);


    /*PUBLIC TOPIC*/

    //WELLPRO 8xAnalog modbus id=1
    node.begin(1, modbus_iso);
    /* ------------------------------------------------------------------- */
    //lot1 - analog 4-20mA 20 bytes
    start_register = 257;
    qty = 10;
    str = "";
    str = nid;
    str += "01";
    str += GET_RSSI();
    result = node.readInputRegisters(start_register, qty);
    Serial.print("Result = ");
    Serial.println(result);
    delay(1500);
    if (result == node.ku8MBSuccess) {
      for (i = 0; i < qty; i++) {
        char buf[5];
        sprintf(buf, "%04x", node.getResponseBuffer(i));
        str += String(buf);
        Serial.println(str);
        
      }
    }
    CMD("AT+NETOPEN");
    CMD("AT+CIPOPEN=1,\"UDP\",,,9958");
    CMD("AT+CIPSEND=1,5,\"188.166.178.255\",9958");
    CMD(str);

    CMD("AT+CIPCLOSE=1");
    CMD("AT+NETCLOSE");
    /* ------------------------------------------------------------------- */

    digitalWrite(WORK_DE, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  }  //if milli()

  //check connect status every 1minute
  if (millis() - last2 >= 60000) {
    last2 = millis();

    if (digitalRead(WORK_DE) == LOW) {

      chk = CMD("AT+CREG?");
      if (chk.indexOf("+CREG: 0,5") > 0) {
        Serial.println("Network connected.");
      } else {
        CMD("AT+CFUN=0");
        delay(1000);
        CMD("AT+CFUN=1");
      }
    }
  }  //milli() last2

  //hard-reset EC25 module every month
  if (millis() - last3 >= 2629800000) {
    last3 = millis();

    digitalWrite(A3, HIGH);
    delay(1000);
    digitalWrite(A3, LOW);
  }  //milli() last3

  //check ping network every 1hr
  if (millis() - last4 >= 3600000) {
    last4 = millis();

    if (digitalRead(WORK_DE) == LOW) {

      chk = CMD("AT+QPING=1,\"8.8.8.8\",5,3");
      if (chk.indexOf("8.8.8.8") > 0) {
        Serial.println("Ping OK.");
      } else {
        CMD("AT+CFUN=0");
        delay(1000);
        CMD("AT+CFUN=1");
      }
    }
  }  //milli() last2
}

//-
String CMD(String at) {
  String txt = "";
  LTEserial.println(at);
  delay(100);
  while (LTEserial.available()) {
    txt += (char)LTEserial.read();
  }
  Serial.println(txt);
  return txt;
}

String GET_RSSI() {
  String msg = CMD("AT+CSQ");
  String r = msg.substring(msg.indexOf(" "), msg.indexOf(","));

  int rv = (r.toInt() * 2) - 113;
  if (rv == -113 || rv == 85) {
    rv = -113;
  }

  //Serial.println(rv);
  int myrssi = rv * -1;

  char rbuf[3];
  sprintf(rbuf, "%02x", myrssi);

  return String(rbuf);
}

String GET_CCID() {
  String msg = CMD("AT+QCCID");
  String r = msg.substring(msg.indexOf(" ") + 1, 39);
  return r;
}

void preTransmission() {
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_DE, 0);
}

void SERCOM0_Handler() {
  modbus_iso.IrqHandler();
}

void SERCOM3_Handler() {
  LTEserial.IrqHandler();
}
