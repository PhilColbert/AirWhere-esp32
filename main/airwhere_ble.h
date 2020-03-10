/*
 * airwhere_ble.h
 *
 *  Created on: 15 Nov 2017
 *      Author: Fugazi
 */


#ifndef MAIN_AIRWHERE_BLE_H_
#define MAIN_AIRWHERE_BLE_H_

#include <esp_coexist.h>
//#include <sstream>             // Part of C++ Standard library

extern bool deviceConnected;

BLECharacteristic *pCharacteristic;

const char *CHARACTERISTIC_UUID_DEVICENAME = "00002A00-0000-1000-8000-00805F9B34FB";
const char *CHARACTERISTIC_UUID_RXTX = "0000FFE1-0000-1000-8000-00805F9B34FB";
const char *CHARACTERISTIC_UUID_RXTX_DESCRIPTOR = "00002902-0000-1000-8000-00805F9B34FB";
const char *SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB";

class MyServerCallbacks : public BLEServerCallbacks {

	void onConnect(BLEServer* pServer) {
		if ( debug_level > 0 )
		{
	  	  Serial.println("***************************** BLE CONNECTED *****************");
	    }

		deviceConnected = true;
	};

	void onDisconnect(BLEServer* pServer) {
	if ( debug_level > 0 )
	{
      Serial.println("***************************** BLE DISCONNECTED *****************");
    }
		deviceConnected = false;
		delay(1000);
	//	pServer->
	}

};



class MyCallbacks : public BLECharacteristicCallbacks {

	void onWrite(BLECharacteristic *pCharacteristic) {

		std::string rxValue = pCharacteristic->getValue();

		if (rxValue.length() > 0) {

			Serial.println("*********");

			Serial.print("Received Value: ");

			for (int i = 0; i < rxValue.length(); i++)
				Serial.print(rxValue[i]);
			Serial.println();
			Serial.println("*********");

		}

	}

};

void BLESendChunks(String str)
{
	String substr;
//	Serial.print("ENTER>>>>");
	//		Serial.print(str);
	//	Serial.print("<<<<<<<");

	if (deviceConnected) {
		if ( debug_level > 0 )
		{
		  Serial.println("");
		  Serial.printf( "in BLESendChunks - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	    }
	//	Serial.println(">>>>>>>>>>  >>>>>>>>>>>>>>Device connected");
		for (int k = 0; k < str.length(); k += _min(str.length(), 20)) {
			substr = str.substring(k, k + _min(str.length() - k, 20));
			//Serial.println("in BLESendChunks 1a");

		///	Serial.println("");
			Serial.print("Sending>>>>");
		//	Serial.print(substr.c_str());
		//	Serial.println("<<<<");
			pCharacteristic->setValue(substr.c_str());
			//Serial.println("in BLESendChunks 1b");
			pCharacteristic->notify();
			vTaskDelay(5);
		}
		if ( debug_level > 0 )
		{
		  Serial.println("");
		  Serial.printf( "Out BLESendChunks - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	    }
	}
	else
	{
		Serial.println("<<<<<<<<<<<<<<<<<<<<<<Device DISconnected");
	}

	vTaskDelay(20);


}

void NEMEA_Checksum(String *sentence)
{

	Serial.println("in NEMEA_Checksum");

	char chksum[3];


	const char *n = (*sentence).c_str() + 1; // Plus one, skip '$'
	uint8_t chk = 0;

	/* While current char isn't '*' or sentence ending (newline) */
	while ('*' != *n && '\n' != *n) {

		chk ^= (uint8_t)*n;
		n++;
	}

	//convert chk to hexadecimal characters and add to sentence
	sprintf(chksum, "%02X\n", chk);
	(*sentence).concat(chksum);


}


void start_ble (String airwhereID)
{

	    esp_coex_preference_set(ESP_COEX_PREFER_BT);

	    String airwherebleID="AirWhere-"+airwhereID;

	//std::string airwhereIDble=airwhereID.c_str();

	    BLEDevice::init(airwherebleID.c_str());
	 //   const char* ssid     = airWhereSsid;

		// Create the BLE Server

		BLEServer *pServer = BLEDevice::createServer();
		pServer->setCallbacks(new MyServerCallbacks());

		// Create the BLE Service
	//	BLEService *pService = pServer->createService(SERVICE_UUID);
BLEService *pService = pServer->createService(BLEUUID((uint16_t)0xFFE0));
		// Create a BLE Characteristic
		pCharacteristic = pService->createCharacteristic(BLEUUID((uint16_t)0xFFE1),
			BLECharacteristic::PROPERTY_NOTIFY| BLECharacteristic::PROPERTY_WRITE| BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR
		);

		//pCharacteristic->addDescriptor(new BLEDescriptor(CHARACTERISTIC_UUID_RXTX_DESCRIPTOR));
		pCharacteristic->addDescriptor(new BLE2902());

    	BLECharacteristic *pCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_DEVICENAME,
		BLECharacteristic::PROPERTY_READ
		);

		pCharacteristic->setValue("esp32ble-hm10");
		pCharacteristic->setCallbacks(new MyCallbacks());
		Serial.println("Starting BLE ");
		// Start the service
		pService->start();
pServer->getAdvertising()->addServiceUUID(BLEUUID((uint16_t)0xFFE0));
		// Start advertising
		pServer->getAdvertising()->start();
		Serial.println("Waiting a client connection to notify...");
}

#endif /* MAIN_AIRWHERE_BLE_H_ */
