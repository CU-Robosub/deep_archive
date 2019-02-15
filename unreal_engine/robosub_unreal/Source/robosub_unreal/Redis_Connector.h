// Created by Cody Constine (cody.constine@colorado.edu)

#pragma once

#include "CoreMinimal.h"
#include "Networking.h"

/**
 * 
 */
class ROBOSUB_UNREAL_API Redis_Connector
{
public:
	Redis_Connector();
	~Redis_Connector();
	double get_key(FString key);
	int set_key(FString key, double set);
	int set_key_string(FString key, FString set);
private:
	bool connect();
	bool connected = false;
	int parseReturnString(char * s);
	FSocket* ConnectionSocket;
	FIPv4Endpoint RemoteAddressForConnection;
	FDateTime lastCheck;

};
