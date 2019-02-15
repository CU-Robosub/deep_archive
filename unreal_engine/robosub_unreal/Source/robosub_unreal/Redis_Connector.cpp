// Fill out your copyright notice in the Description page of Project Settings.

#include "Redis_Connector.h"

Redis_Connector::Redis_Connector()
{
	
	lastCheck = FDateTime::UtcNow();
	UE_LOG(LogTemp, Warning, TEXT("Setting Up Redis Connection"));
	FIPv4Address ip;
	ip = FIPv4Address(127, 0, 0, 1);
	TSharedRef<FInternetAddr> addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	addr->SetIp(ip.Value);
	addr->SetPort(6379);
	ConnectionSocket = FTcpSocketBuilder("Redis_Socket").AsBlocking();
	connected = ConnectionSocket->Connect(*addr);
	if (connected) {
		UE_LOG(LogTemp, Warning, TEXT("Connected!"));
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("NOT Connected!"));
	}
}

Redis_Connector::~Redis_Connector()
{
}

bool Redis_Connector::connect() {
	if (connected) return connected;
	FDateTime now =FDateTime::UtcNow();
	int64 duration = now.ToUnixTimestamp() - lastCheck.ToUnixTimestamp();
	if (duration < 10)
	{
		//UE_LOG(LogTemp, Warning, TEXT("Waiting 10 seconds before checking for redis again: Time since last check %d"), duration);
		return false;
	}
	lastCheck = FDateTime::UtcNow();
	UE_LOG(LogTemp, Warning, TEXT("Setting Up Redis Connection"));
	FIPv4Address ip;
	ip = FIPv4Address(127, 0, 0, 1);
	TSharedRef<FInternetAddr> addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	addr->SetIp(ip.Value);
	addr->SetPort(6379);
	ConnectionSocket = FTcpSocketBuilder("Redis_Socket").AsBlocking();
	connected = ConnectionSocket->Connect(*addr);
	if (connected) {
		UE_LOG(LogTemp, Warning, TEXT("Connected!"));
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("NOT Connected!"));
	}
	return connected;
}

double Redis_Connector::get_key(FString key) {
	//first check if we are connected
	if (!connect()) return 0.0;
	int32 BytesSent = 0;
	FBufferArchive buffer;
	FString serialized =TEXT("*2\r\n$3\r\nGET\r\n$")+FString::FromInt(key.Len())+TEXT("\r\n")+key+TEXT("\r\n");
	TCHAR *serializedChar = serialized.GetCharArray().GetData();
	int32 size = FCString::Strlen(serializedChar);
	ConnectionSocket->Send((uint8*)TCHAR_TO_UTF8(serializedChar), size, BytesSent);
	uint8 recvData[100] = "test";
	int32 BytesRead = 1;
	ConnectionSocket->Recv(recvData, 100, BytesRead);
	TCHAR m[100];
	bool after_buff = false;
	for (int i = 0; i < 100; i++) {
		m[i] = recvData[i];
	}
	FString temp = m;
	TArray<FString> splitarray;
	const TCHAR * pchDelim = TEXT("\r\n");
	temp.ParseIntoArray(splitarray, pchDelim,true);
	return FCString::Atof(*splitarray[1]);
}

int Redis_Connector::set_key(FString key, double set) {
	//first check if we are connected
	if (!connect()) return 1;
	int32 BytesSent = 0;
	FBufferArchive buffer;
	FString set_value = FString::SanitizeFloat(set);
	FString serialized = TEXT("*3\r\n$3\r\nSET\r\n$") + FString::FromInt(key.Len()) + TEXT("\r\n") + key 
		+ TEXT("\r\n$")+ FString::FromInt(set_value.Len()) + TEXT("\r\n") + set_value +TEXT("\r\n");
	TCHAR *serializedChar = serialized.GetCharArray().GetData();
	int32 size = FCString::Strlen(serializedChar);
	ConnectionSocket->Send((uint8*)TCHAR_TO_UTF8(serializedChar), size, BytesSent);

	uint8 recvData[100] = "test";
	int32 BytesRead = 1;
	ConnectionSocket->Recv(recvData, 100, BytesRead);
	TCHAR m[100];
	for (int i = 1; i < 100; i++) {
		m[i] = recvData[i];
	}
	return 0;
}

int Redis_Connector::set_key_string(FString key, FString set) {
	//first check if we are connected
	if (!connect()) return 1;
	int32 BytesSent = 0;
	FBufferArchive buffer;
	FString serialized = TEXT("*3\r\n$3\r\nSET\r\n$") + FString::FromInt(key.Len()) + TEXT("\r\n") + key
		+ TEXT("\r\n$") + FString::FromInt(set.Len()) + TEXT("\r\n") + set + TEXT("\r\n");
	TCHAR *serializedChar = serialized.GetCharArray().GetData();
	int32 size = FCString::Strlen(serializedChar);
	ConnectionSocket->Send((uint8*)TCHAR_TO_UTF8(serializedChar), size, BytesSent);

	uint8 recvData[100] = "test";
	int32 BytesRead = 1;
	ConnectionSocket->Recv(recvData, 100, BytesRead);
	TCHAR m[100];
	for (int i = 1; i < 100; i++) {
		m[i] = recvData[i];
	}
	return 0;
}

int Redis_Connector::parseReturnString(char * p) {
	int len = 0;

	p++;
	while (*p != '\r') {
		len = (len * 10) + (*p - '0');
		p++;
	}

	/* Now p points at '\r', and the len is in bulk_len. */
	printf("%d\n", len);
	return 0;
}