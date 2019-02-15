// Fill out your copyright notice in the Description page of Project Settings.

#include "Runtime/Engine/Classes/Components/SceneCaptureComponent2D.h"
#include "Runtime/Engine/Classes/Engine/TextureRenderTarget2D.h"
#include "Runtime/Engine/Classes/Engine/Texture2D.h"
#include "Redis_Connector.h"
#pragma once
#include "CoreMinimal.h"
#pragma once
#include "GameFramework/Pawn.h"
#pragma once
#include "Leviathan.generated.h"


UCLASS()
class ROBOSUB_UNREAL_API ALeviathan : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	ALeviathan();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	//virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
	// Called to reset the actor
	UFUNCTION(BlueprintCallable, Category ="Sub")
	void tripReset();
	// Called to get the current Location of the actor
	UFUNCTION(BlueprintCallable, Category = "Sub")
	FVector getLocation();
	UFUNCTION(BlueprintCallable, Category = "Sub")
	FRotator getRotation();
	//Functions to set X,Y,Z targets
	void setXTarget(float x);
	void setYTarget(float y);
	void setZTarget(float z);
	//Functions to set Roll, Pitch, Yaw
	void setRollTarget(float r);
	void setPitchTarget(float p);
	void setYawTarget(float y);

	UPROPERTY(EDITAnywhere, BlueprintReadWrite, Category = "Sub")
	float movementSpeed = 100;

private:
	bool reset;
	FVector location;
	FRotator rotation;
	FVector targetLinear;
	FRotator targetRotation;
	Redis_Connector redis;
	template<class T>
	T updatePos(T current, T target, T movement_speed, float DeltaTime);
	double last_x = 0;
	double last_y = 0;
	USceneCaptureComponent2D *Camera2;
	UTextureRenderTarget2D *RenderTarget;
	UTexture2D *Texture2D;
	void captureCamera();
	int image_counter;

};
