#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"

#include "A320PlayerController.generated.h"

// Polls keyboard, gamepad and mouse every frame (no input assets needed) and drives the
// aircraft; mouse clicks go to the cockpit panel drawn by AA320Hud.
UCLASS()
class AA320PlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AA320PlayerController();

	virtual void PlayerTick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	double KeyStickPitch = 0.0;
	double KeyStickRoll = 0.0;
	double KeyPedals = 0.0;
	FVector2D LastMouse = FVector2D::ZeroVector;
	bool bLooking = false;
};
