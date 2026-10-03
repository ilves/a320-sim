#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"

#include "A320GameMode.generated.h"

UCLASS()
class AA320GameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AA320GameMode();

	// The level has no PlayerStart: the aircraft places itself from the simulation state.
	virtual void RestartPlayer(AController* NewPlayer) override;
};
