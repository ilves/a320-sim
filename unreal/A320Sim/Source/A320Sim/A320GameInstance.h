#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"

#include "A320GameInstance.generated.h"

UCLASS()
class UA320GameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	// The engine's /Engine/Maps/Entry may name its own game mode in its world settings, which
	// would beat GlobalDefaultGameMode; this project only ever runs the simulator.
	virtual AGameModeBase* CreateGameModeForURL(FURL InURL, UWorld* InWorld) override;
};
