#include "A320GameInstance.h"

#include "A320Sim.h"
#include "Engine/EngineBaseTypes.h"
#include "GameFramework/GameModeBase.h"

AGameModeBase* UA320GameInstance::CreateGameModeForURL(FURL InURL, UWorld* InWorld)
{
	// A "game=" URL option takes precedence over the map's world settings.
	InURL.AddOption(TEXT("game=/Script/A320Sim.A320GameMode"));
	AGameModeBase* GameMode = Super::CreateGameModeForURL(InURL, InWorld);
	UE_LOG(LogA320, Log, TEXT("Game mode: %s"), GameMode ? *GameMode->GetClass()->GetName() : TEXT("none"));
	return GameMode;
}
