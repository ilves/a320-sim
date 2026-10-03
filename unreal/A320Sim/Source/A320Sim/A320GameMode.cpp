#include "A320GameMode.h"

#include "A320Aircraft.h"
#include "A320Hud.h"
#include "A320PlayerController.h"
#include "Engine/World.h"

AA320GameMode::AA320GameMode()
{
	DefaultPawnClass = AA320Aircraft::StaticClass();
	PlayerControllerClass = AA320PlayerController::StaticClass();
	HUDClass = AA320Hud::StaticClass();
}

void AA320GameMode::RestartPlayer(AController* NewPlayer)
{
	if (!NewPlayer || NewPlayer->GetPawn())
	{
		return;
	}
	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AA320Aircraft* Aircraft = GetWorld()->SpawnActor<AA320Aircraft>(AA320Aircraft::StaticClass(), FTransform::Identity, Params);
	NewPlayer->Possess(Aircraft);
}
