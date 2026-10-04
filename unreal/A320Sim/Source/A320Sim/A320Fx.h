#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Math/RandomStream.h"

#include "A320Shapes.h"

#include "A320Fx.generated.h"

class UMaterialInstanceDynamic;
class UMaterialInterface;
class UPointLightComponent;
class UStaticMeshComponent;
class UTexture2D;

// What an effect shows. Sizes scale with SizeM (the fireball's radius).
struct FA320FxSpec
{
	double SizeM = 30.0;
	bool bExplosion = true;
	bool bFire = true;
	bool bSmoke = true;
	int32 DebrisCount = 0;
	FVector DebrisVelocity = FVector::ZeroVector;  // cm/s: the wreck's own motion, the debris carry part of it
	double GroundZ = 0.0;                          // cm: where debris comes to rest
	int32 PuffPool = 120;                          // smoke puffs alive at most
	double PuffLifeS = 20.0;
	double PuffScale = 1.0;                        // puff size relative to SizeM
};

// Game effects for a destroyed aircraft, built from engine primitives so the repository needs no
// assets: an explosion fireball with its flash, flickering fire, a drifting smoke column and
// debris thrown out. A source moved every frame (a falling section) leaves a smoke trail.
UCLASS()
class AA320Fx : public AActor
{
	GENERATED_BODY()

public:
	AA320Fx();
	virtual void Tick(float DeltaSeconds) override;

	void Start(const FA320FxSpec& InSpec);
	// The fire follows the source; new smoke starts there, so a moving source leaves a trail.
	void MoveSource(const FVector& LocationCm);
	// No more fire or new smoke; the smoke already made drifts away.
	void Extinguish();

private:
	struct FPuff
	{
		FVector Pos = FVector::ZeroVector, Vel = FVector::ZeroVector;
		double SizeM = 0.0, Age = 0.0, Life = 0.0;
		bool bAlive = false;
	};
	struct FPiece
	{
		FVector Pos = FVector::ZeroVector, Vel = FVector::ZeroVector, SizeM = FVector::OneVector;
		FRotator Rot = FRotator::ZeroRotator, Spin = FRotator::ZeroRotator;  // deg, deg/s
		bool bResting = false;
	};

	UStaticMeshComponent* AddMesh(UStaticMesh* Mesh, UMaterialInterface* Material);
	// Unlit, so fire glows at night too: the engine's unlit widget material with a 1-pixel texture.
	UMaterialInterface* Glow(const FLinearColor& Color);
	void UpdateExplosion(double T);
	void UpdateFire(double T);
	void UpdateSmoke(double Dt);
	void UpdateDebris(double Dt);
	void EmitPuff(const FVector& At);

	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<UPointLightComponent> Light;
	UPROPERTY() FA320Shapes Shapes;
	UPROPERTY() TObjectPtr<UMaterialInterface> UnlitMaterial;
	UPROPERTY() TArray<TObjectPtr<UTexture2D>> GlowTextures;
	UPROPERTY() TMap<uint32, TObjectPtr<UMaterialInstanceDynamic>> GlowCache;
	UPROPERTY() TObjectPtr<UStaticMeshComponent> FireballInner;
	UPROPERTY() TObjectPtr<UStaticMeshComponent> FireballOuter;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Flames;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> PuffMeshes;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> PieceMeshes;

	FA320FxSpec Spec;
	FVector Source = FVector::ZeroVector;
	FVector LastEmit = FVector::ZeroVector;  // where the last puff started
	TArray<FVector> FlameOffsets;  // cm, around the source
	TArray<FPuff> Puffs;
	TArray<FPiece> Pieces;
	double Age = 0.0;
	double NextPuffAt = 0.0;
	bool bStarted = false;
	bool bExtinguished = false;
	FRandomStream Random;
};
