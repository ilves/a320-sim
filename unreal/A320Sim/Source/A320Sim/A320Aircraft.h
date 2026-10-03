#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "A320Commands.h"
#include "A320Shapes.h"
#include "a320/a320_api.h"

#include "A320Aircraft.generated.h"

class AA320World;
class UAudioComponent;
class USoundWaveProcedural;
class UCameraComponent;
class USpringArmComponent;
class UStaticMeshComponent;

// Continuous pilot inputs, sampled every frame by the player controller.
struct FA320FlightInputs
{
	double StickPitch = 0.0;  // +1 = full back
	double StickRoll = 0.0;   // +1 = full right
	double Pedals = 0.0;      // +1 = right
	double Brakes = 0.0;      // 0..1
	double ThrustRate = 0.0;  // lever movement per second
};

// The A320: owns the JSBSim-based core (A320Core.dll), moves itself from its state and
// executes cockpit commands. Visuals are built from engine primitives.
UCLASS()
class AA320Aircraft : public APawn
{
	GENERATED_BODY()

public:
	AA320Aircraft();

	virtual void Tick(float DeltaSeconds) override;

	void SetFlightInputs(const FA320FlightInputs& Inputs, float DeltaSeconds);
	// bLarge: Shift held, for 10x FCU steps.
	void ExecuteCommand(EA320Command Command, bool bLarge = false);
	void AddLook(double YawDeg, double PitchDeg);
	void ResetLook();

	bool IsSimReady() const { return Sim != nullptr; }
	const FString& GetSimError() const { return SimError; }
	const A320State& GetSimState() const { return State; }
	const A320Controls& GetSimControls() const { return Controls; }
	const TArray<A320RunwayInfo>& GetRunways() const { return Runways; }
	int32 GetActiveRunway() const { return ActiveRunway; }
	double GetMagneticVariation() const { return MagneticVariationDeg; }
	double GetFieldElevationFt() const { return FieldElevationFt; }
	bool IsCockpitView() const { return bCockpitView; }
	bool IsLsOn() const { return bLsOn; }
	int32 GetNdRangeNm() const { return NdRangeNm; }
	bool IsHelpVisible() const { return bHelpVisible; }
	bool IsSoundOn() const { return bSoundOn; }

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	void BuildModel();
	UStaticMeshComponent* AddPart(UStaticMesh* Mesh, const FVector& CentreM, const FRotator& Rotation,
		const FVector& SizeM, const FLinearColor& Color);
	void ResetScenario(A320Scenario Scenario);
	void SyncControlsFromState();
	void ApplyView();
	void UpdateTransform();
	void StartAudio();
	void PumpAudio(float DeltaSeconds);
	void AdjustFcu(double DSpd, double DHdg, double DAlt, double DVs);

	A320Sim* Sim = nullptr;
	A320State State{};
	A320Controls Controls{};
	FString SimError;
	TArray<A320RunwayInfo> Runways;
	int32 ActiveRunway = 0;
	double MagneticVariationDeg = 0.0;
	double FieldElevationFt = 0.0;

	bool bCockpitView = true;
	bool bLsOn = false;
	bool bHelpVisible = true;
	bool bSoundOn = true;
	int32 AudioRate = 44100;
	TArray<int16> AudioScratch;
	int32 NdRangeNm = 10;
	FRotator LookOffset = FRotator::ZeroRotator;

	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<UCameraComponent> CockpitCamera;
	UPROPERTY() TObjectPtr<USpringArmComponent> ChaseArm;
	UPROPERTY() TObjectPtr<UCameraComponent> ChaseCamera;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> ModelParts;
	UPROPERTY() TObjectPtr<AA320World> World;
	UPROPERTY() TObjectPtr<UAudioComponent> AudioOut;
	UPROPERTY() TObjectPtr<USoundWaveProcedural> AudioWave;
	UPROPERTY() FA320Shapes Shapes;
};
