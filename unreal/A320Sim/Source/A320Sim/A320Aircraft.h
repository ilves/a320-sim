#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "A320Commands.h"
#include "A320Shapes.h"
#include "a320/a320_api.h"

#include "A320Aircraft.generated.h"

class AA320World;
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
	void ExecuteCommand(EA320Command Command);
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
	int32 NdRangeNm = 10;
	FRotator LookOffset = FRotator::ZeroRotator;

	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<UCameraComponent> CockpitCamera;
	UPROPERTY() TObjectPtr<USpringArmComponent> ChaseArm;
	UPROPERTY() TObjectPtr<UCameraComponent> ChaseCamera;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> ModelParts;
	UPROPERTY() TObjectPtr<AA320World> World;
	UPROPERTY() FA320Shapes Shapes;
};
