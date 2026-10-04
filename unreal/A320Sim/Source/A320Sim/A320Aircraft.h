#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "A320Commands.h"
#include "A320Shapes.h"
#include "a320/a320_api.h"

#include "A320Aircraft.generated.h"

class AA320World;
class UAudioComponent;
class UPointLightComponent;
class USpotLightComponent;
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
	void SetLever(EA320Lever Lever, double Position);
	// Thrust levers from a hardware throttle (0..1; with reverse the reverse amount). Without
	// bSplit engine 2 follows lever 1; keyboard, mouse and commands move both together again.
	void SetThrustLevers(double Lever1, bool bReverse1, double Lever2, bool bReverse2, bool bSplit);
	void SetSwitch(EA320Switch Switch, int32 Value);
	void SetSpeedbrake(double Amount);  // 0..1, from a hardware lever
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
	bool IsOverheadVisible() const { return bOverheadVisible; }
	bool IsNdRose() const { return NdMode != A320_ND_ARC; }
	int32 GetNdMode() const { return NdMode; }  // A320_ND_*
	bool IsGuideMenuVisible() const { return bGuideMenu; }
	bool IsMcduVisible() const { return bMcduVisible; }
	void McduKey(int32 Key);
	void GetMcduDisplay(A320McduDisplay& Out) const;
	A320GuideStatus GetGuideStatus() const;
	FString GetGuideAlert() const;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	void BuildModel();
	UStaticMeshComponent* AddPart(UStaticMesh* Mesh, const FVector& CentreM, const FRotator& Rotation,
		const FVector& SizeM, const FLinearColor& Color);
	void ResetScenario(A320Scenario Scenario);
	void StartGuide(int32 Guide);
	void UpdateExteriorLights();
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
	bool bOverheadVisible = false;
	int32 NdMode = A320_ND_ARC;
	bool bGuideMenu = false;
	bool bMcduVisible = false;
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
	UPROPERTY() TObjectPtr<UPointLightComponent> BeaconLight;
	UPROPERTY() TObjectPtr<UPointLightComponent> StrobeLeft;
	UPROPERTY() TObjectPtr<UPointLightComponent> StrobeRight;
	UPROPERTY() TObjectPtr<UPointLightComponent> NavLeft;
	UPROPERTY() TObjectPtr<UPointLightComponent> NavRight;
	UPROPERTY() TObjectPtr<USpotLightComponent> LandingLeft;
	UPROPERTY() TObjectPtr<USpotLightComponent> LandingRight;
	UPROPERTY() TObjectPtr<USpotLightComponent> NoseLight;
	UPROPERTY() FA320Shapes Shapes;
};
