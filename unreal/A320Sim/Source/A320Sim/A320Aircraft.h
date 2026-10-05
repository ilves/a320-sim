#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "A320Commands.h"
#include "A320Shapes.h"
#include "a320/a320_api.h"

#include "A320Aircraft.generated.h"

class AA320Fx;

// Where a free flight goes when it is not to an airport: a place picked on the world map.
struct FA320Destination
{
	bool bSet = false;
	FString Name;
	double NorthM = 0.0, EastM = 0.0;
};
class AA320World;
struct FA320FxSpec;
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
	// Param: the extra value some buttons carry (a transponder digit or mode, an ATC reply number).
	void ExecuteCommand(EA320Command Command, bool bLarge = false, int32 Param = 0);
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
	const TArray<A320AirportInfo>& GetAirports() const { return Airports; }
	// The flight plan's waypoints (a320_get_waypoint), refreshed every frame.
	const TArray<A320Waypoint>& GetRoute() const { return Route; }
	// The route the flight being set up will get (a320_preview_route), while the setup screen is open.
	const TArray<A320Waypoint>& GetPreviewRoute() const { return PreviewRoute; }
	// At the nearest airport.
	double GetMagneticVariation() const { return State.magneticVariationDeg; }
	double GetFieldElevationFt() const;
	// The FLIGHT menu and the flight it sets up: departure and arrival (indices into GetRunways()),
	// how the flight starts and, for a start in the air, how far out.
	// The flight setup screen (map, flight, weather) before every flight; the sim waits paused.
	bool IsSetupVisible() const { return bSetup; }
	bool IsMapVisible() const { return bMapVisible || bSetup; }
	bool IsMapWindowVisible() const { return bMapVisible && !bSetup; }
	EA320Confirm GetConfirm() const { return Confirm; }
	// From the world map: depart from an airport (its best runway), fly to an airport or to a place.
	void SetDepartureAirport(int32 Airport);
	void SetArrivalAirport(int32 Airport);
	void SetPlaceDestination(const FA320Destination& Place);
	// Back to a circuit: land where the flight departs.
	void SetLocalFlight();
	// A place picked on the map (no ATC on such a flight), or none: the arrival airport then.
	const FA320Destination& GetPlaceDestination() const { return PlaceDestination; }
	// The runway to use at an airport: one with an ILS, else the first.
	int32 BestRunway(int32 Airport) const;
	int32 FindRunway(const TCHAR* Icao, const TCHAR* Ident) const;
	int32 GetDepRunway() const { return DepRunway; }
	int32 GetArrRunway() const { return ArrRunway; }
	A320Scenario GetFlightScenario() const { return FlightScenario; }
	int32 GetFlightDistanceNm() const { return FlightDistanceNm; }
	int32 GetFlightPlan() const { return FlightPlan; }  // A320FlightPlan: how much the MCDU starts with
	int32 GetWeather() const { return Weather; }  // A320Weather
	bool IsNight() const { return bNight; }
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
	// RADIO window: the radio log (heard transmissions, newest last), the transponder digits being
	// typed, and the latest ATC call for the subtitle.
	bool IsRadioVisible() const { return bRadioVisible; }
	const TArray<A320AtcMessage>& GetRadioLog() const { return RadioLog; }
	const FString& GetXpdrEntry() const { return XpdrEntry; }
	A320AtcStatus GetAtcStatus() const;
	const FString& GetAtcSubtitle(double& OutAgeSeconds) const;
	bool HasVoices() const;
	void McduKey(int32 Key);
	void GetMcduDisplay(A320McduDisplay& Out) const;
	A320GuideStatus GetGuideStatus() const;
	FString GetGuideAlert() const;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	void BuildModel();
	// Parent: Root (the nose section) or RearRoot (the rest), which part flies on after a breakup.
	UStaticMeshComponent* AddPart(UStaticMesh* Mesh, const FVector& CentreM, const FRotator& Rotation,
		const FVector& SizeM, const FLinearColor& Color, USceneComponent* Parent);
	// A destroyed aircraft (A320State.destroyed): the pieces, the effects and the view.
	void UpdateDestruction();
	void StartBreakup();
	void StartCrash();
	void ClearDestruction();
	AA320Fx* SpawnFx(const FVector& LocationCm, const FA320FxSpec& Spec);
	// The chase view of a wreck: steady instead of tumbling with the pieces, the view kept as it was.
	void SteadyChaseView(double ArmLengthCm);
	void ResetScenario(A320Scenario Scenario);
	void StartGuide(int32 Guide);
	void UpdateExteriorLights();
	void ApplyView();
	void UpdateTransform();
	void StartAudio();
	void PumpAudio(float DeltaSeconds);
	void AdjustFcu(double DSpd, double DHdg, double DAlt, double DVs);
	void AdjustVs(int32 Clicks);

	A320Sim* Sim = nullptr;
	A320State State{};
	A320Controls Controls{};
	FString SimError;
	TArray<A320RunwayInfo> Runways;
	TArray<A320Waypoint> Route;
	TArray<A320Waypoint> PreviewRoute;
	FString PreviewKey;  // the selection PreviewRoute was made for
	void RefreshPreviewRoute();
	TArray<A320AirportInfo> Airports;
	int32 DepRunway = 0;
	int32 ArrRunway = 0;
	A320Scenario FlightScenario = A320_SCENARIO_RUNWAY;
	int32 FlightDistanceNm = 20;
	int32 FlightPlan = A320_PLAN_FULL;
	// The weather and day or night: kept for every flight until changed, applied at once.
	int32 Weather = A320_WEATHER_SUNNY;
	bool bNight = false;
	void ApplyWeather();
	bool bSetup = true;  // shown at start, and after a flight ended with Esc
	EA320Confirm Confirm = EA320Confirm::None;
	bool bPausedBeforeConfirm = false;
	void EnterSetup();
	void LeaveSetup();  // FLY: the flight as set up
	void AskConfirm(EA320Confirm What);
	bool bMapVisible = false;
	FA320Destination PlaceDestination;
	bool bLessonStart = false;  // ResetScenario for a lesson: its own flight plan
	double FieldElevationFt = 0.0;  // the first airport's, the flat world's zero

	bool bCockpitView = true;
	bool bLsOn = false;
	bool bHelpVisible = true;
	bool bSoundOn = true;
	bool bOverheadVisible = false;
	int32 NdMode = A320_ND_ARC;
	bool bGuideMenu = false;
	bool bMcduVisible = false;
	bool bRadioVisible = false;
	void PumpRadio();
	TSharedPtr<class FA320Voice> Voice;
	uint32 LastAtcSeq = 0;
	uint32 LastDestroyedSeq = 0;
	double WreckViewYawDeg = 0.0;  // the chase camera's look direction at a breakup or crash
	uint32 LastImpactSeq[2] = {0, 0};
	TArray<A320AtcMessage> RadioLog;
	FString XpdrEntry;
	FString AtcSubtitle;
	double AtcSubtitleAt = -1000.0;
	int32 AudioRate = 44100;
	TArray<int16> AudioScratch;
	int32 NdRangeNm = 10;
	FRotator LookOffset = FRotator::ZeroRotator;

	UPROPERTY() TObjectPtr<USceneComponent> Root;
	// The wings, engines and tail: on the aircraft until a breakup, then on their own.
	UPROPERTY() TObjectPtr<USceneComponent> RearRoot;
	UPROPERTY() TArray<TObjectPtr<AA320Fx>> Effects;
	UPROPERTY() TObjectPtr<AA320Fx> TrailFront;
	UPROPERTY() TObjectPtr<AA320Fx> TrailRear;
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
