#include "A320Aircraft.h"

#include "A320Sim.h"
#include "A320World.h"
#include "Camera/CameraComponent.h"
#include "Components/AudioComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Sound/SoundWaveProcedural.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/SpringArmComponent.h"
#include "Misc/Paths.h"

namespace
{
	// JSBSim EYEPOINT relative to the CG (A320.xml: x aft, z up, inches): captain's seat.
	const FVector CockpitEyeCm(1504.0, -76.0, 279.0);
	const double CockpitPitchDeg = -5.0;
	const float CockpitFovDeg = 90.0f;
}

AA320Aircraft::AA320Aircraft()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	// The simulation owns the attitude; the controller must not steer the pawn's yaw.
	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
	bUseControllerRotationRoll = false;
	Shapes.LoadInConstructor();

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Root->SetMobility(EComponentMobility::Movable);
	SetRootComponent(Root);

	CockpitCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("CockpitCamera"));
	CockpitCamera->SetupAttachment(Root);
	CockpitCamera->SetRelativeLocation(CockpitEyeCm);
	CockpitCamera->SetRelativeRotation(FRotator(CockpitPitchDeg, 0.0, 0.0));
	CockpitCamera->SetFieldOfView(CockpitFovDeg);

	ChaseArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("ChaseArm"));
	ChaseArm->SetupAttachment(Root);
	ChaseArm->TargetArmLength = 7000.0f;
	ChaseArm->SetRelativeRotation(FRotator(-12.0, 0.0, 0.0));
	ChaseArm->TargetOffset = FVector(0.0, 0.0, -1200.0);
	ChaseArm->bDoCollisionTest = false;
	ChaseArm->bInheritRoll = false;
	ChaseArm->bEnableCameraLag = true;
	ChaseArm->CameraLagSpeed = 8.0f;
	ChaseArm->bEnableCameraRotationLag = true;
	ChaseArm->CameraRotationLagSpeed = 4.0f;

	ChaseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
	ChaseCamera->SetupAttachment(ChaseArm, USpringArmComponent::SocketName);
	ChaseCamera->SetFieldOfView(70.0f);

	// Exterior lights (metres from the CG): beacon on top, strobes and nav lights at the wing
	// tips, landing lights in the wing roots, taxi/take-off light on the nose gear.
	auto MakePoint = [this](const TCHAR* Name, const FVector& PosM, const FLinearColor& Color, float Candela)
	{
		UPointLightComponent* L = CreateDefaultSubobject<UPointLightComponent>(Name);
		L->SetupAttachment(Root);
		L->SetRelativeLocation(PosM * 100.0);
		L->SetIntensityUnits(ELightUnits::Candelas);
		L->SetIntensity(Candela);
		L->SetLightColor(Color);
		L->SetAttenuationRadius(3000.0f);
		L->SetCastShadows(false);
		L->SetVisibility(false);
		return L;
	};
	auto MakeSpot = [this](const TCHAR* Name, const FVector& PosM, float PitchDeg, float Candela, float ConeDeg)
	{
		USpotLightComponent* L = CreateDefaultSubobject<USpotLightComponent>(Name);
		L->SetupAttachment(Root);
		L->SetRelativeLocation(PosM * 100.0);
		L->SetRelativeRotation(FRotator(PitchDeg, 0.0, 0.0));
		L->SetIntensityUnits(ELightUnits::Candelas);
		L->SetIntensity(Candela);
		L->SetOuterConeAngle(ConeDeg);
		L->SetInnerConeAngle(ConeDeg * 0.6f);
		L->SetAttenuationRadius(120000.0f);
		L->SetCastShadows(false);
		L->SetVisibility(false);
		return L;
	};
	BeaconLight = MakePoint(TEXT("Beacon"), FVector(-2.0, 0.0, 3.1), FLinearColor(1.0f, 0.05f, 0.05f), 2000.0f);
	StrobeLeft = MakePoint(TEXT("StrobeL"), FVector(-6.0, -17.0, 0.6), FLinearColor::White, 20000.0f);
	StrobeRight = MakePoint(TEXT("StrobeR"), FVector(-6.0, 17.0, 0.6), FLinearColor::White, 20000.0f);
	NavLeft = MakePoint(TEXT("NavL"), FVector(-5.8, -17.0, 0.6), FLinearColor(1.0f, 0.0f, 0.0f), 300.0f);
	NavRight = MakePoint(TEXT("NavR"), FVector(-5.8, 17.0, 0.6), FLinearColor(0.0f, 1.0f, 0.1f), 300.0f);
	LandingLeft = MakeSpot(TEXT("LandingL"), FVector(2.5, -4.0, -1.0), -4.0f, 150000.0f, 12.0f);
	LandingRight = MakeSpot(TEXT("LandingR"), FVector(2.5, 4.0, -1.0), -4.0f, 150000.0f, 12.0f);
	NoseLight = MakeSpot(TEXT("NoseLight"), FVector(12.3, 0.0, -1.6), -6.0f, 60000.0f, 25.0f);

	AudioOut = CreateDefaultSubobject<UAudioComponent>(TEXT("Audio"));
	AudioOut->SetupAttachment(Root);
	AudioOut->bAutoActivate = false;
	AudioOut->bAllowSpatialization = false;
}

void AA320Aircraft::BeginPlay()
{
	Super::BeginPlay();
	BuildModel();

	if (!A320CoreDll::IsLoaded())
	{
		SimError = TEXT("A320Core.dll is not loaded. Run Setup.bat, then start the game again.");
	}
	else if (a320_api_version() != A320_API_VERSION)
	{
		SimError = TEXT("A320Core.dll does not match this build. Run Setup.bat again.");
	}
	else
	{
		const FString DataDir = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectContentDir(), TEXT("JSBSim")));
		char Error[512] = {0};
		Sim = a320_create(TCHAR_TO_UTF8(*DataDir), Error, (int)sizeof(Error));
		if (!Sim)
		{
			SimError = FString::Printf(TEXT("Flight model failed to load from %s: %s"), *DataDir, UTF8_TO_TCHAR(Error));
		}
	}
	if (!Sim)
	{
		UE_LOG(LogA320, Error, TEXT("%s"), *SimError);
		ApplyView();
		return;
	}

	FieldElevationFt = a320_field_elevation_ft(Sim);
	MagneticVariationDeg = a320_magnetic_variation_deg(Sim);
	for (int32 i = 0; i < a320_runway_count(Sim); ++i)
	{
		A320RunwayInfo Info;
		if (a320_get_runway(Sim, i, &Info))
		{
			Runways.Add(Info);
			if (FCString::Strcmp(UTF8_TO_TCHAR(Info.ident), TEXT("26")) == 0)
			{
				ActiveRunway = i;
			}
		}
	}

	FActorSpawnParameters Params;
	Params.Owner = this;
	World = GetWorld()->SpawnActor<AA320World>(AA320World::StaticClass(), FTransform::Identity, Params);
	if (World)
	{
		World->Build(Runways);
	}

	ResetScenario(A320_SCENARIO_RUNWAY);
	ApplyView();
	StartAudio();
	UE_LOG(LogA320, Log, TEXT("Flight model ready: %d runways, lined up on %s"), Runways.Num(),
		Runways.IsValidIndex(ActiveRunway) ? UTF8_TO_TCHAR(Runways[ActiveRunway].ident) : TEXT("?"));
}

void AA320Aircraft::EndPlay(const EEndPlayReason::Type Reason)
{
	if (AudioOut)
	{
		AudioOut->Stop();
	}
	if (Sim)
	{
		a320_destroy(Sim);
		Sim = nullptr;
	}
	Super::EndPlay(Reason);
}

void AA320Aircraft::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!Sim)
	{
		return;
	}
	const double SimTimeBefore = State.simTimeS;
	a320_set_controls(Sim, &Controls);
	a320_update(Sim, DeltaSeconds);
	a320_get_state(Sim, &State);
	// Only once the core has actually stepped (not when paused, or on a frame shorter than its
	// 120 Hz step): release the momentary APU START, and let the autobrake selector follow the
	// core, which disarms it when the pilot brakes.
	if (State.simTimeS > SimTimeBefore)
	{
		Controls.apuStart = 0;
		Controls.autobrake = State.autobrake;
	}
	UpdateTransform();
	UpdateExteriorLights();
	if (World)
	{
		World->UpdatePapi(State.ilsRunwayIndex, State.papiWhite);
	}
	PumpAudio(DeltaSeconds);
}

void AA320Aircraft::StartAudio()
{
	const FString SoundsDir = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Sounds")));
	if (!Sim || !a320_audio_init(Sim, AudioRate, TCHAR_TO_UTF8(*SoundsDir)))
	{
		UE_LOG(LogA320, Warning, TEXT("Audio disabled"));
		return;
	}
	// The core renders the whole cockpit soundscape; this wave just streams it.
	AudioWave = NewObject<USoundWaveProcedural>(this);
	AudioWave->SetSampleRate(AudioRate);
	AudioWave->NumChannels = 1;
	AudioWave->Duration = INDEFINITELY_LOOPING_DURATION;
	AudioWave->SoundGroup = SOUNDGROUP_Default;
	AudioWave->bLooping = false;
	PumpAudio(0.0f);
	AudioOut->SetSound(AudioWave);
	AudioOut->Play();
	UE_LOG(LogA320, Log, TEXT("Audio started (%d Hz, sounds from %s)"), AudioRate, *SoundsDir);
}

void AA320Aircraft::PumpAudio(float DeltaSeconds)
{
	if (!Sim || !AudioWave)
	{
		return;
	}
	// Keep ~200 ms queued: rides out frame hitches such as a scenario reset.
	const int32 TargetBytes = AudioRate * 2 / 5;  // 200 ms
	const int32 Missing = TargetBytes - AudioWave->GetAvailableAudioByteCount();
	if (Missing <= 0)
	{
		return;
	}
	const int32 Frames = Missing / 2;
	AudioScratch.SetNumUninitialized(Frames);
	a320_audio_render(Sim, AudioScratch.GetData(), Frames);
	AudioWave->QueueAudio(reinterpret_cast<const uint8*>(AudioScratch.GetData()), Frames * 2);
}

void AA320Aircraft::AdjustFcu(double DSpd, double DHdg, double DAlt, double DVs)
{
	if (Sim)
	{
		a320_fcu_set_targets(Sim, State.fcuSpdKt + DSpd, State.fcuHdgMagDeg + DHdg, State.fcuAltFt + DAlt, State.fcuVsFpm + DVs);
		a320_get_state(Sim, &State);
	}
}

void AA320Aircraft::UpdateTransform()
{
	// Flat world: X north, Y east, Z up from field elevation, in centimetres. FRotator's
	// pitch/yaw/roll match JSBSim's theta/psi/phi signs (nose up, clockwise, right wing down).
	const FVector Location(State.northM * 100.0, State.eastM * 100.0, State.heightAboveFieldM * 100.0);
	const FRotator Rotation(State.pitchDeg, State.headingTrueDeg, State.bankDeg);
	SetActorLocationAndRotation(Location, Rotation);
}

void AA320Aircraft::SetFlightInputs(const FA320FlightInputs& Inputs, float DeltaSeconds)
{
	Controls.stickPitch = FMath::Clamp(Inputs.StickPitch, -1.0, 1.0);
	Controls.stickRoll = FMath::Clamp(Inputs.StickRoll, -1.0, 1.0);
	Controls.pedals = FMath::Clamp(Inputs.Pedals, -1.0, 1.0);
	Controls.brakeLeft = Controls.brakeRight = FMath::Clamp(Inputs.Brakes, 0.0, 1.0);
	if (Inputs.ThrustRate != 0.0 && !State.paused)
	{
		Controls.thrustLever = FMath::Clamp(Controls.thrustLever + Inputs.ThrustRate * DeltaSeconds, 0.0, 1.0);
		Controls.splitThrust = 0;
	}
}

void AA320Aircraft::ExecuteCommand(EA320Command Command, bool bLarge)
{
	if (Command != EA320Command::None && Sim)
	{
		a320_audio_event(Sim, A320_SOUND_CLICK);
	}
	const double Step = bLarge ? 10.0 : 1.0;
	auto Fcu = [this](A320FcuCommand Cmd)
	{
		if (Sim)
		{
			a320_fcu_command(Sim, Cmd);
			a320_get_state(Sim, &State);
		}
	};
	switch (Command)
	{
	case EA320Command::FcuAp: Fcu(A320_FCU_AP1); break;
	case EA320Command::FcuAthr: Fcu(A320_FCU_ATHR); break;
	case EA320Command::FcuHdgPull: Fcu(A320_FCU_HDG_PULL); break;
	case EA320Command::FcuLoc: Fcu(A320_FCU_LOC); break;
	case EA320Command::FcuAppr:
		Fcu(A320_FCU_APPR);
		bLsOn = bLsOn || (State.armed & A320_ARMED_GS) != 0;
		break;
	case EA320Command::ResetColdDark: ResetScenario(A320_SCENARIO_COLD_DARK); break;
	case EA320Command::ApDisconnect:
		if (State.apEngaged)
		{
			Fcu(A320_FCU_AP1);
		}
		break;
	case EA320Command::OverheadToggle: bOverheadVisible = !bOverheadVisible; break;
	case EA320Command::NdModeToggle: bNdRose = !bNdRose; break;
	case EA320Command::EngMaster1: Controls.engMaster[0] = Controls.engMaster[0] ? 0 : 1; break;
	case EA320Command::EngMaster2: Controls.engMaster[1] = Controls.engMaster[1] ? 0 : 1; break;
	case EA320Command::EngModeCrank: Controls.engMode = A320_ENG_MODE_CRANK; break;
	case EA320Command::EngModeNorm: Controls.engMode = A320_ENG_MODE_NORM; break;
	case EA320Command::EngModeIgnStart: Controls.engMode = A320_ENG_MODE_IGN_START; break;
	case EA320Command::ApuMaster: Controls.apuMaster = Controls.apuMaster ? 0 : 1; break;
	case EA320Command::ApuStart: Controls.apuStart = 1; break;
	case EA320Command::ApuBleed: Controls.apuBleed = Controls.apuBleed ? 0 : 1; break;
	case EA320Command::AutobrakeLo:
		Controls.autobrake = State.autobrake == A320_AUTOBRAKE_LO ? A320_AUTOBRAKE_OFF : A320_AUTOBRAKE_LO;
		break;
	case EA320Command::AutobrakeMed:
		Controls.autobrake = State.autobrake == A320_AUTOBRAKE_MED ? A320_AUTOBRAKE_OFF : A320_AUTOBRAKE_MED;
		break;
	case EA320Command::AutobrakeMax:
		Controls.autobrake = State.autobrake == A320_AUTOBRAKE_MAX ? A320_AUTOBRAKE_OFF : A320_AUTOBRAKE_MAX;
		break;
	case EA320Command::SpoilerArm:
		Controls.spoilersArmed = Controls.spoilersArmed ? 0 : 1;
		if (Controls.spoilersArmed)
		{
			Controls.speedbrake = 0.0;
		}
		break;
	case EA320Command::LightStrobe: Controls.lights ^= A320_LT_STROBE; break;
	case EA320Command::LightBeacon: Controls.lights ^= A320_LT_BEACON; break;
	case EA320Command::LightNav: Controls.lights ^= A320_LT_NAV; break;
	case EA320Command::LightLanding: Controls.lights ^= A320_LT_LANDING; break;
	case EA320Command::LightRwyTurnoff: Controls.lights ^= A320_LT_RWY_TURNOFF; break;
	case EA320Command::LightNose:
		// OFF -> TAXI -> T.O -> OFF
		if (Controls.lights & A320_LT_TAKEOFF)
		{
			Controls.lights &= ~(A320_LT_TAKEOFF | A320_LT_TAXI);
		}
		else if (Controls.lights & A320_LT_TAXI)
		{
			Controls.lights = (Controls.lights & ~A320_LT_TAXI) | A320_LT_TAKEOFF;
		}
		else
		{
			Controls.lights |= A320_LT_TAXI;
		}
		break;
	case EA320Command::SignSeatbelts: Controls.signs ^= A320_SIGN_SEATBELTS; break;
	case EA320Command::SignNoSmoking: Controls.signs ^= A320_SIGN_NO_SMOKING; break;
	case EA320Command::FcuAltPull: Fcu(A320_FCU_ALT_PULL); break;
	case EA320Command::FcuVsPull: Fcu(A320_FCU_VS_PULL); break;
	case EA320Command::SpdDec: AdjustFcu(-Step, 0.0, 0.0, 0.0); break;
	case EA320Command::SpdInc: AdjustFcu(Step, 0.0, 0.0, 0.0); break;
	case EA320Command::HdgDec: AdjustFcu(0.0, -Step, 0.0, 0.0); break;
	case EA320Command::HdgInc: AdjustFcu(0.0, Step, 0.0, 0.0); break;
	case EA320Command::AltDec: AdjustFcu(0.0, 0.0, -100.0 * Step, 0.0); break;
	case EA320Command::AltInc: AdjustFcu(0.0, 0.0, 100.0 * Step, 0.0); break;
	case EA320Command::VsDec: AdjustFcu(0.0, 0.0, 0.0, bLarge ? -500.0 : -100.0); break;
	case EA320Command::VsInc: AdjustFcu(0.0, 0.0, 0.0, bLarge ? 500.0 : 100.0); break;
	case EA320Command::MasterWarnAck:
		if (Sim)
		{
			a320_audio_event(Sim, A320_SOUND_ACK_WARNING);
		}
		break;
	case EA320Command::SoundToggle:
		bSoundOn = !bSoundOn;
		if (Sim)
		{
			a320_audio_set_volume(Sim, bSoundOn ? 0.8 : 0.0);
		}
		break;
	case EA320Command::GearToggle: Controls.gearDown = Controls.gearDown ? 0 : 1; break;
	case EA320Command::FlapsUp: Controls.flapsLever = FMath::Max(Controls.flapsLever - 1, 0); break;
	case EA320Command::FlapsDown: Controls.flapsLever = FMath::Min(Controls.flapsLever + 1, 4); break;
	case EA320Command::SpeedbrakeToggle: Controls.speedbrake = Controls.speedbrake > 0.5 ? 0.0 : 1.0; break;
	case EA320Command::ParkBrakeToggle: Controls.parkBrake = Controls.parkBrake ? 0 : 1; break;
	case EA320Command::ReverseToggle:
		// Reversers only deploy on the ground; the thrust lever then sets reverse thrust.
		if (Controls.reverse || State.onGround)
		{
			Controls.reverse = Controls.reverse ? 0 : 1;
			Controls.thrustLever = 0.0;
			Controls.splitThrust = 0;
		}
		break;
	case EA320Command::ThrustIdle: Controls.thrustLever = 0.0; Controls.splitThrust = 0; break;
	case EA320Command::ThrustClimb: Controls.thrustLever = 0.75; Controls.splitThrust = 0; break;
	case EA320Command::ThrustFlex: Controls.thrustLever = 0.88; Controls.splitThrust = 0; break;
	case EA320Command::ThrustToga: Controls.thrustLever = 1.0; Controls.splitThrust = 0; break;
	case EA320Command::LsToggle: bLsOn = !bLsOn; break;
	case EA320Command::NdRangeDown: NdRangeNm = FMath::Max(NdRangeNm / 2, 5); break;
	case EA320Command::NdRangeUp: NdRangeNm = FMath::Min(NdRangeNm * 2, 80); break;
	case EA320Command::PauseToggle:
		if (Sim)
		{
			a320_set_paused(Sim, State.paused ? 0 : 1);
			State.paused = State.paused ? 0 : 1;
		}
		break;
	case EA320Command::SimRateCycle:
		if (Sim)
		{
			const double Rate = State.simRate >= 4.0 ? 1.0 : State.simRate * 2.0;
			a320_set_sim_rate(Sim, Rate);
			State.simRate = Rate;
		}
		break;
	case EA320Command::ViewToggle: bCockpitView = !bCockpitView; ApplyView(); break;
	case EA320Command::HelpToggle: bHelpVisible = !bHelpVisible; break;
	case EA320Command::RunwaySwap:
		if (Runways.Num() > 1)
		{
			ActiveRunway = (ActiveRunway + 1) % Runways.Num();
			ResetScenario(A320_SCENARIO_RUNWAY);
		}
		break;
	case EA320Command::ResetRunway: ResetScenario(A320_SCENARIO_RUNWAY); break;
	case EA320Command::ResetFinal10: ResetScenario(A320_SCENARIO_FINAL_10NM); break;
	case EA320Command::ResetFinal4: ResetScenario(A320_SCENARIO_FINAL_4NM); break;
	case EA320Command::None: break;
	default: break;  // joystick setup commands are handled by the player controller
	}
}

void AA320Aircraft::ResetScenario(A320Scenario Scenario)
{
	if (!Sim)
	{
		return;
	}
	a320_reset(Sim, Scenario, ActiveRunway);
	a320_get_state(Sim, &State);
	a320_get_controls(Sim, &Controls);
	bLsOn = Scenario == A320_SCENARIO_FINAL_10NM || Scenario == A320_SCENARIO_FINAL_4NM;
	UpdateTransform();
}

void AA320Aircraft::UpdateExteriorLights()
{
	const double T = GetWorld()->GetTimeSeconds();
	const int32 L = State.lights;
	BeaconLight->SetVisibility((L & A320_LT_BEACON) && FMath::Fmod(T, 1.0) < 0.12);
	// Strobes: double flash every 1.5 s.
	const double Ph = FMath::Fmod(T, 1.5);
	const bool bStrobe = (L & A320_LT_STROBE) && (Ph < 0.05 || (Ph > 0.12 && Ph < 0.17));
	StrobeLeft->SetVisibility(bStrobe);
	StrobeRight->SetVisibility(bStrobe);
	NavLeft->SetVisibility((L & A320_LT_NAV) != 0);
	NavRight->SetVisibility((L & A320_LT_NAV) != 0);
	LandingLeft->SetVisibility((L & A320_LT_LANDING) != 0);
	LandingRight->SetVisibility((L & A320_LT_LANDING) != 0);
	NoseLight->SetVisibility((L & (A320_LT_TAXI | A320_LT_TAKEOFF)) != 0);
	NoseLight->SetIntensity((L & A320_LT_TAKEOFF) ? 100000.0f : 40000.0f);
}

void AA320Aircraft::SetThrustLevers(double Lever1, bool bReverse1, double Lever2, bool bReverse2, bool bSplit)
{
	Controls.thrustLever = FMath::Clamp(Lever1, 0.0, 1.0);
	Controls.reverse = bReverse1 ? 1 : 0;
	Controls.splitThrust = bSplit ? 1 : 0;
	Controls.thrustLever2 = FMath::Clamp(Lever2, 0.0, 1.0);
	Controls.reverse2 = bReverse2 ? 1 : 0;
}

void AA320Aircraft::SetLever(EA320Lever Lever, double Position)
{
	Position = FMath::Clamp(Position, 0.0, 1.0);
	switch (Lever)
	{
	case EA320Lever::Thrust:
	{
		Controls.splitThrust = 0;
		// Top 72 % of the slot is forward thrust (TOGA at the top), the rest reverse.
		constexpr double ForwardSpan = 0.72;
		if (Position <= ForwardSpan || !State.onGround)
		{
			double Lev = 1.0 - FMath::Min(Position, ForwardSpan) / ForwardSpan;
			for (const double Detent : {1.0, 0.88, 0.75, 0.0})
			{
				if (FMath::Abs(Lev - Detent) < 0.035)
				{
					Lev = Detent;
				}
			}
			Controls.reverse = 0;
			Controls.thrustLever = Lev;
		}
		else
		{
			Controls.reverse = 1;
			Controls.thrustLever = (Position - ForwardSpan) / (1.0 - ForwardSpan);
		}
		break;
	}
	case EA320Lever::Flaps:
		Controls.flapsLever = FMath::Clamp(static_cast<int32>(FMath::RoundToInt(Position * 4.0)), 0, 4);
		break;
	case EA320Lever::Speedbrake:
	{
		double Sb = Position;
		for (const double Detent : {0.0, 0.5, 1.0})
		{
			if (FMath::Abs(Sb - Detent) < 0.08)
			{
				Sb = Detent;
			}
		}
		Controls.speedbrake = Sb;
		if (Sb > 0.0)
		{
			Controls.spoilersArmed = 0;  // ARM only exists with the lever at RET
		}
		break;
	}
	case EA320Lever::None:
		break;
	}
}

void AA320Aircraft::AddLook(double YawDeg, double PitchDeg)
{
	LookOffset.Yaw = FMath::Clamp(LookOffset.Yaw + YawDeg, -160.0, 160.0);
	LookOffset.Pitch = FMath::Clamp(LookOffset.Pitch + PitchDeg, -60.0, 60.0);
	ApplyView();
}

void AA320Aircraft::ResetLook()
{
	LookOffset = FRotator::ZeroRotator;
	ApplyView();
}

void AA320Aircraft::ApplyView()
{
	CockpitCamera->SetRelativeRotation(FRotator(CockpitPitchDeg + LookOffset.Pitch, LookOffset.Yaw, 0.0));
	ChaseArm->SetRelativeRotation(FRotator(-12.0 + LookOffset.Pitch, LookOffset.Yaw, 0.0));
	CockpitCamera->SetActive(bCockpitView);
	ChaseCamera->SetActive(!bCockpitView);
	// From the captain's seat the primitive fuselage would only block the view.
	for (UStaticMeshComponent* Part : ModelParts)
	{
		Part->SetVisibility(!bCockpitView);
	}
}

UStaticMeshComponent* AA320Aircraft::AddPart(UStaticMesh* Mesh, const FVector& CentreM, const FRotator& Rotation,
	const FVector& SizeM, const FLinearColor& Color)
{
	UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
	Part->SetMobility(EComponentMobility::Movable);
	Part->SetStaticMesh(Mesh);
	Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Part->SetupAttachment(Root);
	Part->SetRelativeLocation(CentreM * 100.0);
	Part->SetRelativeRotation(Rotation);
	Part->SetRelativeScale3D(SizeM);  // basic shapes are 1 m, so scale = size in metres
	Part->SetMaterial(0, Shapes.Tint(this, Color));
	Part->RegisterComponent();
	ModelParts.Add(Part);
	return Part;
}

void AA320Aircraft::BuildModel()
{
	// Rough A320 proportions around the CG (metres; x forward, y right, z up). The gear
	// touches the ground 2.57 m below the CG, matching the flight model at rest.
	const FLinearColor BodyWhite(0.85f, 0.87f, 0.9f);
	const FLinearColor BodyGrey(0.35f, 0.37f, 0.4f);
	const FLinearColor BodyDark(0.05f, 0.05f, 0.06f);
	const FLinearColor TailBlue(0.05f, 0.15f, 0.45f);
	const FRotator AlongX(90.0, 0.0, 0.0);  // cylinder axis Z -> X

	// Fuselage, nose and tail cone.
	AddPart(Shapes.Cylinder, FVector(0.5, 0.0, 0.9), AlongX, FVector(3.95, 3.95, 27.0), BodyWhite);
	AddPart(Shapes.Sphere, FVector(14.0, 0.0, 0.75), FRotator::ZeroRotator, FVector(7.0, 3.95, 3.6), BodyWhite);
	AddPart(Shapes.Cylinder, FVector(16.6, 0.0, 1.15), AlongX, FVector(0.2, 2.6, 0.2), BodyDark);  // windscreen band
	AddPart(Shapes.Sphere, FVector(-15.5, 0.0, 1.5), FRotator(-6.0, 0.0, 0.0), FVector(10.0, 3.2, 2.6), BodyWhite);

	// Wings (25 degree sweep, slight dihedral) and engines under them.
	for (const double Side : {-1.0, 1.0})
	{
		// Positive yaw sweeps the right wing aft; negative roll raises the right tip.
		AddPart(Shapes.Cube, FVector(-1.6, Side * 8.8, -0.5), FRotator(0.0, Side * 25.0, Side * -5.0),
			FVector(4.2, 15.5, 0.35), BodyGrey);
		AddPart(Shapes.Cylinder, FVector(2.2, Side * 5.1, -1.2), AlongX, FVector(2.1, 2.1, 4.2), BodyGrey);
		AddPart(Shapes.Cylinder, FVector(4.35, Side * 5.1, -1.2), AlongX, FVector(1.8, 1.8, 0.1), BodyDark);
		// Horizontal stabiliser.
		AddPart(Shapes.Cube, FVector(-17.0, Side * 3.2, 1.6), FRotator(0.0, Side * 30.0, 0.0),
			FVector(2.6, 6.0, 0.25), BodyGrey);
		// Main gear.
		AddPart(Shapes.Cylinder, FVector(-0.4, Side * 3.67, -1.6), FRotator::ZeroRotator, FVector(0.25, 0.25, 2.0), BodyGrey);
		AddPart(Shapes.Cylinder, FVector(-0.4, Side * 3.67, -2.0), FRotator(0.0, 0.0, 90.0), FVector(1.15, 1.15, 0.9), BodyDark);
	}
	// Fin.
	AddPart(Shapes.Cube, FVector(-16.0, 0.0, 5.3), FRotator(35.0, 0.0, 0.0), FVector(5.5, 0.35, 6.2), TailBlue);
	// Nose gear.
	AddPart(Shapes.Cylinder, FVector(12.1, 0.0, -1.5), FRotator::ZeroRotator, FVector(0.2, 0.2, 2.0), BodyGrey);
	AddPart(Shapes.Cylinder, FVector(12.1, 0.0, -2.2), FRotator(0.0, 0.0, 90.0), FVector(0.75, 0.75, 0.5), BodyDark);
}
