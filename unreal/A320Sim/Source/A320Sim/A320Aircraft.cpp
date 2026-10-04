#include "A320Aircraft.h"

#include "A320Fx.h"
#include "A320Sim.h"
#include "A320Voice.h"
#include "a320/RadioTuning.h"
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
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"

namespace
{
	// JSBSim EYEPOINT relative to the CG (A320.xml: x aft, z up, inches): captain's seat.
	const FVector CockpitEyeCm(1504.0, -76.0, 279.0);
	const double CockpitPitchDeg = -5.0;
	const float CockpitFovDeg = 90.0f;
	const FVector ChaseOffsetCm(0.0, 0.0, -1200.0);
	constexpr float ChaseArmCm = 7000.0f;

	// A piece of a destroyed aircraft, in the flat world (cm).
	FVector SectionLocation(const A320Section& S)
	{
		return FVector(S.northM * 100.0, S.eastM * 100.0, S.heightAboveFieldM * 100.0);
	}
	FRotator SectionRotation(const A320Section& S)
	{
		return FRotator(S.pitchDeg, S.gridHeadingDeg, S.bankDeg);
	}
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
	RearRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RearRoot"));
	RearRoot->SetupAttachment(Root);
	RearRoot->SetMobility(EComponentMobility::Movable);

	CockpitCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("CockpitCamera"));
	CockpitCamera->SetupAttachment(Root);
	CockpitCamera->SetRelativeLocation(CockpitEyeCm);
	CockpitCamera->SetRelativeRotation(FRotator(CockpitPitchDeg, 0.0, 0.0));
	CockpitCamera->SetFieldOfView(CockpitFovDeg);

	ChaseArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("ChaseArm"));
	ChaseArm->SetupAttachment(Root);
	ChaseArm->TargetArmLength = ChaseArmCm;
	ChaseArm->SetRelativeRotation(FRotator(-12.0, 0.0, 0.0));
	ChaseArm->TargetOffset = ChaseOffsetCm;
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
	// The real terrain under the flight model, so the radio altimeter and crashes follow it.
	const FString TerrainDir = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Terrain")));
	if (!a320_load_ground(Sim, TCHAR_TO_UTF8(*TerrainDir)))
	{
		UE_LOG(LogA320, Warning, TEXT("No ground map in %s: the ground is each airport's elevation"), *TerrainDir);
	}
	for (int32 i = 0; i < a320_airport_count(Sim); ++i)
	{
		A320AirportInfo Info;
		if (a320_get_airport(Sim, i, &Info))
		{
			Airports.Add(Info);
		}
	}
	for (int32 i = 0; i < a320_runway_count(Sim); ++i)
	{
		A320RunwayInfo Info;
		if (a320_get_runway(Sim, i, &Info))
		{
			Runways.Add(Info);
		}
	}
	{
		const int32 Tallinn26 = FindRunway(TEXT("EETN"), TEXT("26"));
		if (Tallinn26 >= 0)
		{
			DepRunway = ArrRunway = Tallinn26;
		}
	}

	FActorSpawnParameters Params;
	Params.Owner = this;
	World = GetWorld()->SpawnActor<AA320World>(AA320World::StaticClass(), FTransform::Identity, Params);
	if (World)
	{
		World->Build(Runways, Airports);
	}

	ResetScenario(A320_SCENARIO_RUNWAY);
	ApplyView();
	StartAudio();
	Voice = MakeShared<FA320Voice>();
	UE_LOG(LogA320, Log, TEXT("Flight model ready: %d airports, %d runways, lined up on %s"), Airports.Num(), Runways.Num(),
		Runways.IsValidIndex(DepRunway) ? UTF8_TO_TCHAR(Runways[DepRunway].ident) : TEXT("?"));
}

void AA320Aircraft::EndPlay(const EEndPlayReason::Type Reason)
{
	Voice.Reset();  // stops the speech thread
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
	Controls.efisLs = bLsOn ? 1 : 0;
	Controls.ndMode = NdMode;
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
	UpdateDestruction();
	UpdateExteriorLights();
	PumpRadio();
	if (World)
	{
		for (int32 i = 0; i < Runways.Num() && i < 4; ++i)
		{
			World->UpdatePapi(i, State.papiRunway[i]);
		}
	}
	PumpAudio(DeltaSeconds);
}

void AA320Aircraft::PumpRadio()
{
	// New transmissions: those heard on COM 1 go to the log and the voice; ATC's become the subtitle.
	for (uint32 Seq = LastAtcSeq + 1; Seq <= State.atcMessageSeq; ++Seq)
	{
		A320AtcMessage Message;
		if (!a320_atc_message(Sim, Seq, &Message) || !Message.heard)
		{
			continue;
		}
		RadioLog.Add(Message);
		if (RadioLog.Num() > 40)
		{
			RadioLog.RemoveAt(0);
		}
		if (Voice && bSoundOn)
		{
			Voice->Speak(UTF8_TO_TCHAR(Message.speech), Message.speaker, Message.frequencyKhz);
		}
		if (Message.speaker == A320_ATC_SPEAKER_ATC)
		{
			AtcSubtitle = FString::Printf(TEXT("%s: %s"), UTF8_TO_TCHAR(Message.station), UTF8_TO_TCHAR(Message.text));
			AtcSubtitleAt = FPlatformTime::Seconds();
		}
	}
	LastAtcSeq = State.atcMessageSeq;
	if (Voice)
	{
		TArray<int16> Pcm;
		int32 Khz = 0;
		while (Voice->PopClip(Pcm, Khz))
		{
			a320_audio_radio_clip(Sim, Pcm.GetData(), Pcm.Num(), FA320Voice::SampleRate, Khz);
		}
	}
}

A320AtcStatus AA320Aircraft::GetAtcStatus() const
{
	A320AtcStatus Status;
	FMemory::Memzero(Status);
	if (Sim)
	{
		a320_atc_get_status(Sim, &Status);
	}
	return Status;
}

double AA320Aircraft::GetFieldElevationFt() const
{
	// The nearest airport's: the flight model's ground there.
	const double AboveFirstM = Airports.IsValidIndex(State.nearestAirport) ? Airports[State.nearestAirport].elevationM : 0.0;
	return FieldElevationFt + AboveFirstM / 0.3048;
}

const FString& AA320Aircraft::GetAtcSubtitle(double& OutAgeSeconds) const
{
	OutAgeSeconds = FPlatformTime::Seconds() - AtcSubtitleAt;
	return AtcSubtitle;
}

bool AA320Aircraft::HasVoices() const
{
	return Voice && Voice->GetVoiceCount() != 0;
}

void AA320Aircraft::McduKey(int32 Key)
{
	if (Sim)
	{
		a320_audio_event(Sim, A320_SOUND_CLICK);
		a320_mcdu_key(Sim, Key);
		a320_get_state(Sim, &State);  // the PFD and ND show a new ILS or minimums at once
	}
}

void AA320Aircraft::GetMcduDisplay(A320McduDisplay& Out) const
{
	if (Sim)
	{
		a320_mcdu_get_display(Sim, &Out);
	}
	else
	{
		FMemory::Memzero(Out);
	}
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

void AA320Aircraft::AdjustVs(int32 Clicks)
{
	// A click of the V/S knob is 100 ft/min, or 0.1 degree in TRK-FPA.
	if (!Sim)
	{
		return;
	}
	if (State.fcuTrkFpa)
	{
		a320_fcu_set_fpa(Sim, State.fcuFpaDeg + 0.1 * Clicks);
		a320_get_state(Sim, &State);
	}
	else
	{
		AdjustFcu(0.0, 0.0, 0.0, 100.0 * Clicks);
	}
}

void AA320Aircraft::UpdateTransform()
{
	// Flat world: X north, Y east, Z up from field elevation, in centimetres. FRotator's
	// pitch/yaw/roll match JSBSim's theta/psi/phi signs (nose up, clockwise, right wing down).
	if (State.destroyed != A320_DESTROYED_NONE)
	{
		// The nose section carries the pawn (and the cameras); the rest flies on its own.
		const A320Section& Front = State.sections[0];
		SetActorLocationAndRotation(SectionLocation(Front), SectionRotation(Front));
		if (State.destroyed == A320_DESTROYED_BREAKUP)
		{
			RearRoot->SetWorldLocationAndRotation(SectionLocation(State.sections[1]), SectionRotation(State.sections[1]));
		}
		return;
	}
	const FVector Location(State.northM * 100.0, State.eastM * 100.0, State.heightAboveFieldM * 100.0);
	// The flat world's north is true north at EETN only: yaw by the grid heading.
	const FRotator Rotation(State.pitchDeg, State.gridHeadingDeg, State.bankDeg);
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

void AA320Aircraft::ExecuteCommand(EA320Command Command, bool bLarge, int32 Param)
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
	case EA320Command::FcuAp2: Fcu(A320_FCU_AP2); break;
	case EA320Command::FcuAthr: Fcu(A320_FCU_ATHR); break;
	// Shift (keyboard or click) pushes a knob instead of pulling it, as the joystick's bLarge.
	case EA320Command::FcuHdgPull: Fcu(bLarge ? A320_FCU_HDG_PUSH : A320_FCU_HDG_PULL); break;
	case EA320Command::FcuHdgPush: Fcu(A320_FCU_HDG_PUSH); break;
	case EA320Command::FcuTrkFpa: Fcu(A320_FCU_TRK_FPA); break;
	case EA320Command::FcuLoc: Fcu(A320_FCU_LOC); break;
	case EA320Command::FcuAppr:
		Fcu(A320_FCU_APPR);
		bLsOn = bLsOn || (State.armed & A320_ARMED_GS) != 0;
		break;
	case EA320Command::ResetColdDark: ResetScenario(A320_SCENARIO_COLD_DARK); break;
	case EA320Command::ApDisconnect:
		// Both autopilots off (the pushbuttons toggle; Fcu refreshes State in between).
		if (State.ap1Engaged)
		{
			Fcu(A320_FCU_AP1);
		}
		if (State.ap2Engaged)
		{
			Fcu(A320_FCU_AP2);
		}
		break;
	case EA320Command::OverheadToggle: bOverheadVisible = !bOverheadVisible; break;
	case EA320Command::McduToggle: bMcduVisible = !bMcduVisible; break;
	case EA320Command::RadioToggle: bRadioVisible = !bRadioVisible; break;
	case EA320Command::ComSwap: Swap(Controls.com1ActiveKhz, Controls.com1StandbyKhz); break;
	case EA320Command::ComMhzDec: Controls.com1StandbyKhz = a320::radio::stepMhz(Controls.com1StandbyKhz, -1); break;
	case EA320Command::ComMhzInc: Controls.com1StandbyKhz = a320::radio::stepMhz(Controls.com1StandbyKhz, 1); break;
	case EA320Command::ComKhzDec: Controls.com1StandbyKhz = a320::radio::stepKhz(Controls.com1StandbyKhz, -1); break;
	case EA320Command::ComKhzInc: Controls.com1StandbyKhz = a320::radio::stepKhz(Controls.com1StandbyKhz, 1); break;
	case EA320Command::XpdrDigit:
		// The code changes once all four digits are in, as on the ATC panel.
		if (Param >= 0 && Param <= 7)
		{
			XpdrEntry.AppendChar(static_cast<TCHAR>(TEXT('0') + Param));
			if (XpdrEntry.Len() == 4)
			{
				Controls.xpdrCode = FCString::Atoi(*XpdrEntry);
				XpdrEntry.Reset();
			}
		}
		break;
	case EA320Command::XpdrClear: XpdrEntry.Reset(); break;
	case EA320Command::XpdrMode: Controls.xpdrMode = FMath::Clamp(Param, A320_XPDR_STBY, A320_XPDR_ON); break;
	case EA320Command::AtcToggle:
		if (Sim)
		{
			a320_atc_set_enabled(Sim, State.atcEnabled ? 0 : 1);
		}
		break;
	case EA320Command::AtcReply:
		if (Sim)
		{
			a320_atc_choose(Sim, Param);
			a320_get_state(Sim, &State);
			PumpRadio();  // the crew's call shows and speaks at once
		}
		break;
	case EA320Command::NdModeToggle:
		// EFIS mode selector: ARC -> ROSE NAV -> ROSE LS.
		NdMode = NdMode == A320_ND_ARC ? A320_ND_ROSE_NAV : (NdMode == A320_ND_ROSE_NAV ? A320_ND_ROSE_LS : A320_ND_ARC);
		break;
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
	case EA320Command::FcuAltPull: Fcu(bLarge ? A320_FCU_ALT_PUSH : A320_FCU_ALT_PULL); break;
	case EA320Command::FcuAltPush: Fcu(A320_FCU_ALT_PUSH); break;
	case EA320Command::FcuVsPull: Fcu(bLarge ? A320_FCU_VS_PUSH : A320_FCU_VS_PULL); break;
	case EA320Command::SpdDec: AdjustFcu(-Step, 0.0, 0.0, 0.0); break;
	case EA320Command::SpdInc: AdjustFcu(Step, 0.0, 0.0, 0.0); break;
	case EA320Command::HdgDec: AdjustFcu(0.0, -Step, 0.0, 0.0); break;
	case EA320Command::HdgInc: AdjustFcu(0.0, Step, 0.0, 0.0); break;
	case EA320Command::AltDec: AdjustFcu(0.0, 0.0, -100.0 * Step, 0.0); break;
	case EA320Command::AltInc: AdjustFcu(0.0, 0.0, 100.0 * Step, 0.0); break;
	case EA320Command::VsDec: AdjustVs(bLarge ? -5 : -1); break;
	case EA320Command::VsInc: AdjustVs(bLarge ? 5 : 1); break;
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
	case EA320Command::NdRangeUp: NdRangeNm = FMath::Min(NdRangeNm * 2, 320); break;
	case EA320Command::FcuVsPush: Fcu(A320_FCU_VS_PUSH); break;
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
		// The other direction at the same airport; a local flight keeps landing where it departs.
		for (int32 i = 1; i < Runways.Num(); ++i)
		{
			const int32 Next = (DepRunway + i) % Runways.Num();
			if (Runways[Next].airport == Runways[DepRunway].airport)
			{
				ArrRunway = ArrRunway == DepRunway ? Next : ArrRunway;
				DepRunway = Next;
				break;
			}
		}
		ResetScenario(A320_SCENARIO_RUNWAY);
		break;
	case EA320Command::FlightMenu:
		bFlightMenu = !bFlightMenu;
		bMapVisible = bMapVisible && !bFlightMenu;
		bGuideMenu = bGuideMenu && !bFlightMenu;
		break;
	case EA320Command::FlightDep:
		if (Runways.IsValidIndex(Param))
		{
			ArrRunway = ArrRunway == DepRunway ? Param : ArrRunway;
			DepRunway = Param;
		}
		break;
	case EA320Command::FlightArr:
		if (Runways.IsValidIndex(Param))
		{
			ArrRunway = Param;
			PlaceDestination = FA320Destination{};
		}
		break;
	case EA320Command::FlightStart: FlightScenario = static_cast<A320Scenario>(Param); break;
	case EA320Command::FlightDistance: FlightDistanceNm = FMath::Clamp(Param, 8, 150); break;
	case EA320Command::FlightPlan: FlightPlan = Param == A320_PLAN_EMPTY ? A320_PLAN_EMPTY : A320_PLAN_FULL; break;
	case EA320Command::FlightDepAirport: SetDepartureAirport(Param); break;
	case EA320Command::FlightArrAirport: SetArrivalAirport(Param); break;
	case EA320Command::MapToggle:
		bMapVisible = !bMapVisible;
		bFlightMenu = bFlightMenu && !bMapVisible;
		bGuideMenu = bGuideMenu && !bMapVisible;
		break;
	case EA320Command::FlightGo:
		bFlightMenu = false;
		bMapVisible = false;
		ResetScenario(FlightScenario);
		break;
	case EA320Command::ResetRunway: ResetScenario(A320_SCENARIO_RUNWAY); break;
	case EA320Command::ResetFinal10: ResetScenario(A320_SCENARIO_FINAL_10NM); break;
	case EA320Command::ResetFinal4: ResetScenario(A320_SCENARIO_FINAL_4NM); break;
	case EA320Command::ResetApproach: ResetScenario(A320_SCENARIO_APPROACH); break;
	case EA320Command::GuideMenu:
		bGuideMenu = !bGuideMenu;
		bFlightMenu = bFlightMenu && !bGuideMenu;
		bMapVisible = bMapVisible && !bGuideMenu;
		break;
	case EA320Command::GuideStart0:
	case EA320Command::GuideStart1:
	case EA320Command::GuideStart2:
	case EA320Command::GuideStart3:
		StartGuide(static_cast<int32>(Command) - static_cast<int32>(EA320Command::GuideStart0));
		break;
	case EA320Command::GuideNext:
		if (Sim)
		{
			a320_guide_next(Sim);
		}
		break;
	case EA320Command::GuideBack:
		if (Sim)
		{
			a320_guide_back(Sim);
		}
		break;
	case EA320Command::GuideStop:
		if (Sim)
		{
			a320_guide_stop(Sim);
		}
		break;
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
	a320_guide_stop(Sim);  // a lesson restarts its own scenario
	FlightScenario = Scenario;
	a320_start_flight(Sim, Scenario, DepRunway, ArrRunway, FlightDistanceNm, bLessonStart ? A320_PLAN_ROUTE : FlightPlan);
	// To a place rather than an airport: a free (VFR) flight, without ATC; RADIO can switch it on.
	a320_atc_set_enabled(Sim, PlaceDestination.bSet ? 0 : 1);
	a320_get_state(Sim, &State);
	a320_get_controls(Sim, &Controls);
	ClearDestruction();
	LastAtcSeq = State.atcMessageSeq;  // a new flight starts a new radio log
	if (Voice)
	{
		Voice->Flush();
	}
	RadioLog.Reset();
	XpdrEntry.Reset();
	AtcSubtitle.Reset();
	bLsOn = Scenario == A320_SCENARIO_FINAL_10NM || Scenario == A320_SCENARIO_FINAL_4NM;
	NdMode = A320_ND_ARC;
	UpdateTransform();
}

void AA320Aircraft::StartGuide(int32 Guide)
{
	if (!Sim || Guide < 0 || Guide >= a320_guide_count())
	{
		return;
	}
	// The lesson's texts are written for one runway, a local flight from 20 NM out.
	const FString Ident = UTF8_TO_TCHAR(a320_guide_runway(Guide));
	const int32 LessonRunway = FindRunway(TEXT("EETN"), *Ident);  // the lessons are written for Tallinn
	if (LessonRunway >= 0)
	{
		DepRunway = ArrRunway = LessonRunway;
	}
	PlaceDestination = FA320Destination{};
	FlightDistanceNm = 20;
	// The lessons' steps are written for the route only: the crew enters the rest.
	bLessonStart = true;
	ResetScenario(a320_guide_scenario(Guide));
	bLessonStart = false;
	a320_guide_start(Sim, Guide);
	bGuideMenu = false;
	bFlightMenu = false;
	bHelpVisible = false;
	bOverheadVisible = false;
}

A320GuideStatus AA320Aircraft::GetGuideStatus() const
{
	A320GuideStatus Status{};
	if (Sim)
	{
		a320_guide_get_status(Sim, &Status);
	}
	return Status;
}

FString AA320Aircraft::GetGuideAlert() const
{
	return Sim ? FString(UTF8_TO_TCHAR(a320_guide_alert(Sim))) : FString();
}

void AA320Aircraft::UpdateExteriorLights()
{
	const double T = GetWorld()->GetTimeSeconds();
	const int32 L = State.destroyed != A320_DESTROYED_NONE ? 0 : State.lights;  // nothing left to power them
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

void AA320Aircraft::SetSwitch(EA320Switch Switch, int32 Value)
{
	switch (Switch)
	{
	case EA320Switch::Gear: Controls.gearDown = Value ? 1 : 0; break;
	case EA320Switch::ParkBrake: Controls.parkBrake = Value ? 1 : 0; break;
	case EA320Switch::SpoilersArm:
		Controls.spoilersArmed = Value ? 1 : 0;
		if (Value)
		{
			Controls.speedbrake = 0.0;
		}
		break;
	case EA320Switch::Autobrake: Controls.autobrake = FMath::Clamp(Value, A320_AUTOBRAKE_OFF, A320_AUTOBRAKE_MAX); break;
	case EA320Switch::EngMaster1: Controls.engMaster[0] = Value ? 1 : 0; break;
	case EA320Switch::EngMaster2: Controls.engMaster[1] = Value ? 1 : 0; break;
	case EA320Switch::EngMode: Controls.engMode = FMath::Clamp(Value, A320_ENG_MODE_CRANK, A320_ENG_MODE_IGN_START); break;
	case EA320Switch::Flaps: Controls.flapsLever = FMath::Clamp(Value, 0, 4); break;
	case EA320Switch::NdMode: NdMode = FMath::Clamp(Value, A320_ND_ARC, A320_ND_ROSE_LS); break;
	case EA320Switch::NdRange: NdRangeNm = FMath::Clamp(Value, 5, 320); break;
	}
}

void AA320Aircraft::SetSpeedbrake(double Amount)
{
	Controls.speedbrake = FMath::Clamp(Amount, 0.0, 1.0);
	if (Amount > 0.05)
	{
		Controls.spoilersArmed = 0;  // the lever left the ARM position
	}
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
	if (State.destroyed == A320_DESTROYED_NONE)
	{
		ChaseArm->SetRelativeRotation(FRotator(-12.0 + LookOffset.Pitch, LookOffset.Yaw, 0.0));
	}
	else
	{
		// The arm is absolute then (see SteadyChaseView): looking around still works.
		ChaseArm->SetWorldRotation(FRotator(-12.0 + LookOffset.Pitch, WreckViewYawDeg + LookOffset.Yaw, 0.0));
	}
	CockpitCamera->SetActive(bCockpitView);
	ChaseCamera->SetActive(!bCockpitView);
	// From the captain's seat the primitive fuselage would only block the view.
	// After a crash only the fire and the debris are left.
	for (UStaticMeshComponent* Part : ModelParts)
	{
		Part->SetVisibility(!bCockpitView && State.destroyed != A320_DESTROYED_CRASH);
	}
}

UStaticMeshComponent* AA320Aircraft::AddPart(UStaticMesh* Mesh, const FVector& CentreM, const FRotator& Rotation,
	const FVector& SizeM, const FLinearColor& Color, USceneComponent* Parent)
{
	UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
	Part->SetMobility(EComponentMobility::Movable);
	Part->SetStaticMesh(Mesh);
	Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Part->SetupAttachment(Parent);
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
	// Livery in the style of airBaltic's (a white aircraft, the colour on the tail, engines and
	// wingtips), in the colours of the Estonian flag: blue (#0072CE), black and white.
	const FLinearColor BodyWhite(0.85f, 0.87f, 0.9f);
	const FLinearColor BodyGrey(0.35f, 0.37f, 0.4f);
	const FLinearColor BodyDark(0.05f, 0.05f, 0.06f);
	const FLinearColor FlagBlue = FLinearColor::FromSRGBColor(FColor(0, 114, 206));
	const FLinearColor FlagBlack(0.012f, 0.012f, 0.014f);
	const FLinearColor FlagWhite(0.92f, 0.93f, 0.95f);
	const FRotator AlongX(90.0, 0.0, 0.0);  // cylinder axis Z -> X

	// The nose section (Root) and the rest (RearRoot) meet where the fuselage breaks in a breakup.
	USceneComponent* Nose = Root;
	USceneComponent* Rest = RearRoot;
	const double Split = A320_BREAKUP_SPLIT_X_M, FuselageAft = -13.0, FuselageFwd = 14.0;

	// Fuselage (in two lengths, nose and rest), nose and tail cone.
	AddPart(Shapes.Cylinder, FVector((Split + FuselageFwd) / 2.0, 0.0, 0.9), AlongX, FVector(3.95, 3.95, FuselageFwd - Split), BodyWhite, Nose);
	AddPart(Shapes.Cylinder, FVector((FuselageAft + Split) / 2.0, 0.0, 0.9), AlongX, FVector(3.95, 3.95, Split - FuselageAft), BodyWhite, Rest);
	AddPart(Shapes.Sphere, FVector(14.0, 0.0, 0.75), FRotator::ZeroRotator, FVector(7.0, 3.95, 3.6), BodyWhite, Nose);
	AddPart(Shapes.Cylinder, FVector(16.6, 0.0, 1.15), AlongX, FVector(0.2, 2.6, 0.2), BodyDark, Nose);  // windscreen band
	AddPart(Shapes.Sphere, FVector(-15.5, 0.0, 1.5), FRotator(-6.0, 0.0, 0.0), FVector(10.0, 3.2, 2.6), BodyWhite, Rest);

	// Wings (25 degree sweep, slight dihedral) and engines under them.
	for (const double Side : {-1.0, 1.0})
	{
		// Positive yaw sweeps the right wing aft; negative roll raises the right tip.
		AddPart(Shapes.Cube, FVector(-1.6, Side * 8.8, -0.5), FRotator(0.0, Side * 25.0, Side * -5.0),
			FVector(4.2, 15.5, 0.35), BodyGrey, Rest);
		AddPart(Shapes.Cylinder, FVector(2.2, Side * 5.1, -1.2), AlongX, FVector(2.1, 2.1, 4.2), FlagBlue, Rest);  // nacelle
		AddPart(Shapes.Cylinder, FVector(4.35, Side * 5.1, -1.2), AlongX, FVector(1.8, 1.8, 0.1), BodyDark, Rest);
		// Sharklet at the wing tip (15.8 m out, swept with the wing), in the tail's blue.
		AddPart(Shapes.Cube, FVector(-5.1, Side * 15.85, 1.25), FRotator(0.0, Side * 25.0, Side * 8.0), FVector(1.5, 0.15, 2.2), FlagBlue, Rest);
		// The Estonian flag by the forward door, both sides, on a thin grey border.
		AddPart(Shapes.Cube, FVector(9.6, Side * 1.73, 1.9), FRotator::ZeroRotator, FVector(1.2, 0.02, 0.76), BodyGrey, Nose);
		const FLinearColor Bands[3] = {FlagBlue, FlagBlack, FlagWhite};
		for (int32 b = 0; b < 3; ++b)
		{
			AddPart(Shapes.Cube, FVector(9.6, Side * 1.745, 2.12 - b * 0.22), FRotator::ZeroRotator, FVector(1.1, 0.02, 0.22), Bands[b], Nose);
		}
		// Horizontal stabiliser.
		AddPart(Shapes.Cube, FVector(-17.0, Side * 3.2, 1.6), FRotator(0.0, Side * 30.0, 0.0),
			FVector(2.6, 6.0, 0.25), BodyGrey, Rest);
		// Main gear.
		AddPart(Shapes.Cylinder, FVector(-0.4, Side * 3.67, -1.6), FRotator::ZeroRotator, FVector(0.25, 0.25, 2.0), BodyGrey, Rest);
		AddPart(Shapes.Cylinder, FVector(-0.4, Side * 3.67, -2.0), FRotator(0.0, 0.0, 90.0), FVector(1.15, 1.15, 0.9), BodyDark, Rest);
	}
	// Fin, swept 35 degrees, in the flag's horizontal bands (blue, black, white from the top): stacked
	// level slices, each a little further aft, so the bands stay horizontal on the swept shape.
	{
		const double FinCentreX = -16.0, FinCentreZ = 5.3, FinHeight = 6.2 * FMath::Cos(FMath::DegreesToRadians(35.0));
		const double Sweep = FMath::Tan(FMath::DegreesToRadians(35.0)), Chord = 5.5 / FMath::Cos(FMath::DegreesToRadians(35.0));
		constexpr int32 Slices = 12;
		const double SliceH = FinHeight / Slices;
		for (int32 k = 0; k < Slices; ++k)
		{
			const double Z = FinCentreZ - FinHeight / 2.0 + (k + 0.5) * SliceH;
			const FLinearColor& Band = k < Slices / 3 ? FlagWhite : (k < 2 * Slices / 3 ? FlagBlack : FlagBlue);
			AddPart(Shapes.Cube, FVector(FinCentreX - (Z - FinCentreZ) * Sweep, 0.0, Z), FRotator::ZeroRotator,
				FVector(Chord * 0.82, 0.35, SliceH * 1.02), Band, Rest);
		}
	}
	// Nose gear.
	AddPart(Shapes.Cylinder, FVector(12.1, 0.0, -1.5), FRotator::ZeroRotator, FVector(0.2, 0.2, 2.0), BodyGrey, Nose);
	AddPart(Shapes.Cylinder, FVector(12.1, 0.0, -2.2), FRotator(0.0, 0.0, 90.0), FVector(0.75, 0.75, 0.5), BodyDark, Nose);
}

AA320Fx* AA320Aircraft::SpawnFx(const FVector& LocationCm, const FA320FxSpec& Spec)
{
	FActorSpawnParameters Params;
	Params.Owner = this;
	AA320Fx* Fx = GetWorld()->SpawnActor<AA320Fx>(AA320Fx::StaticClass(), FTransform(LocationCm), Params);
	if (Fx)
	{
		Fx->Start(Spec);
		Effects.Add(Fx);
	}
	return Fx;
}

void AA320Aircraft::SteadyChaseView(double ArmLengthCm)
{
	// The cockpit camera rides the nose section, so from the seat the fall is felt; the chase
	// camera keeps the heading it had and follows the nose without its pitch and roll.
	WreckViewYawDeg = State.sections[0].gridHeadingDeg;
	ChaseArm->SetUsingAbsoluteRotation(true);
	ChaseArm->TargetArmLength = static_cast<float>(ArmLengthCm);
	ApplyView();
}

void AA320Aircraft::UpdateDestruction()
{
	if (State.destroyedSeq != LastDestroyedSeq)
	{
		LastDestroyedSeq = State.destroyedSeq;
		if (State.destroyed == A320_DESTROYED_BREAKUP)
		{
			StartBreakup();
		}
		else if (State.destroyed == A320_DESTROYED_CRASH)
		{
			StartCrash();
		}
	}
	if (State.destroyed != A320_DESTROYED_BREAKUP)
	{
		return;
	}
	// Each piece trails fire and smoke from where it broke, until it hits the ground: then an
	// explosion, a fire and debris there.
	AA320Fx* Trails[2] = {TrailFront.Get(), TrailRear.Get()};
	for (int32 i = 0; i < 2; ++i)
	{
		const A320Section& Piece = State.sections[i];
		const FTransform Pose(SectionRotation(Piece), SectionLocation(Piece));
		const FVector BreakFace = Pose.TransformPosition(FVector(A320_BREAKUP_SPLIT_X_M * 100.0, 0.0, 90.0));
		if (Trails[i])
		{
			Trails[i]->MoveSource(BreakFace);
		}
		if (Piece.impactSeq != LastImpactSeq[i])
		{
			LastImpactSeq[i] = Piece.impactSeq;
			if (Trails[i])
			{
				Trails[i]->Extinguish();
			}
			FA320FxSpec Impact;
			Impact.SizeM = i == 0 ? 20.0 : 28.0;  // the wing tanks hold the fuel
			Impact.DebrisCount = 18;
			Impact.GroundZ = State.groundHeightM * 100.0;
			FVector Centre = Pose.TransformPosition(FVector(i == 0 ? 900.0 : -300.0, 0.0, 0.0));
			Centre.Z = Impact.GroundZ;
			SpawnFx(Centre, Impact);
		}
	}
}

void AA320Aircraft::StartBreakup()
{
	LastImpactSeq[0] = State.sections[0].impactSeq;
	LastImpactSeq[1] = State.sections[1].impactSeq;
	RearRoot->SetUsingAbsoluteLocation(true);
	RearRoot->SetUsingAbsoluteRotation(true);
	UpdateTransform();
	SteadyChaseView(ChaseArmCm);

	const FTransform Pose(SectionRotation(State.sections[0]), SectionLocation(State.sections[0]));
	const FVector BreakPoint = Pose.TransformPosition(FVector(A320_BREAKUP_SPLIT_X_M * 100.0, 0.0, 90.0));
	const FVector Velocity(State.velNorthMps * 100.0, State.velEastMps * 100.0, State.velUpMps * 100.0);
	FA320FxSpec Burst;
	Burst.SizeM = 12.0;
	Burst.bFire = false;
	Burst.bSmoke = false;
	Burst.DebrisCount = 14;
	Burst.DebrisVelocity = Velocity;
	Burst.GroundZ = State.groundHeightM * 100.0;
	SpawnFx(BreakPoint, Burst);
	FA320FxSpec Trail;
	Trail.SizeM = 6.0;
	Trail.bExplosion = false;
	// Bigger, shorter-lived puffs spaced along the fall: a continuous trail from a small pool.
	Trail.PuffScale = 3.5;
	Trail.PuffPool = 130;
	Trail.PuffLifeS = 7.0;
	Trail.GroundZ = Burst.GroundZ;
	TrailFront = SpawnFx(BreakPoint, Trail);
	TrailRear = SpawnFx(BreakPoint, Trail);
}

void AA320Aircraft::StartCrash()
{
	SteadyChaseView(ChaseArmCm * 2.0);  // also hides the aircraft: what is left is fire and debris
	FA320FxSpec Crash;
	Crash.SizeM = 35.0;
	Crash.DebrisCount = 36;
	Crash.DebrisVelocity = FVector(State.velNorthMps * 100.0, State.velEastMps * 100.0, 0.0);
	Crash.GroundZ = State.groundHeightM * 100.0;
	SpawnFx(FVector(GetActorLocation().X, GetActorLocation().Y, Crash.GroundZ), Crash);
}

void AA320Aircraft::ClearDestruction()
{
	for (AA320Fx* Fx : Effects)
	{
		if (IsValid(Fx))
		{
			Fx->Destroy();
		}
	}
	Effects.Reset();
	TrailFront = nullptr;
	TrailRear = nullptr;
	LastDestroyedSeq = State.destroyedSeq;
	LastImpactSeq[0] = State.sections[0].impactSeq;
	LastImpactSeq[1] = State.sections[1].impactSeq;
	// The rest of the aircraft back in place.
	RearRoot->SetUsingAbsoluteLocation(false);
	RearRoot->SetUsingAbsoluteRotation(false);
	RearRoot->SetRelativeLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator);
	ChaseArm->SetUsingAbsoluteRotation(false);
	ChaseArm->TargetArmLength = ChaseArmCm;
	ChaseArm->TargetOffset = ChaseOffsetCm;
	ApplyView();
}

int32 AA320Aircraft::FindRunway(const TCHAR* Icao, const TCHAR* Ident) const
{
	for (int32 i = 0; i < Runways.Num(); ++i)
	{
		if (FCString::Strcmp(UTF8_TO_TCHAR(Runways[i].icao), Icao) == 0 && FCString::Strcmp(UTF8_TO_TCHAR(Runways[i].ident), Ident) == 0)
		{
			return i;
		}
	}
	return -1;
}

int32 AA320Aircraft::BestRunway(int32 Airport) const
{
	int32 First = -1;
	for (int32 i = 0; i < Runways.Num(); ++i)
	{
		if (Runways[i].airport != Airport)
		{
			continue;
		}
		if (Runways[i].hasIls)
		{
			return i;
		}
		First = First < 0 ? i : First;
	}
	return First;
}

void AA320Aircraft::SetDepartureAirport(int32 Airport)
{
	const int32 Runway = BestRunway(Airport);
	if (Runway < 0)
	{
		return;
	}
	// A local flight stays local; a trip elsewhere keeps its destination.
	ArrRunway = ArrRunway == DepRunway ? Runway : ArrRunway;
	DepRunway = Runway;
}

void AA320Aircraft::SetArrivalAirport(int32 Airport)
{
	const int32 Runway = BestRunway(Airport);
	if (Runway >= 0)
	{
		ArrRunway = Runway;
		PlaceDestination = FA320Destination{};
	}
}

void AA320Aircraft::SetLocalFlight()
{
	PlaceDestination = FA320Destination{};
	ArrRunway = DepRunway;
}

void AA320Aircraft::SetPlaceDestination(const FA320Destination& Place)
{
	// The flight plan and the airborne starts use the departure airport; the map, the ND and the
	// route show the place.
	PlaceDestination = Place;
	ArrRunway = DepRunway;
}
