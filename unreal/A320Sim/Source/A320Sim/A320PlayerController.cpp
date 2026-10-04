#include "A320PlayerController.h"

#include "A320Aircraft.h"
#include "A320Commands.h"
#include "A320Hud.h"
#include "Engine/World.h"
#include "InputCoreTypes.h"
#include "Misc/Paths.h"

namespace
{
	struct FKeyCommand
	{
		FKey Key;
		EA320Command Command;
	};

	double MoveTowards(double Current, double Target, double MaxDelta)
	{
		return Current + FMath::Clamp(Target - Current, -MaxDelta, MaxDelta);
	}

	double Deadzone(double V, double Zone = 0.12)
	{
		return FMath::Abs(V) < Zone ? 0.0 : (V - FMath::Sign(V) * Zone) / (1.0 - Zone);
	}
}

AA320PlayerController::AA320PlayerController()
{
	bShowMouseCursor = true;
}

void AA320PlayerController::BeginPlay()
{
	Super::BeginPlay();
	FInputModeGameAndUI Mode;
	Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
	Mode.SetHideCursorDuringCapture(false);
	SetInputMode(Mode);
	Joystick.Init(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("A320Joystick.ini")));
}

// Per-function joystick commands are indexed by a320::joy::Function.
static_assert(static_cast<int32>(EA320Command::JoyAxis0) - static_cast<int32>(EA320Command::JoyLearn0) == a320::joy::kFunctionCount &&
	static_cast<int32>(EA320Command::JoyInvert0) - static_cast<int32>(EA320Command::JoyAxis0) == a320::joy::kFunctionCount &&
	static_cast<int32>(EA320Command::JoyCalStart) - static_cast<int32>(EA320Command::JoyInvert0) == a320::joy::kFunctionCount,
	"EA320Command joystick blocks must match kFunctionCount");

bool AA320PlayerController::HandleJoystickCommand(EA320Command Command)
{
	const int32 Learn = static_cast<int32>(Command) - static_cast<int32>(EA320Command::JoyLearn0);
	const int32 Axis = static_cast<int32>(Command) - static_cast<int32>(EA320Command::JoyAxis0);
	const int32 Invert = static_cast<int32>(Command) - static_cast<int32>(EA320Command::JoyInvert0);
	if (Command == EA320Command::JoystickPanel)
	{
		bJoystickPanel = !bJoystickPanel;
	}
	else if (Command == EA320Command::JoyRescan)
	{
		Joystick.Rescan();
	}
	else if (Command == EA320Command::JoyCalStart)
	{
		Joystick.StartCalibration();
	}
	else if (Command == EA320Command::JoyCalSet)
	{
		Joystick.CalibrationSet();
	}
	else if (Command == EA320Command::JoyCalSkip)
	{
		Joystick.CalibrationSkip();
	}
	else if (Command == EA320Command::JoyCalCancel)
	{
		Joystick.CancelCalibration();
	}
	else if (Learn >= 0 && Learn < a320::joy::kFunctionCount)
	{
		Joystick.StartLearn(Learn);
	}
	else if (Axis >= 0 && Axis < a320::joy::kFunctionCount)
	{
		Joystick.CycleAxis(Axis);
	}
	else if (Invert >= 0 && Invert < a320::joy::kFunctionCount)
	{
		Joystick.ToggleInvert(Invert);
	}
	else
	{
		return false;
	}
	return true;
}

void AA320PlayerController::ApplyJoystickButtons(AA320Aircraft* Aircraft, FA320FlightInputs& Inputs, float DeltaTime)
{
	using namespace a320::joy;
	Inputs.StickPitch += Joystick.Value(kPitch);
	Inputs.StickRoll += Joystick.Value(kRoll);
	Inputs.Pedals += Joystick.Value(kRudder);
	Inputs.Brakes = FMath::Max3(Inputs.Brakes, brakeAmount(Joystick.Value(kBrakeLeft)), brakeAmount(Joystick.Value(kBrakeRight)));
	if (Joystick.ThrottleMoved())
	{
		// A lever calibrated without a reverse range keeps the REVERSE key/button's selection
		// and then sets the reverse amount, as before.
		const A320Controls& Ctl = Aircraft->GetSimControls();
		const LeverPosition L1 = Joystick.Lever(0), L2 = Joystick.Lever(1);
		const bool bRev1 = Joystick.GetConfig().cal[0].hasReverse ? L1.reverse : Ctl.reverse != 0;
		const bool bRev2 = Joystick.GetConfig().cal[1].hasReverse ? L2.reverse : bRev1;
		Aircraft->SetThrustLevers(L1.lever, bRev1, L2.lever, Joystick.HasSecondLever() ? bRev2 : bRev1, Joystick.HasSecondLever());
	}

	// Buttons of the stick and of the throttle, by the command names in Saved/A320Joystick.ini.
	const A320State& St = Aircraft->GetSimState();
	for (const int32 Device : {Joystick.GetStickDevice(), Joystick.GetThrottleDevice()})
	{
		for (int32 B = 0; Device >= 0 && B < kButtons; ++B)
		{
			const FString CommandName = Joystick.ButtonCommand(Device, B);
			if (CommandName.IsEmpty())
			{
				continue;
			}
			const bool bPressed = Joystick.WasButtonPressed(Device, B);
			const bool bReleased = Joystick.WasButtonReleased(Device, B);
			if (CommandName == TEXT("BRAKES"))
			{
				if (Joystick.IsButtonDown(Device, B))
				{
					Inputs.Brakes = 1.0;
				}
				continue;
			}
			// Hardware switches (a quadrant's ENG MASTER and ENG MODE): on while held. Only a
			// change is applied, so the cockpit switches still work in between.
			if (CommandName == TEXT("ENG1_MASTER") || CommandName == TEXT("ENG2_MASTER"))
			{
				const int32 Engine = CommandName == TEXT("ENG1_MASTER") ? 0 : 1;
				const bool bOn = Aircraft->GetSimControls().engMaster[Engine] != 0;
				if ((bPressed && !bOn) || (bReleased && bOn))
				{
					Aircraft->ExecuteCommand(Engine == 0 ? EA320Command::EngMaster1 : EA320Command::EngMaster2);
				}
				continue;
			}
			if (CommandName == TEXT("ENG_MODE_CRANK") || CommandName == TEXT("ENG_MODE_IGN"))
			{
				if (bPressed)
				{
					Aircraft->ExecuteCommand(CommandName == TEXT("ENG_MODE_CRANK") ? EA320Command::EngModeCrank : EA320Command::EngModeIgnStart);
				}
				else if (bReleased)
				{
					Aircraft->ExecuteCommand(EA320Command::EngModeNorm);
				}
				continue;
			}
			if (!bPressed)
			{
				continue;
			}
			if (CommandName == TEXT("ATHR_DISCONNECT"))
			{
				// The instinctive disconnect on the thrust levers: off only, never on.
				if (St.athrEngaged)
				{
					Aircraft->ExecuteCommand(EA320Command::FcuAthr);
				}
				continue;
			}
			static const TPair<const TCHAR*, EA320Command> Map[] = {
				TPair<const TCHAR*, EA320Command>(TEXT("AP_DISCONNECT"), EA320Command::ApDisconnect),
				TPair<const TCHAR*, EA320Command>(TEXT("FLAPS_UP"), EA320Command::FlapsUp),
				TPair<const TCHAR*, EA320Command>(TEXT("FLAPS_DOWN"), EA320Command::FlapsDown),
				TPair<const TCHAR*, EA320Command>(TEXT("GEAR"), EA320Command::GearToggle),
				TPair<const TCHAR*, EA320Command>(TEXT("REVERSE"), EA320Command::ReverseToggle),
				TPair<const TCHAR*, EA320Command>(TEXT("SPEEDBRAKE"), EA320Command::SpeedbrakeToggle),
				TPair<const TCHAR*, EA320Command>(TEXT("VIEW"), EA320Command::ViewToggle),
				TPair<const TCHAR*, EA320Command>(TEXT("PAUSE"), EA320Command::PauseToggle),
				TPair<const TCHAR*, EA320Command>(TEXT("TOGA"), EA320Command::ThrustToga),
				TPair<const TCHAR*, EA320Command>(TEXT("IDLE"), EA320Command::ThrustIdle),
				TPair<const TCHAR*, EA320Command>(TEXT("AP1"), EA320Command::FcuAp),
				TPair<const TCHAR*, EA320Command>(TEXT("AP2"), EA320Command::FcuAp2),
				TPair<const TCHAR*, EA320Command>(TEXT("ATHR"), EA320Command::FcuAthr),
			};
			for (const TPair<const TCHAR*, EA320Command>& Entry : Map)
			{
				if (CommandName == Entry.Key)
				{
					Aircraft->ExecuteCommand(Entry.Value);
				}
			}
		}
	}

	// Hat switch: look around.
	const int32 Pov = Joystick.GetPov();
	if (Pov >= 0)
	{
		const double Rad = FMath::DegreesToRadians(Pov / 100.0);
		Aircraft->AddLook(FMath::Sin(Rad) * 90.0 * DeltaTime, FMath::Cos(Rad) * 60.0 * DeltaTime);
	}
}

void AA320PlayerController::PlayerTick(float DeltaTime)
{
	Super::PlayerTick(DeltaTime);
	AA320Aircraft* Aircraft = Cast<AA320Aircraft>(GetPawn());
	if (!Aircraft)
	{
		return;
	}

	static const FKeyCommand Bindings[] = {
		{EKeys::G, EA320Command::GearToggle},
		{EKeys::F, EA320Command::FlapsUp},
		{EKeys::V, EA320Command::FlapsDown},
		{EKeys::Slash, EA320Command::SpeedbrakeToggle},
		{EKeys::N, EA320Command::ParkBrakeToggle},
		{EKeys::R, EA320Command::ReverseToggle},
		{EKeys::End, EA320Command::ThrustIdle},
		{EKeys::Insert, EA320Command::ThrustClimb},
		{EKeys::Delete, EA320Command::ThrustFlex},
		{EKeys::Home, EA320Command::ThrustToga},
		{EKeys::L, EA320Command::LsToggle},
		{EKeys::Comma, EA320Command::NdRangeDown},
		{EKeys::Period, EA320Command::NdRangeUp},
		{EKeys::P, EA320Command::PauseToggle},
		{EKeys::Pause, EA320Command::PauseToggle},
		{EKeys::Equals, EA320Command::SimRateCycle},
		{EKeys::C, EA320Command::ViewToggle},
		{EKeys::H, EA320Command::HelpToggle},
		{EKeys::F1, EA320Command::HelpToggle},
		{EKeys::F5, EA320Command::ResetRunway},
		{EKeys::F6, EA320Command::ResetFinal10},
		{EKeys::F7, EA320Command::ResetFinal4},
		{EKeys::F9, EA320Command::RunwaySwap},  // F8 is the editor's eject key in PIE
		{EKeys::A, EA320Command::FcuAp},
		{EKeys::T, EA320Command::FcuAthr},
		{EKeys::U, EA320Command::FcuHdgPull},
		{EKeys::J, EA320Command::FcuLoc},
		{EKeys::K, EA320Command::FcuAppr},
		{EKeys::Nine, EA320Command::FcuAltPull},
		{EKeys::Zero, EA320Command::FcuVsPull},
		{EKeys::One, EA320Command::SpdDec},
		{EKeys::Two, EA320Command::SpdInc},
		{EKeys::Three, EA320Command::HdgDec},
		{EKeys::Four, EA320Command::HdgInc},
		{EKeys::Five, EA320Command::AltDec},
		{EKeys::Six, EA320Command::AltInc},
		{EKeys::Seven, EA320Command::VsDec},
		{EKeys::Eight, EA320Command::VsInc},
		{EKeys::M, EA320Command::MasterWarnAck},
		{EKeys::O, EA320Command::OverheadToggle},
		{EKeys::F2, EA320Command::JoystickPanel},
		{EKeys::F3, EA320Command::GuideMenu},
		{EKeys::F4, EA320Command::ResetApproach},
		{EKeys::Hyphen, EA320Command::SoundToggle},
		{EKeys::Gamepad_FaceButton_Bottom, EA320Command::GearToggle},
		{EKeys::Gamepad_LeftShoulder, EA320Command::FlapsUp},
		{EKeys::Gamepad_RightShoulder, EA320Command::FlapsDown},
		{EKeys::Gamepad_FaceButton_Left, EA320Command::SpeedbrakeToggle},
		{EKeys::Gamepad_FaceButton_Top, EA320Command::ReverseToggle},
		{EKeys::Gamepad_Special_Right, EA320Command::PauseToggle},
		{EKeys::Gamepad_Special_Left, EA320Command::ViewToggle},
	};
	const bool bShift = IsInputKeyDown(EKeys::LeftShift) || IsInputKeyDown(EKeys::RightShift);
	for (const FKeyCommand& Binding : Bindings)
	{
		if (WasInputKeyJustPressed(Binding.Key) && !HandleJoystickCommand(Binding.Command))
		{
			// Shift+F5: cold and dark instead of lined up with engines running.
			const bool bCold = Binding.Command == EA320Command::ResetRunway && bShift;
			const bool bAp2 = Binding.Command == EA320Command::FcuAp && bShift;  // Shift+A: AP2
			Aircraft->ExecuteCommand(bCold ? EA320Command::ResetColdDark : (bAp2 ? EA320Command::FcuAp2 : Binding.Command), bShift);
		}
	}

	if (WasInputKeyJustPressed(EKeys::Escape) && GetWorld()->WorldType == EWorldType::Game)
	{
		ConsoleCommand(TEXT("quit"));
		return;
	}

	// Keyboard stick: a spring-loaded sidestick. Half deflection, full with Shift held.
	auto KeyAxis = [this](const FKey& Negative, const FKey& Positive, const FKey& AltNegative, const FKey& AltPositive)
	{
		const bool Pos = IsInputKeyDown(Positive) || IsInputKeyDown(AltPositive);
		const bool Neg = IsInputKeyDown(Negative) || IsInputKeyDown(AltNegative);
		return (Pos ? 1.0 : 0.0) - (Neg ? 1.0 : 0.0);
	};
	const double Gain = bShift ? 1.0 : 0.5;
	// Up arrow pushes the stick forward (nose down), as in every flight sim.
	const double PitchTarget = Gain * KeyAxis(EKeys::Up, EKeys::Down, EKeys::NumPadEight, EKeys::NumPadTwo);
	const double RollTarget = Gain * KeyAxis(EKeys::Left, EKeys::Right, EKeys::NumPadFour, EKeys::NumPadSix);
	const double PedalTarget = KeyAxis(EKeys::Q, EKeys::E, EKeys::Z, EKeys::X);
	KeyStickPitch = MoveTowards(KeyStickPitch, PitchTarget, 3.0 * DeltaTime);
	KeyStickRoll = MoveTowards(KeyStickRoll, RollTarget, 3.0 * DeltaTime);
	KeyPedals = MoveTowards(KeyPedals, PedalTarget, 2.0 * DeltaTime);

	Joystick.Poll(DeltaTime);
	FA320FlightInputs Inputs;
	Inputs.StickPitch = KeyStickPitch - Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_LeftY));
	Inputs.StickRoll = KeyStickRoll + Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_LeftX));
	Inputs.Pedals = KeyPedals + Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_RightX));
	Inputs.Brakes = FMath::Max3(IsInputKeyDown(EKeys::B) ? 1.0 : 0.0,
		IsInputKeyDown(EKeys::Gamepad_FaceButton_Right) ? 1.0 : 0.0,
		(double)GetInputAnalogKeyState(EKeys::Gamepad_LeftTriggerAxis));
	Inputs.ThrustRate = (IsInputKeyDown(EKeys::PageUp) ? 0.4 : 0.0) - (IsInputKeyDown(EKeys::PageDown) ? 0.4 : 0.0)
		+ 0.4 * Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_RightY));
	ApplyJoystickButtons(Aircraft, Inputs, DeltaTime);
	Aircraft->SetFlightInputs(Inputs, DeltaTime);

	// Mouse: left click presses panel buttons, right drag looks around, middle resets.
	float MouseX = 0.0f, MouseY = 0.0f;
	const bool bHasMouse = GetMousePosition(MouseX, MouseY);
	const FVector2D Mouse(MouseX, MouseY);
	const AA320Hud* Hud = Cast<AA320Hud>(GetHUD());
	if (bHasMouse && Hud && WasInputKeyJustPressed(EKeys::LeftMouseButton))
	{
		// Pushbuttons and switches first (pop-up panels sit on top), then levers to drag.
		const EA320Command Clicked = Hud->CommandAt(Mouse);
		DraggedLever = Clicked == EA320Command::None ? Hud->LeverAt(Mouse) : EA320Lever::None;
		if (Clicked != EA320Command::None && !HandleJoystickCommand(Clicked))
		{
			Aircraft->ExecuteCommand(Clicked, bShift);
		}
	}
	if (DraggedLever != EA320Lever::None)
	{
		if (bHasMouse && Hud && IsInputKeyDown(EKeys::LeftMouseButton))
		{
			Aircraft->SetLever(DraggedLever, Hud->LeverPosition(DraggedLever, Mouse));
		}
		else
		{
			DraggedLever = EA320Lever::None;
		}
	}
	if (bHasMouse && IsInputKeyDown(EKeys::RightMouseButton))
	{
		if (bLooking)
		{
			const FVector2D Delta = Mouse - LastMouse;
			Aircraft->AddLook(Delta.X * 0.2, -Delta.Y * 0.2);
		}
		bLooking = true;
		LastMouse = Mouse;
	}
	else
	{
		bLooking = false;
	}
	if (WasInputKeyJustPressed(EKeys::MiddleMouseButton))
	{
		Aircraft->ResetLook();
	}
}
