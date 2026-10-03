#include "A320PlayerController.h"

#include "A320Aircraft.h"
#include "A320Commands.h"
#include "A320Hud.h"
#include "Engine/World.h"
#include "InputCoreTypes.h"

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
		if (WasInputKeyJustPressed(Binding.Key))
		{
			// Shift+F5: cold and dark instead of lined up with engines running.
			const bool bCold = Binding.Command == EA320Command::ResetRunway && bShift;
			Aircraft->ExecuteCommand(bCold ? EA320Command::ResetColdDark : Binding.Command, bShift);
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

	FA320FlightInputs Inputs;
	Inputs.StickPitch = KeyStickPitch - Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_LeftY));
	Inputs.StickRoll = KeyStickRoll + Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_LeftX));
	Inputs.Pedals = KeyPedals + Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_RightX));
	Inputs.Brakes = FMath::Max3(IsInputKeyDown(EKeys::B) ? 1.0 : 0.0,
		IsInputKeyDown(EKeys::Gamepad_FaceButton_Right) ? 1.0 : 0.0,
		(double)GetInputAnalogKeyState(EKeys::Gamepad_LeftTriggerAxis));
	Inputs.ThrustRate = (IsInputKeyDown(EKeys::PageUp) ? 0.4 : 0.0) - (IsInputKeyDown(EKeys::PageDown) ? 0.4 : 0.0)
		+ 0.4 * Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_RightY));
	Aircraft->SetFlightInputs(Inputs, DeltaTime);

	// Mouse: left click presses panel buttons, right drag looks around, middle resets.
	float MouseX = 0.0f, MouseY = 0.0f;
	const bool bHasMouse = GetMousePosition(MouseX, MouseY);
	const FVector2D Mouse(MouseX, MouseY);
	const AA320Hud* Hud = Cast<AA320Hud>(GetHUD());
	if (bHasMouse && Hud && WasInputKeyJustPressed(EKeys::LeftMouseButton))
	{
		// Levers are dragged; everything else is a pushbutton or switch.
		DraggedLever = Hud->LeverAt(Mouse);
		if (DraggedLever == EA320Lever::None)
		{
			Aircraft->ExecuteCommand(Hud->CommandAt(Mouse), bShift);
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
