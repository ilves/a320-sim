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

bool AA320PlayerController::HandleJoystickCommand(EA320Command Command, int32 Param)
{
	const int32 Learn = static_cast<int32>(Command) - static_cast<int32>(EA320Command::JoyLearn0);
	const int32 Axis = static_cast<int32>(Command) - static_cast<int32>(EA320Command::JoyAxis0);
	const int32 Invert = static_cast<int32>(Command) - static_cast<int32>(EA320Command::JoyInvert0);
	switch (Command)
	{
	case EA320Command::JoystickPanel:
		if (bJoystickPanel)
		{
			Joystick.StartLearn(-1);  // closing cancels LEARN, SET and CAL
		}
		bJoystickPanel = !bJoystickPanel;
		return true;
	case EA320Command::JoyRescan: Joystick.Rescan(); return true;
	case EA320Command::JoyCalStart: Joystick.StartCalibration(a320::joy::kCalThrust); return true;
	case EA320Command::JoyCalFlaps: Joystick.StartCalibration(a320::joy::kCalFlaps); return true;
	case EA320Command::JoyCalSpeedbrake: Joystick.StartCalibration(a320::joy::kCalSpeedbrake); return true;
	case EA320Command::JoyCalSet: Joystick.CalibrationSet(); return true;
	case EA320Command::JoyCalSkip: Joystick.CalibrationSkip(); return true;
	case EA320Command::JoyCalCancel: Joystick.CancelCalibration(); return true;
	case EA320Command::JoyPageAxes: bJoystickButtonsPage = false; return true;
	case EA320Command::JoyPageButtons: bJoystickButtonsPage = true; return true;
	case EA320Command::JoyBindSet: Joystick.StartButtonLearn(Param); return true;
	case EA320Command::JoyBindClear: Joystick.ClearCommand(Param); return true;
	default: break;
	}
	if (Learn >= 0 && Learn < a320::joy::kFunctionCount)
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

void AA320PlayerController::ApplyHardwareCommand(AA320Aircraft* Aircraft, FA320FlightInputs& Inputs, const FString& Name,
	int32 Presses, bool bReleased, bool bDown)
{
	using a320::joy::Action;
	const a320::joy::CommandInfo* Info = a320::joy::findCommand(TCHAR_TO_UTF8(*Name));
	if (!Info)
	{
		return;
	}
	if (Info->action == Action::Held)
	{
		// Switches: only a change is applied, so the cockpit switches still work in between.
		if (Name == TEXT("BRAKES"))
		{
			if (bDown)
			{
				Inputs.Brakes = 1.0;
			}
			return;
		}
		if (Name == TEXT("ALT_1000"))
		{
			bAltStep1000 = bDown;
			return;
		}
		if (Presses == 0 && !bReleased)
		{
			return;
		}
		if (Name == TEXT("ENG1_MASTER")) Aircraft->SetSwitch(EA320Switch::EngMaster1, bDown);
		else if (Name == TEXT("ENG2_MASTER")) Aircraft->SetSwitch(EA320Switch::EngMaster2, bDown);
		else if (Name == TEXT("ENG_MODE_CRANK") || Name == TEXT("ENG_MODE_IGN"))
		{
			// Released: back to NORM, unless the selector already went to the other position.
			const int32 Mode = Name == TEXT("ENG_MODE_CRANK") ? A320_ENG_MODE_CRANK : A320_ENG_MODE_IGN_START;
			if (bDown || Aircraft->GetSimControls().engMode == Mode)
			{
				Aircraft->SetSwitch(EA320Switch::EngMode, bDown ? Mode : A320_ENG_MODE_NORM);
			}
		}
		else if (Name == TEXT("PARK_BRAKE")) Aircraft->SetSwitch(EA320Switch::ParkBrake, bDown);
		else if (Name == TEXT("GEAR_DOWN_SW")) Aircraft->SetSwitch(EA320Switch::Gear, bDown ? 1 : 0);
		else if (Name == TEXT("GEAR_UP_SW")) Aircraft->SetSwitch(EA320Switch::Gear, bDown ? 0 : 1);
		else if (Name == TEXT("SPOILERS_ARM")) Aircraft->SetSwitch(EA320Switch::SpoilersArm, bDown);
		return;
	}
	if (Presses == 0)
	{
		return;
	}
	if (Info->action == Action::Select)
	{
		struct FSelect
		{
			const TCHAR* Name;
			EA320Switch Switch;
			int32 Value;
		};
		static const FSelect Selects[] = {
			{TEXT("ND_ARC"), EA320Switch::NdMode, A320_ND_ARC},
			{TEXT("ND_NAV"), EA320Switch::NdMode, A320_ND_ROSE_NAV},
			{TEXT("ND_LS"), EA320Switch::NdMode, A320_ND_ROSE_LS},
			{TEXT("ND_RANGE_10"), EA320Switch::NdRange, 10},
			{TEXT("ND_RANGE_20"), EA320Switch::NdRange, 20},
			{TEXT("ND_RANGE_40"), EA320Switch::NdRange, 40},
			{TEXT("ND_RANGE_80"), EA320Switch::NdRange, 80},
			{TEXT("ND_RANGE_160"), EA320Switch::NdRange, 160},
			{TEXT("ND_RANGE_320"), EA320Switch::NdRange, 320},
			{TEXT("GEAR_UP"), EA320Switch::Gear, 0},
			{TEXT("GEAR_DOWN"), EA320Switch::Gear, 1},
			{TEXT("AUTOBRK_OFF"), EA320Switch::Autobrake, A320_AUTOBRAKE_OFF},
			{TEXT("AUTOBRK_LO"), EA320Switch::Autobrake, A320_AUTOBRAKE_LO},
			{TEXT("AUTOBRK_MED"), EA320Switch::Autobrake, A320_AUTOBRAKE_MED},
			{TEXT("AUTOBRK_MAX"), EA320Switch::Autobrake, A320_AUTOBRAKE_MAX},
		};
		for (const FSelect& Select : Selects)
		{
			if (Name == Select.Name)
			{
				Aircraft->SetSwitch(Select.Switch, Select.Value);
			}
		}
		return;
	}
	if (Name == TEXT("ATHR_DISCONNECT"))
	{
		// The instinctive disconnect on the thrust levers: off only, never on.
		if (Aircraft->GetSimState().athrEngaged)
		{
			Aircraft->ExecuteCommand(EA320Command::FcuAthr);
		}
		return;
	}
	struct FPress
	{
		const TCHAR* Name;
		EA320Command Command;
	};
	static const FPress PressCommands[] = {
		{TEXT("AP1"), EA320Command::FcuAp},
		{TEXT("AP2"), EA320Command::FcuAp2},
		{TEXT("ATHR"), EA320Command::FcuAthr},
		{TEXT("LOC"), EA320Command::FcuLoc},
		{TEXT("APPR"), EA320Command::FcuAppr},
		{TEXT("SPD_INC"), EA320Command::SpdInc},
		{TEXT("SPD_DEC"), EA320Command::SpdDec},
		{TEXT("HDG_INC"), EA320Command::HdgInc},
		{TEXT("HDG_DEC"), EA320Command::HdgDec},
		{TEXT("HDG_PULL"), EA320Command::FcuHdgPull},
		{TEXT("HDG_PUSH"), EA320Command::FcuHdgPush},
		{TEXT("ALT_INC"), EA320Command::AltInc},
		{TEXT("ALT_DEC"), EA320Command::AltDec},
		{TEXT("ALT_PULL"), EA320Command::FcuAltPull},
		{TEXT("ALT_PUSH"), EA320Command::FcuAltPush},
		{TEXT("VS_INC"), EA320Command::VsInc},
		{TEXT("VS_DEC"), EA320Command::VsDec},
		{TEXT("VS_PULL"), EA320Command::FcuVsPull},
		{TEXT("VS_PUSH"), EA320Command::FcuVsPush},
		{TEXT("TRK_FPA"), EA320Command::FcuTrkFpa},
		{TEXT("LS"), EA320Command::LsToggle},
		{TEXT("ND_MODE"), EA320Command::NdModeToggle},
		{TEXT("ND_RANGE_INC"), EA320Command::NdRangeUp},
		{TEXT("ND_RANGE_DEC"), EA320Command::NdRangeDown},
		{TEXT("AP_DISCONNECT"), EA320Command::ApDisconnect},
		{TEXT("TOGA"), EA320Command::ThrustToga},
		{TEXT("IDLE"), EA320Command::ThrustIdle},
		{TEXT("REVERSE"), EA320Command::ReverseToggle},
		{TEXT("FLAPS_UP"), EA320Command::FlapsUp},
		{TEXT("FLAPS_DOWN"), EA320Command::FlapsDown},
		{TEXT("SPEEDBRAKE"), EA320Command::SpeedbrakeToggle},
		{TEXT("GEAR"), EA320Command::GearToggle},
		{TEXT("APU_MASTER"), EA320Command::ApuMaster},
		{TEXT("APU_START"), EA320Command::ApuStart},
		{TEXT("MASTER_WARN"), EA320Command::MasterWarnAck},
		{TEXT("VIEW"), EA320Command::ViewToggle},
		{TEXT("PAUSE"), EA320Command::PauseToggle},
	};
	for (const FPress& Press : PressCommands)
	{
		if (Name != Press.Name)
		{
			continue;
		}
		// Every encoder click counts, also several in one frame. The ALT knob steps 100 or
		// 1000 ft with the 100/1000 switch, as on the FCU.
		const bool bLarge = (Press.Command == EA320Command::AltInc || Press.Command == EA320Command::AltDec) && bAltStep1000;
		for (int32 i = 0; i < Presses; ++i)
		{
			Aircraft->ExecuteCommand(Press.Command, bLarge);
		}
	}
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
	if (Joystick.AxisMoved(kFlaps))
	{
		Aircraft->SetSwitch(EA320Switch::Flaps, Joystick.FlapsLever());
	}
	if (Joystick.AxisMoved(kSpeedbrake))
	{
		const SpeedbrakePosition Lever = Joystick.Speedbrake();
		Aircraft->SetSpeedbrake(Lever.amount);
		if (Joystick.GetConfig().speedbrakeCal.hasArm)
		{
			Aircraft->SetSwitch(EA320Switch::SpoilersArm, Lever.armed);
		}
	}

	// Buttons and switches of every device, by the assignments in Saved/A320Joystick.ini.
	if (!Joystick.ButtonsSuppressed() && Joystick.GetButtonLearning() < 0)
	{
		for (const ButtonBind& Bind : Joystick.GetConfig().binds)
		{
			const int32 Device = Joystick.BindDevice(Bind);
			if (Device < 0)
			{
				continue;
			}
			ApplyHardwareCommand(Aircraft, Inputs, UTF8_TO_TCHAR(Bind.command.c_str()), Joystick.ButtonPresses(Device, Bind.button),
				Joystick.WasButtonReleased(Device, Bind.button), Joystick.IsButtonDown(Device, Bind.button));
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

namespace
{
	struct FMcduTypingKey
	{
		FKey Key;
		int32 Code;
	};

	// Keyboard keys that type into the MCDU scratchpad (letters, digits, '.', '/', space, CLR).
	const TArray<FMcduTypingKey>& McduTypingKeys()
	{
		static TArray<FMcduTypingKey> Keys;
		if (Keys.Num() == 0)
		{
			const FKey Letters[] = {EKeys::A, EKeys::B, EKeys::C, EKeys::D, EKeys::E, EKeys::F, EKeys::G, EKeys::H, EKeys::I,
				EKeys::J, EKeys::K, EKeys::L, EKeys::M, EKeys::N, EKeys::O, EKeys::P, EKeys::Q, EKeys::R, EKeys::S, EKeys::T,
				EKeys::U, EKeys::V, EKeys::W, EKeys::X, EKeys::Y, EKeys::Z};
			int32 Code = 'A';
			for (const FKey& Letter : Letters)
			{
				Keys.Add({Letter, Code++});
			}
			const FKey Digits[] = {EKeys::Zero, EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four, EKeys::Five, EKeys::Six,
				EKeys::Seven, EKeys::Eight, EKeys::Nine};
			Code = '0';
			for (const FKey& Digit : Digits)
			{
				Keys.Add({Digit, Code++});
			}
			const FKey NumPad[] = {EKeys::NumPadZero, EKeys::NumPadOne, EKeys::NumPadTwo, EKeys::NumPadThree, EKeys::NumPadFour,
				EKeys::NumPadFive, EKeys::NumPadSix, EKeys::NumPadSeven, EKeys::NumPadEight, EKeys::NumPadNine};
			Code = '0';
			for (const FKey& Digit : NumPad)
			{
				Keys.Add({Digit, Code++});
			}
			Keys.Add({EKeys::Period, '.'});
			Keys.Add({EKeys::Decimal, '.'});
			Keys.Add({EKeys::Slash, '/'});
			Keys.Add({EKeys::Divide, '/'});
			Keys.Add({EKeys::SpaceBar, ' '});
			Keys.Add({EKeys::Hyphen, '-'});
			Keys.Add({EKeys::Subtract, '-'});
			Keys.Add({EKeys::Add, '+'});
			Keys.Add({EKeys::BackSpace, A320_MCDU_CLR});
			Keys.Add({EKeys::Delete, A320_MCDU_CLR});
		}
		return Keys;
	}
}

bool AA320PlayerController::IsMcduTypingKey(const FKey& Key) const
{
	for (const FMcduTypingKey& Typing : McduTypingKeys())
	{
		if (Typing.Key == Key)
		{
			return true;
		}
	}
	return false;
}

void AA320PlayerController::TypeIntoMcdu(AA320Aircraft& Aircraft)
{
	for (const FMcduTypingKey& Typing : McduTypingKeys())
	{
		if (WasInputKeyJustPressed(Typing.Key))
		{
			Aircraft.McduKey(Typing.Code);
		}
	}
}

bool AA320PlayerController::IsMapTypingKey(const FKey& Key)
{
	static const FKey Keys[] = {EKeys::A, EKeys::B, EKeys::C, EKeys::D, EKeys::E, EKeys::F, EKeys::G, EKeys::H, EKeys::I,
		EKeys::J, EKeys::K, EKeys::L, EKeys::M, EKeys::N, EKeys::O, EKeys::P, EKeys::Q, EKeys::R, EKeys::S, EKeys::T,
		EKeys::U, EKeys::V, EKeys::W, EKeys::X, EKeys::Y, EKeys::Z, EKeys::Zero, EKeys::One, EKeys::Two, EKeys::Three,
		EKeys::Four, EKeys::Five, EKeys::Six, EKeys::Seven, EKeys::Eight, EKeys::Nine, EKeys::SpaceBar, EKeys::Hyphen,
		EKeys::BackSpace, EKeys::Slash, EKeys::Comma, EKeys::Period, EKeys::Equals};
	for (const FKey& K : Keys)
	{
		if (K == Key)
		{
			return true;
		}
	}
	return false;
}

void AA320PlayerController::TypeIntoMapSearch(AA320Hud& Hud, AA320Aircraft& Aircraft)
{
	static const FKey Letters[] = {EKeys::A, EKeys::B, EKeys::C, EKeys::D, EKeys::E, EKeys::F, EKeys::G, EKeys::H, EKeys::I,
		EKeys::J, EKeys::K, EKeys::L, EKeys::M, EKeys::N, EKeys::O, EKeys::P, EKeys::Q, EKeys::R, EKeys::S, EKeys::T,
		EKeys::U, EKeys::V, EKeys::W, EKeys::X, EKeys::Y, EKeys::Z};
	static const FKey Digits[] = {EKeys::Zero, EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four, EKeys::Five, EKeys::Six,
		EKeys::Seven, EKeys::Eight, EKeys::Nine};
	for (int32 i = 0; i < 26; ++i)
	{
		if (WasInputKeyJustPressed(Letters[i]))
		{
			Hud.MapType(static_cast<TCHAR>('a' + i));
		}
	}
	for (int32 i = 0; i < 10; ++i)
	{
		if (WasInputKeyJustPressed(Digits[i]))
		{
			Hud.MapType(static_cast<TCHAR>('0' + i));
		}
	}
	if (WasInputKeyJustPressed(EKeys::SpaceBar))
	{
		Hud.MapType(TEXT(' '));
	}
	if (WasInputKeyJustPressed(EKeys::Hyphen))
	{
		Hud.MapType(TEXT('-'));
	}
	if (WasInputKeyJustPressed(EKeys::BackSpace))
	{
		Hud.MapBackspace();
	}
	if (WasInputKeyJustPressed(EKeys::Enter))
	{
		Hud.MapEnter(Aircraft);
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
		{EKeys::Backslash, EA320Command::FcuTrkFpa},
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
		{EKeys::Tab, EA320Command::McduToggle},
		{EKeys::F10, EA320Command::RadioToggle},
		{EKeys::F11, EA320Command::FlightMenu},
		{EKeys::F12, EA320Command::MapToggle},
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
	// With the MCDU open the keyboard types into its scratchpad; those keys don't fly the aircraft.
	const bool bMcduTyping = Aircraft->IsMcduVisible();
	if (bMcduTyping)
	{
		TypeIntoMcdu(*Aircraft);
	}
	// The world map's search box takes the letter and digit keys while it is active.
	AA320Hud* MapHud = Cast<AA320Hud>(GetHUD());
	const bool bMapTyping = Aircraft->IsMapVisible() && MapHud && MapHud->IsMapSearchActive() && !bMcduTyping;
	if (bMapTyping)
	{
		TypeIntoMapSearch(*MapHud, *Aircraft);
	}
	// With the RADIO window open (and the MCDU closed), 1-6 pick an ATC reply instead of FCU keys.
	static const FKey ReplyKeys[] = {EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four, EKeys::Five, EKeys::Six};
	const bool bRadioReplies = Aircraft->IsRadioVisible() && !bMcduTyping && !bMapTyping;
	if (bRadioReplies)
	{
		int32 Reply = 0;
		for (const FKey& ReplyKey : ReplyKeys)
		{
			if (WasInputKeyJustPressed(ReplyKey))
			{
				Aircraft->ExecuteCommand(EA320Command::AtcReply, false, Reply);
			}
			++Reply;
		}
	}
	for (const FKeyCommand& Binding : Bindings)
	{
		if ((bMcduTyping && IsMcduTypingKey(Binding.Key)) || (bMapTyping && IsMapTypingKey(Binding.Key)))
		{
			continue;
		}
		bool bReplyKey = false;
		for (const FKey& ReplyKey : ReplyKeys)
		{
			bReplyKey = bReplyKey || (bRadioReplies && Binding.Key == ReplyKey);
		}
		if (bReplyKey)
		{
			continue;
		}
		if (WasInputKeyJustPressed(Binding.Key) && !HandleJoystickCommand(Binding.Command))
		{
			// Shift+F5: cold and dark instead of lined up with engines running.
			const bool bCold = Binding.Command == EA320Command::ResetRunway && bShift;
			const bool bAp2 = Binding.Command == EA320Command::FcuAp && bShift;  // Shift+A: AP2
			Aircraft->ExecuteCommand(bCold ? EA320Command::ResetColdDark : (bAp2 ? EA320Command::FcuAp2 : Binding.Command), bShift);
		}
	}

	if ((Aircraft->IsFlightMenuVisible() || (Aircraft->IsMapVisible() && !bMapTyping)) && WasInputKeyJustPressed(EKeys::Enter))
	{
		Aircraft->ExecuteCommand(EA320Command::FlightGo, false);
	}
	if (WasInputKeyJustPressed(EKeys::Escape) && Aircraft->IsMapVisible())
	{
		Aircraft->ExecuteCommand(EA320Command::MapToggle, false);  // Esc closes the map first
	}
	else if (WasInputKeyJustPressed(EKeys::Escape) && Aircraft->IsFlightMenuVisible())
	{
		Aircraft->ExecuteCommand(EA320Command::FlightMenu, false);  // Esc closes the FLIGHT menu first
	}
	else if (WasInputKeyJustPressed(EKeys::Escape) && bMcduTyping)
	{
		Aircraft->ExecuteCommand(EA320Command::McduToggle, false);  // Esc closes the MCDU first
	}
	else if (WasInputKeyJustPressed(EKeys::Escape) && GetWorld()->WorldType == EWorldType::Game)
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
	// While the MCDU is open the numpad types; the arrow keys still fly.
	const FKey NumUp = bMcduTyping ? EKeys::Invalid : EKeys::NumPadEight, NumDown = bMcduTyping ? EKeys::Invalid : EKeys::NumPadTwo;
	const FKey NumLeft = bMcduTyping ? EKeys::Invalid : EKeys::NumPadFour, NumRight = bMcduTyping ? EKeys::Invalid : EKeys::NumPadSix;
	const double PitchTarget = Gain * KeyAxis(EKeys::Up, EKeys::Down, NumUp, NumDown);
	const double RollTarget = Gain * KeyAxis(EKeys::Left, EKeys::Right, NumLeft, NumRight);
	const double PedalTarget = bMcduTyping || bMapTyping ? 0.0 : KeyAxis(EKeys::Q, EKeys::E, EKeys::Z, EKeys::X);
	KeyStickPitch = MoveTowards(KeyStickPitch, PitchTarget, 3.0 * DeltaTime);
	KeyStickRoll = MoveTowards(KeyStickRoll, RollTarget, 3.0 * DeltaTime);
	KeyPedals = MoveTowards(KeyPedals, PedalTarget, 2.0 * DeltaTime);

	Joystick.Poll(DeltaTime);
	WingFlex.Tick(*Aircraft, DeltaTime);
	FA320FlightInputs Inputs;
	Inputs.StickPitch = KeyStickPitch - Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_LeftY));
	Inputs.StickRoll = KeyStickRoll + Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_LeftX));
	Inputs.Pedals = KeyPedals + Deadzone(GetInputAnalogKeyState(EKeys::Gamepad_RightX));
	Inputs.Brakes = FMath::Max3(IsInputKeyDown(EKeys::B) && !bMcduTyping && !bMapTyping ? 1.0 : 0.0,
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
	AA320Hud* Hud = Cast<AA320Hud>(GetHUD());
	const bool bOnMap = bHasMouse && Hud && Aircraft->IsMapVisible() && Hud->IsOverMap(Mouse);
	if (bOnMap && WasInputKeyJustPressed(EKeys::MouseScrollUp))
	{
		Hud->MapZoom(1.0 / 1.25, Mouse);
	}
	if (bOnMap && WasInputKeyJustPressed(EKeys::MouseScrollDown))
	{
		Hud->MapZoom(1.25, Mouse);
	}
	if (bHasMouse && Hud && WasInputKeyJustPressed(EKeys::LeftMouseButton))
	{
		// Pushbuttons and switches first (pop-up panels sit on top), then levers to drag, but not
		// through a window.
		const EA320Command Clicked = Hud->CommandAt(Mouse);
		DraggedLever = Clicked == EA320Command::None && !Hud->IsOverButton(Mouse) ? Hud->LeverAt(Mouse) : EA320Lever::None;
		if (AA320Hud::IsMapCommand(Clicked))
		{
			Hud->MapCommand(Clicked, Hud->ParamAt(Mouse), *Aircraft);
		}
		else if (Clicked == EA320Command::None && bOnMap)
		{
			bMapDragging = true;
			MapDragPixels = 0.0;
			LastMapMouse = Mouse;
		}
		else if (Clicked == EA320Command::McduKey)
		{
			Aircraft->McduKey(Hud->ParamAt(Mouse));
		}
		else if (Clicked != EA320Command::None && !HandleJoystickCommand(Clicked, Hud->ParamAt(Mouse)))
		{
			Aircraft->ExecuteCommand(Clicked, bShift, Hud->ParamAt(Mouse));
		}
	}
	if (bMapDragging && Hud)
	{
		if (bHasMouse && IsInputKeyDown(EKeys::LeftMouseButton))
		{
			const FVector2D Delta = Mouse - LastMapMouse;
			MapDragPixels += Delta.Size();
			if (MapDragPixels > 4.0)
			{
				Hud->MapPan(Delta);
			}
			LastMapMouse = Mouse;
		}
		else
		{
			if (MapDragPixels <= 4.0 && bHasMouse)
			{
				Hud->MapClick(Mouse);
			}
			bMapDragging = false;
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
