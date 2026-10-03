#include "A320Joystick.h"

#include "A320Sim.h"
#include "Misc/FileHelper.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include "Windows/AllowWindowsPlatformTypes.h"
#include <mmsystem.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
#if PLATFORM_WINDOWS
	// The product name Windows shows in "Game Controllers": JOYCAPS only has a generic driver
	// name, the real one is stored per VID/PID in the registry.
	FString OemName(WORD Mid, WORD Pid)
	{
		const FString Key = FString::Printf(
			TEXT("System\\CurrentControlSet\\Control\\MediaProperties\\PrivateProperties\\Joystick\\OEM\\VID_%04X&PID_%04X"), Mid, Pid);
		for (HKEY Root : {HKEY_CURRENT_USER, HKEY_LOCAL_MACHINE})
		{
			WCHAR Buffer[256] = {};
			DWORD Size = sizeof(Buffer);
			if (RegGetValueW(Root, *Key, L"OEMName", RRF_RT_REG_SZ, nullptr, Buffer, &Size) == ERROR_SUCCESS)
			{
				return FString(Buffer);
			}
		}
		return FString();
	}
#endif
}

using namespace a320::joy;

void FA320Joystick::Init(const FString& InConfigPath)
{
	ConfigPath = InConfigPath;
	FString Text;
	if (FFileHelper::LoadFileToString(Text, *ConfigPath))
	{
		fromText(TCHAR_TO_UTF8(*Text), JoyConfig);
	}
	else
	{
		Save();  // write the defaults so the button mapping can be edited
	}
	Rescan();
}

void FA320Joystick::Save() const
{
	FFileHelper::SaveStringToFile(UTF8_TO_TCHAR(toText(JoyConfig).c_str()), *ConfigPath);
}

void FA320Joystick::Rescan()
{
	Devices.Reset();
#if PLATFORM_WINDOWS
	for (UINT Id = 0; Id < 16; ++Id)
	{
		JOYINFOEX Info = {};
		Info.dwSize = sizeof(Info);
		Info.dwFlags = JOY_RETURNALL;
		if (joyGetPosEx(Id, &Info) != JOYERR_NOERROR)
		{
			continue;
		}
		JOYCAPSW Caps = {};
		if (joyGetDevCapsW(Id, &Caps, sizeof(Caps)) != JOYERR_NOERROR)
		{
			continue;
		}
		FA320JoystickDevice Device;
		Device.SystemId = static_cast<int32>(Id);
		const FString Oem = OemName(Caps.wMid, Caps.wPid);
		Device.Name = Oem.IsEmpty() ? FString(Caps.szPname) : Oem;
		// Xbox/XInput pads are already Unreal gamepads; reading them here too would double them.
		constexpr WORD MicrosoftVid = 0x045E;
		if (Device.Name.Contains(TEXT("xbox")) || Device.Name.Contains(TEXT("xinput")) ||
			(Caps.wMid == MicrosoftVid && Device.Name.Contains(TEXT("controller"))))
		{
			UE_LOG(LogA320, Log, TEXT("Skipping %s: used as a gamepad"), *Device.Name);
			continue;
		}
		Device.NumAxes = static_cast<int32>(Caps.wNumAxes);
		Device.NumButtons = static_cast<int32>(Caps.wNumButtons);
		const UINT Mins[kAxes] = {Caps.wXmin, Caps.wYmin, Caps.wZmin, Caps.wRmin, Caps.wUmin, Caps.wVmin};
		const UINT Maxs[kAxes] = {Caps.wXmax, Caps.wYmax, Caps.wZmax, Caps.wRmax, Caps.wUmax, Caps.wVmax};
		const bool Present[kAxes] = {true, true, (Caps.wCaps & JOYCAPS_HASZ) != 0, (Caps.wCaps & JOYCAPS_HASR) != 0,
			(Caps.wCaps & JOYCAPS_HASU) != 0, (Caps.wCaps & JOYCAPS_HASV) != 0};
		for (int32 A = 0; A < kAxes; ++A)
		{
			Device.AxisMin[A] = Mins[A];
			Device.AxisMax[A] = Maxs[A];
			Device.HasAxis[A] = Present[A];
		}
		Device.bHasPov = (Caps.wCaps & JOYCAPS_HASPOV) != 0;
		UE_LOG(LogA320, Log, TEXT("Joystick %d: %s (%d axes, %d buttons)"), Devices.Num(), *Device.Name, Device.NumAxes, Device.NumButtons);
		Devices.Add(Device);
	}
#endif
	Axes.assign(static_cast<size_t>(Devices.Num()), AxisValues{});
	ButtonsNow.Init(0, Devices.Num());
	ButtonsBefore.Init(0, Devices.Num());
	LastThrottle[0] = LastThrottle[1] = 2.0;
	bFirstPoll = true;

	std::vector<std::string> Names;
	std::vector<std::array<bool, kAxes>> HasAxes;
	for (const FA320JoystickDevice& Device : Devices)
	{
		Names.emplace_back(TCHAR_TO_UTF8(*Device.Name));
		std::array<bool, kAxes> Present{};
		for (int32 A = 0; A < kAxes; ++A)
		{
			Present[static_cast<size_t>(A)] = Device.HasAxis[A];
		}
		HasAxes.push_back(Present);
	}
	const bool bResolved = resolveDevices(JoyConfig, Names);
	if (autoAssignThrottle(JoyConfig, Names, HasAxes))
	{
		const int32 Quadrant = JoyConfig.bind[kThrottle].device;
		Message = FString::Printf(TEXT("Throttle found: %s. Press CALIBRATE THRUST to teach it the detents."),
			Devices.IsValidIndex(Quadrant) ? *Devices[Quadrant].Name : TEXT("?"));
		UE_LOG(LogA320, Log, TEXT("%s"), *Message);
		Save();
	}
	else if (bResolved)
	{
		Save();
	}
}

void FA320Joystick::Poll(float DeltaSeconds)
{
	if (Devices.Num() == 0)
	{
		// Plugged in later? Scanning all ids is slow-ish, so only every few seconds.
		RescanTimer -= DeltaSeconds;
		if (RescanTimer <= 0.0f)
		{
			RescanTimer = 5.0f;
			Rescan();
		}
		bThrottleMoved = false;
		return;
	}
	ButtonsBefore = ButtonsNow;
	Pov = -1;
	const int32 StickDevice = GetStickDevice();
#if PLATFORM_WINDOWS
	for (int32 D = 0; D < Devices.Num(); ++D)
	{
		JOYINFOEX Info = {};
		Info.dwSize = sizeof(Info);
		Info.dwFlags = JOY_RETURNALL | JOY_RETURNPOVCTS;
		if (joyGetPosEx(static_cast<UINT>(Devices[D].SystemId), &Info) != JOYERR_NOERROR)
		{
			Rescan();  // unplugged
			return;
		}
		const DWORD Raw[kAxes] = {Info.dwXpos, Info.dwYpos, Info.dwZpos, Info.dwRpos, Info.dwUpos, Info.dwVpos};
		for (int32 A = 0; A < kAxes; ++A)
		{
			Axes[static_cast<size_t>(D)][static_cast<size_t>(A)] =
				Devices[D].HasAxis[A] ? normalize(Raw[A], Devices[D].AxisMin[A], Devices[D].AxisMax[A]) : 0.0;
		}
		ButtonsNow[D] = static_cast<uint32>(Info.dwButtons);
		if (D == StickDevice)
		{
			// Hundredths of a degree; anything outside 0..35999 (e.g. 65535) means centred.
			Pov = Devices[D].bHasPov && Info.dwPOV <= 35999 ? static_cast<int32>(Info.dwPOV) : -1;
		}
	}
#endif

	if (bFirstPoll)
	{
		// Buttons and switches already held when a device appears are not new presses.
		ButtonsBefore = ButtonsNow;
		bFirstPoll = false;
	}

	if (Learning >= 0)
	{
		int Device = -1, Axis = -1;
		if (detectAxis(LearnBaseline, Axes, Device, Axis))
		{
			SetBindingDevice(JoyConfig.bind[Learning], Device);
			JoyConfig.bind[Learning].axis = Axis;
			ResetLeverCalibration(Learning);
			Learning = -1;
			Save();
		}
	}

	// The first reading only sets the reference, so the levers' resting position does not
	// override the scenario's thrust at start-up.
	bThrottleMoved = false;
	const Function Levers[2] = {kThrottle, kThrottle2};
	for (int32 L = 0; L < 2; ++L)
	{
		if (!IsBound(Levers[L]))
		{
			continue;
		}
		const double Throttle = axisValue(JoyConfig.bind[Levers[L]], Axes);
		const bool bMoved = LastThrottle[L] <= 1.5 && FMath::Abs(Throttle - LastThrottle[L]) > 0.01;
		if (LastThrottle[L] > 1.5 || bMoved)
		{
			LastThrottle[L] = Throttle;
		}
		bThrottleMoved = bThrottleMoved || bMoved;
	}
	if (CalStep >= 0)
	{
		bThrottleMoved = false;  // the levers travel through every detent while calibrating
	}
}

LeverPosition FA320Joystick::Lever(int32 Index) const
{
	const int32 L = Index == 1 && HasSecondLever() ? 1 : 0;
	return throttleFromAxis(Value(L == 1 ? kThrottle2 : kThrottle), JoyConfig.cal[L]);
}

int32 FA320Joystick::GetStickDevice() const
{
	if (IsBound(kPitch))
	{
		return JoyConfig.bind[kPitch].device;
	}
	// No stick bound: the first device that is not the throttle.
	for (int32 D = 0; D < Devices.Num(); ++D)
	{
		if (!IsBound(kThrottle) || JoyConfig.bind[kThrottle].device != D)
		{
			return D;
		}
	}
	return -1;
}

int32 FA320Joystick::GetThrottleDevice() const
{
	const int32 D = IsBound(kThrottle) ? JoyConfig.bind[kThrottle].device : -1;
	return D != GetStickDevice() ? D : -1;
}

bool FA320Joystick::IsBound(Function F) const
{
	const Binding& B = JoyConfig.bind[F];
	return B.device >= 0 && B.device < Devices.Num();
}

double FA320Joystick::Value(Function F) const
{
	if (!IsBound(F))
	{
		return F == kBrakeLeft || F == kBrakeRight ? -1.0 : 0.0;
	}
	const double V = axisValue(JoyConfig.bind[F], Axes);
	return F == kPitch || F == kRoll || F == kRudder ? applyDeadzone(V, JoyConfig.deadzone) : V;
}

bool FA320Joystick::IsButtonDown(int32 Device, int32 Button) const
{
	return ButtonsNow.IsValidIndex(Device) && Button >= 0 && Button < kButtons && (ButtonsNow[Device] & (1u << Button)) != 0;
}

bool FA320Joystick::WasButtonPressed(int32 Device, int32 Button) const
{
	return IsButtonDown(Device, Button) && ButtonsBefore.IsValidIndex(Device) && (ButtonsBefore[Device] & (1u << Button)) == 0;
}

bool FA320Joystick::WasButtonReleased(int32 Device, int32 Button) const
{
	return !IsButtonDown(Device, Button) && ButtonsBefore.IsValidIndex(Device) && Button >= 0 && Button < kButtons &&
		(ButtonsBefore[Device] & (1u << Button)) != 0;
}

FString FA320Joystick::ButtonCommand(int32 Device, int32 Button) const
{
	if (Button < 0 || Button >= kButtons || Device < 0)
	{
		return FString();
	}
	if (Device == GetStickDevice())
	{
		return FString(UTF8_TO_TCHAR(JoyConfig.buttons[Button].c_str()));
	}
	return Device == GetThrottleDevice() ? FString(UTF8_TO_TCHAR(JoyConfig.throttleButtons[Button].c_str())) : FString();
}

void FA320Joystick::SetBindingDevice(Binding& B, int32 Device) const
{
	B.device = Devices.IsValidIndex(Device) ? Device : -1;
	B.deviceName = Devices.IsValidIndex(Device) ? std::string(TCHAR_TO_UTF8(*Devices[Device].Name)) : std::string();
}

void FA320Joystick::ResetLeverCalibration(int32 F)
{
	// Detents are recorded per axis and direction, so a new axis or invert starts linear again.
	const int32 L = F == kThrottle ? 0 : (F == kThrottle2 ? 1 : -1);
	if (L >= 0)
	{
		JoyConfig.cal[L] = ThrottleCal{};
		Message = TEXT("Thrust lever changed: press CALIBRATE THRUST to teach it the detents.");
	}
}

void FA320Joystick::StartLearn(int32 F)
{
	CalStep = -1;
	Learning = F >= 0 && F < kFunctionCount ? F : -1;
	LearnBaseline = Axes;
}

void FA320Joystick::CycleAxis(int32 F)
{
	if (F < 0 || F >= kFunctionCount)
	{
		return;
	}
	// None -> each axis the devices actually have, device by device -> None.
	Binding& B = JoyConfig.bind[F];
	int32 Dev = B.device, Axis = B.device < 0 ? -1 : B.axis;
	for (;;)
	{
		if (Dev < 0)
		{
			Dev = 0;
			Axis = -1;
		}
		if (++Axis >= kAxes)
		{
			Axis = 0;
			++Dev;
		}
		if (Dev >= Devices.Num())
		{
			SetBindingDevice(B, -1);
			B.axis = 0;
			break;
		}
		if (Devices[Dev].HasAxis[Axis])
		{
			SetBindingDevice(B, Dev);
			B.axis = Axis;
			break;
		}
	}
	ResetLeverCalibration(F);
	Save();
}

void FA320Joystick::ToggleInvert(int32 F)
{
	if (F >= 0 && F < kFunctionCount)
	{
		JoyConfig.bind[F].invert = !JoyConfig.bind[F].invert;
		ResetLeverCalibration(F);
		Save();
	}
}

void FA320Joystick::StartCalibration()
{
	if (!IsBound(kThrottle))
	{
		Message = TEXT("Bind THRUST 1 to your throttle first (LEARN, then move the lever).");
		return;
	}
	Learning = -1;
	CalStep = kCalIdle;
	Message.Reset();
}

void FA320Joystick::CalibrationSet()
{
	if (CalStep < 0)
	{
		return;
	}
	CalRaw[0][CalStep] = axisValue(JoyConfig.bind[kThrottle], Axes);
	CalRaw[1][CalStep] = HasSecondLever() ? axisValue(JoyConfig.bind[kThrottle2], Axes) : CalRaw[0][CalStep];
	if (CalStep == kCalReverse)
	{
		FinishCalibration(true);
	}
	else
	{
		++CalStep;
	}
}

void FA320Joystick::CalibrationSkip()
{
	if (CalStep == kCalReverse)
	{
		FinishCalibration(false);
	}
}

void FA320Joystick::CancelCalibration()
{
	CalStep = -1;
	Message.Reset();
}

void FA320Joystick::FinishCalibration(bool bWithReverse)
{
	const int32 Count = HasSecondLever() ? 2 : 1;
	ThrottleCal Cal[2];
	bool bFlip[2] = {false, false};
	for (int32 L = 0; L < Count; ++L)
	{
		std::string Error;
		if (!buildCalibration(CalRaw[L], bWithReverse, Cal[L], bFlip[L], Error))
		{
			Message = FString::Printf(TEXT("Lever %d: %s"), L + 1, UTF8_TO_TCHAR(Error.c_str()));
			CalStep = kCalIdle;
			return;
		}
	}
	const Function Levers[2] = {kThrottle, kThrottle2};
	for (int32 L = 0; L < Count; ++L)
	{
		JoyConfig.cal[L] = Cal[L];
		if (bFlip[L])
		{
			JoyConfig.bind[Levers[L]].invert = !JoyConfig.bind[Levers[L]].invert;
		}
	}
	if (Count == 1)
	{
		JoyConfig.cal[1] = Cal[0];
	}
	CalStep = -1;
	LastThrottle[0] = LastThrottle[1] = 2.0;
	Message = bWithReverse ? TEXT("Thrust levers calibrated, with reverse.") : TEXT("Thrust levers calibrated (no reverse range: use the REVERSE key or a button).");
	Save();
}
