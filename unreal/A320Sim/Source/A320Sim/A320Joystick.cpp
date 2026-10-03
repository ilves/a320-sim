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
	LastThrottle = 2.0;
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
	ButtonsNow = 0;
	Pov = -1;
	const int32 MainDevice = FMath::Clamp(JoyConfig.bind[kPitch].device, 0, Devices.Num() - 1);
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
		if (D == MainDevice)
		{
			ButtonsNow = static_cast<uint32>(Info.dwButtons);
			// Hundredths of a degree; anything outside 0..35999 (e.g. 65535) means centred.
			Pov = Devices[D].bHasPov && Info.dwPOV <= 35999 ? static_cast<int32>(Info.dwPOV) : -1;
		}
	}
#endif

	if (Learning >= 0)
	{
		int Device = -1, Axis = -1;
		if (detectAxis(LearnBaseline, Axes, Device, Axis))
		{
			JoyConfig.bind[Learning].device = Device;
			JoyConfig.bind[Learning].axis = Axis;
			Learning = -1;
			Save();
		}
	}

	bThrottleMoved = false;
	if (IsBound(kThrottle))
	{
		const double Throttle = axisValue(JoyConfig.bind[kThrottle], Axes);
		bThrottleMoved = LastThrottle <= 1.5 && FMath::Abs(Throttle - LastThrottle) > 0.01;
		if (LastThrottle > 1.5 || bThrottleMoved)
		{
			LastThrottle = Throttle;
		}
	}
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

bool FA320Joystick::IsButtonDown(int32 Button) const
{
	return Button >= 0 && Button < kButtons && (ButtonsNow & (1u << Button)) != 0;
}

bool FA320Joystick::WasButtonPressed(int32 Button) const
{
	return IsButtonDown(Button) && (ButtonsBefore & (1u << Button)) == 0;
}

FString FA320Joystick::ButtonCommand(int32 Button) const
{
	return Button >= 0 && Button < kButtons ? FString(UTF8_TO_TCHAR(JoyConfig.buttons[Button].c_str())) : FString();
}

void FA320Joystick::StartLearn(int32 F)
{
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
			B.device = -1;
			B.axis = 0;
			break;
		}
		if (Devices[Dev].HasAxis[Axis])
		{
			B.device = Dev;
			B.axis = Axis;
			break;
		}
	}
	Save();
}

void FA320Joystick::ToggleInvert(int32 F)
{
	if (F >= 0 && F < kFunctionCount)
	{
		JoyConfig.bind[F].invert = !JoyConfig.bind[F].invert;
		Save();
	}
}
