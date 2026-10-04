#include "A320Joystick.h"

#include "A320Sim.h"
#include "Misc/FileHelper.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include "Windows/AllowWindowsPlatformTypes.h"
#ifndef DIRECTINPUT_VERSION
#define DIRECTINPUT_VERSION 0x0800
#endif
#include <dinput.h>
#endif

using namespace a320::joy;

namespace
{
#if PLATFORM_WINDOWS
	IDirectInput8W* AsApi(void* P) { return static_cast<IDirectInput8W*>(P); }
	IDirectInputDevice8W* AsDevice(void* P) { return static_cast<IDirectInputDevice8W*>(P); }

	BOOL CALLBACK OnEnumDevice(LPCDIDEVICEINSTANCEW Instance, LPVOID Context)
	{
		static_cast<TArray<DIDEVICEINSTANCEW>*>(Context)->Add(*Instance);
		return DIENUM_CONTINUE;
	}

	// Which of our 8 axes (X Y Z RZ RX RY SL0 SL1) the device has.
	struct FAxisPresence
	{
		bool Has[kAxes] = {};
		int32 Sliders = 0;
	};

	BOOL CALLBACK OnEnumAxis(LPCDIDEVICEOBJECTINSTANCEW Object, LPVOID Context)
	{
		FAxisPresence* P = static_cast<FAxisPresence*>(Context);
		const GUID& G = Object->guidType;
		if (G == GUID_XAxis) P->Has[0] = true;
		else if (G == GUID_YAxis) P->Has[1] = true;
		else if (G == GUID_ZAxis) P->Has[2] = true;
		else if (G == GUID_RzAxis) P->Has[3] = true;
		else if (G == GUID_RxAxis) P->Has[4] = true;
		else if (G == GUID_RyAxis) P->Has[5] = true;
		else if (G == GUID_Slider)
		{
			if (P->Sliders < 2)
			{
				P->Has[6 + P->Sliders] = true;
			}
			++P->Sliders;
		}
		return DIENUM_CONTINUE;
	}
#endif

	uint8 AddCount(uint8 Count) { return Count < 255 ? static_cast<uint8>(Count + 1) : Count; }
}

FA320Joystick::~FA320Joystick()
{
	ReleaseDevices();
#if PLATFORM_WINDOWS
	if (DirectInput)
	{
		AsApi(DirectInput)->Release();
		DirectInput = nullptr;
	}
#endif
}

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

void FA320Joystick::ReleaseDevices()
{
#if PLATFORM_WINDOWS
	for (FA320JoystickDevice& Device : Devices)
	{
		if (Device.Handle)
		{
			AsDevice(Device.Handle)->Unacquire();
			AsDevice(Device.Handle)->Release();
			Device.Handle = nullptr;
		}
	}
#endif
	Devices.Reset();
}

void FA320Joystick::Rescan()
{
	ReleaseDevices();
#if PLATFORM_WINDOWS
	if (!DirectInput)
	{
		IDirectInput8W* Created = nullptr;
		if (SUCCEEDED(DirectInput8Create(GetModuleHandleW(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8W,
				reinterpret_cast<void**>(&Created), nullptr)))
		{
			DirectInput = Created;
		}
		else
		{
			UE_LOG(LogA320, Warning, TEXT("DirectInput is not available: no joysticks"));
		}
	}
	if (DirectInput)
	{
		TArray<DIDEVICEINSTANCEW> Found;
		AsApi(DirectInput)->EnumDevices(DI8DEVCLASS_GAMECTRL, OnEnumDevice, &Found, DIEDFL_ATTACHEDONLY);
		for (const DIDEVICEINSTANCEW& Instance : Found)
		{
			FA320JoystickDevice Device;
			Device.Name = FString(Instance.tszProductName).TrimStartAndEnd();
			// Xbox/XInput pads are already Unreal gamepads; reading them here too would double them.
			if (Device.Name.Contains(TEXT("xbox")) || Device.Name.Contains(TEXT("xinput")))
			{
				UE_LOG(LogA320, Log, TEXT("Skipping %s: used as a gamepad"), *Device.Name);
				continue;
			}
			IDirectInputDevice8W* Handle = nullptr;
			if (FAILED(AsApi(DirectInput)->CreateDevice(Instance.guidInstance, &Handle, nullptr)) || !Handle)
			{
				continue;
			}
			Handle->SetDataFormat(&c_dfDIJoystick2);
			// Background and non-exclusive: read it even when another window has the focus, and
			// share it with other programs (e.g. a panel's own bridge software).
			if (FAILED(Handle->SetCooperativeLevel(nullptr, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE)))
			{
				Handle->SetCooperativeLevel(GetActiveWindow(), DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);
			}
			DIPROPRANGE Range = {};
			Range.diph.dwSize = sizeof(DIPROPRANGE);
			Range.diph.dwHeaderSize = sizeof(DIPROPHEADER);
			Range.diph.dwHow = DIPH_DEVICE;
			Range.lMin = 0;
			Range.lMax = 65535;
			Handle->SetProperty(DIPROP_RANGE, &Range.diph);
			DIPROPDWORD BufferSize = {};
			BufferSize.diph.dwSize = sizeof(DIPROPDWORD);
			BufferSize.diph.dwHeaderSize = sizeof(DIPROPHEADER);
			BufferSize.diph.dwHow = DIPH_DEVICE;
			BufferSize.dwData = 256;
			Handle->SetProperty(DIPROP_BUFFERSIZE, &BufferSize.diph);
			FAxisPresence Presence;
			Handle->EnumObjects(OnEnumAxis, &Presence, DIDFT_AXIS);
			DIDEVCAPS Caps = {};
			Caps.dwSize = sizeof(DIDEVCAPS);
			Handle->GetCapabilities(&Caps);
			Device.NumAxes = static_cast<int32>(Caps.dwAxes);
			Device.NumButtons = FMath::Min(static_cast<int32>(Caps.dwButtons), kButtons);
			Device.bHasPov = Caps.dwPOVs > 0;
			for (int32 A = 0; A < kAxes; ++A)
			{
				Device.HasAxis[A] = Presence.Has[A];
			}
			Handle->Acquire();
			Device.Handle = Handle;
			UE_LOG(LogA320, Log, TEXT("Joystick %d: %s (%d axes, %d buttons)"), Devices.Num(), *Device.Name, Device.NumAxes, Device.NumButtons);
			Devices.Add(Device);
		}
	}
#endif
	Axes.assign(static_cast<size_t>(Devices.Num()), AxisValues{});
	bHaveLastValues = false;

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
		Message = FString::Printf(TEXT("Throttle found: %s. Press CAL next to THRUST 1 to teach it the detents."),
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
	bSuppressButtons = false;
	if (Devices.Num() == 0)
	{
		// Plugged in later? Scanning is slow-ish, so only every few seconds.
		RescanTimer -= DeltaSeconds;
		if (RescanTimer <= 0.0f)
		{
			RescanTimer = 5.0f;
			Rescan();
		}
		for (bool& M : Moved)
		{
			M = false;
		}
		return;
	}
	Pov = -1;
	const int32 StickDevice = GetStickDevice();
	bool bLost = false;
	for (int32 D = 0; D < Devices.Num(); ++D)
	{
		FA320JoystickDevice& Device = Devices[D];
		FMemory::Memcpy(Device.WasDown, Device.Down, sizeof(Device.Down));
		FMemory::Memzero(Device.Presses, sizeof(Device.Presses));
		FMemory::Memzero(Device.Releases, sizeof(Device.Releases));
#if PLATFORM_WINDOWS
		IDirectInputDevice8W* Handle = AsDevice(Device.Handle);
		if (!Handle)
		{
			continue;
		}
		Handle->Poll();
		DIJOYSTATE2 Js = {};
		if (FAILED(Handle->GetDeviceState(sizeof(DIJOYSTATE2), &Js)))
		{
			// Focus changes and USB hiccups lose the device for a moment; unplugged stays lost.
			Handle->Acquire();
			if (++Device.FailedPolls > 120)
			{
				bLost = true;
			}
			continue;
		}
		Device.FailedPolls = 0;
		const LONG Raw[kAxes] = {Js.lX, Js.lY, Js.lZ, Js.lRz, Js.lRx, Js.lRy, Js.rglSlider[0], Js.rglSlider[1]};
		for (int32 A = 0; A < kAxes; ++A)
		{
			Axes[static_cast<size_t>(D)][static_cast<size_t>(A)] =
				Device.HasAxis[A] ? normalize(static_cast<uint32_t>(FMath::Clamp<LONG>(Raw[A], 0, 65535)), 0, 65535) : 0.0;
		}
		for (int32 B = 0; B < kButtons; ++B)
		{
			Device.Down[B] = (Js.rgbButtons[B] & 0x80) != 0 ? 1 : 0;
		}
		if (D == StickDevice)
		{
			// Hundredths of a degree; the low word 0xFFFF means centred.
			Pov = Device.bHasPov && LOWORD(Js.rgdwPOV[0]) != 0xFFFF ? static_cast<int32>(Js.rgdwPOV[0]) : -1;
		}
		DIDEVICEOBJECTDATA Events[64];
		for (;;)
		{
			DWORD Count = UE_ARRAY_COUNT(Events);
			if (FAILED(Handle->GetDeviceData(sizeof(DIDEVICEOBJECTDATA), Events, &Count, 0)) || Count == 0)
			{
				break;
			}
			for (DWORD i = 0; i < Count; ++i)
			{
				const DWORD Offset = Events[i].dwOfs;
				if (Offset >= DIJOFS_BUTTON0 && Offset < DIJOFS_BUTTON0 + kButtons)
				{
					const int32 B = static_cast<int32>(Offset - DIJOFS_BUTTON0);
					if (Events[i].dwData & 0x80)
					{
						Device.Presses[B] = AddCount(Device.Presses[B]);
					}
					else
					{
						Device.Releases[B] = AddCount(Device.Releases[B]);
					}
				}
			}
			if (Count < UE_ARRAY_COUNT(Events))
			{
				break;
			}
		}
#endif
		// Devices without an event buffer: changes from the state alone.
		for (int32 B = 0; B < kButtons; ++B)
		{
			if (Device.Down[B] && !Device.WasDown[B] && Device.Presses[B] == 0)
			{
				Device.Presses[B] = 1;
			}
			if (!Device.Down[B] && Device.WasDown[B] && Device.Releases[B] == 0)
			{
				Device.Releases[B] = 1;
			}
			// Both edges, so a switch that only closes in one position shows that too.
			if (Device.Presses[B] > 0)
			{
				LastPressed = FString::Printf(TEXT("%s: button %d pressed"), *Device.Name, B + 1);
			}
			else if (Device.Releases[B] > 0)
			{
				LastPressed = FString::Printf(TEXT("%s: button %d released"), *Device.Name, B + 1);
			}
		}
	}
	if (bLost)
	{
		Rescan();  // unplugged
		return;
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

	if (ButtonLearning >= 0)
	{
		for (int32 D = 0; D < Devices.Num() && ButtonLearning >= 0; ++D)
		{
			for (int32 B = 0; B < kButtons; ++B)
			{
				if (Devices[D].Presses[B] > 0)
				{
					const CommandInfo& Info = commandCatalog()[static_cast<size_t>(ButtonLearning)];
					setBind(JoyConfig, TCHAR_TO_UTF8(*Devices[D].Name), B, Info.name);
					Message = FString::Printf(TEXT("%s = %s button %d"), UTF8_TO_TCHAR(Info.label), *Devices[D].Name, B + 1);
					ButtonLearning = -1;
					bSuppressButtons = true;  // the press that assigned it does not also act
					Save();
					break;
				}
			}
		}
	}

	// Levers: the first reading only sets the reference, so their resting position does not
	// override the scenario at start-up. Calibrating moves them through every detent.
	for (int32 F = 0; F < kFunctionCount; ++F)
	{
		Moved[F] = false;
		if (!IsBound(static_cast<Function>(F)))
		{
			continue;
		}
		const double V = axisValue(JoyConfig.bind[F], Axes);
		const double Threshold = F == kFlaps ? 0.03 : 0.01;
		if (!bHaveLastValues || FMath::Abs(V - LastValue[F]) > Threshold)
		{
			Moved[F] = bHaveLastValues && CalStep < 0;
			LastValue[F] = V;
		}
	}
	bHaveLastValues = true;
}

LeverPosition FA320Joystick::Lever(int32 Index) const
{
	const int32 L = Index == 1 && HasSecondLever() ? 1 : 0;
	return throttleFromAxis(Value(L == 1 ? kThrottle2 : kThrottle), JoyConfig.cal[L]);
}

int32 FA320Joystick::FlapsLever() const
{
	return flapsFromAxis(Value(kFlaps), JoyConfig.flapsCal);
}

SpeedbrakePosition FA320Joystick::Speedbrake() const
{
	return speedbrakeFromAxis(Value(kSpeedbrake), JoyConfig.speedbrakeCal);
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

int32 FA320Joystick::BindDevice(const ButtonBind& Bind) const
{
	if (Bind.device == "@stick")
	{
		return GetStickDevice();
	}
	if (Bind.device == "@throttle")
	{
		return GetThrottleDevice();
	}
	const FString Name = UTF8_TO_TCHAR(Bind.device.c_str());
	for (int32 D = 0; D < Devices.Num(); ++D)
	{
		if (Devices[D].Name == Name)
		{
			return D;
		}
	}
	return -1;
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
	return Devices.IsValidIndex(Device) && Button >= 0 && Button < kButtons && Devices[Device].Down[Button] != 0;
}

int32 FA320Joystick::ButtonPresses(int32 Device, int32 Button) const
{
	return Devices.IsValidIndex(Device) && Button >= 0 && Button < kButtons ? Devices[Device].Presses[Button] : 0;
}

bool FA320Joystick::WasButtonReleased(int32 Device, int32 Button) const
{
	return Devices.IsValidIndex(Device) && Button >= 0 && Button < kButtons && Devices[Device].Releases[Button] > 0;
}

FString FA320Joystick::BindText(const char* Command) const
{
	FString Text;
	for (const ButtonBind& B : JoyConfig.binds)
	{
		if (B.command != Command)
		{
			continue;
		}
		const FString Device = B.device == "@stick" ? FString(TEXT("stick")) : (B.device == "@throttle" ? FString(TEXT("throttle")) : FString(UTF8_TO_TCHAR(B.device.c_str())));
		Text += FString::Printf(TEXT("%s%s #%d"), Text.IsEmpty() ? TEXT("") : TEXT(", "), *Device.Left(14), B.button + 1);
	}
	return Text;
}

void FA320Joystick::SetBindingDevice(Binding& B, int32 Device) const
{
	B.device = Devices.IsValidIndex(Device) ? Device : -1;
	B.deviceName = Devices.IsValidIndex(Device) ? std::string(TCHAR_TO_UTF8(*Devices[Device].Name)) : std::string();
}

void FA320Joystick::ResetLeverCalibration(int32 F)
{
	// Detents are recorded per axis and direction, so a new axis or invert starts linear again.
	if (F == kThrottle || F == kThrottle2)
	{
		JoyConfig.cal[F == kThrottle ? 0 : 1] = ThrottleCal{};
	}
	else if (F == kFlaps)
	{
		JoyConfig.flapsCal = FlapsCal{};
	}
	else if (F == kSpeedbrake)
	{
		JoyConfig.speedbrakeCal = SpeedbrakeCal{};
	}
	else
	{
		return;
	}
	Message = FString::Printf(TEXT("%s changed: press CAL to teach it the detents."), UTF8_TO_TCHAR(functionName(F)));
}

void FA320Joystick::StartLearn(int32 F)
{
	CalStep = -1;
	ButtonLearning = -1;
	Learning = F >= 0 && F < kFunctionCount ? F : -1;
	LearnBaseline = Axes;
}

void FA320Joystick::StartButtonLearn(int32 CommandIndex)
{
	Learning = -1;
	ButtonLearning = CommandIndex >= 0 && CommandIndex < static_cast<int32>(commandCatalog().size()) ? CommandIndex : -1;
	if (ButtonLearning >= 0)
	{
		Message = FString::Printf(TEXT("Press the button or move the switch for: %s"),
			UTF8_TO_TCHAR(commandCatalog()[static_cast<size_t>(ButtonLearning)].label));
	}
}

void FA320Joystick::ClearCommand(int32 CommandIndex)
{
	if (CommandIndex >= 0 && CommandIndex < static_cast<int32>(commandCatalog().size()))
	{
		clearBinds(JoyConfig, commandCatalog()[static_cast<size_t>(CommandIndex)].name);
		Save();
	}
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

void FA320Joystick::StartCalibration(int32 Target)
{
	const Function LeverFunction = Target == kCalFlaps ? kFlaps : (Target == kCalSpeedbrake ? kSpeedbrake : kThrottle);
	if (!IsBound(LeverFunction))
	{
		Message = FString::Printf(TEXT("Bind %s to your lever first (LEARN, then move it)."), UTF8_TO_TCHAR(functionName(LeverFunction)));
		return;
	}
	Learning = -1;
	ButtonLearning = -1;
	CalTarget = Target;
	CalStep = 0;
	Message.Reset();
}

void FA320Joystick::CalibrationSet()
{
	if (CalStep < 0)
	{
		return;
	}
	if (CalTarget == kCalThrust)
	{
		CalRaw[0][CalStep] = axisValue(JoyConfig.bind[kThrottle], Axes);
		CalRaw[1][CalStep] = HasSecondLever() ? axisValue(JoyConfig.bind[kThrottle2], Axes) : CalRaw[0][CalStep];
	}
	else
	{
		CalRaw[0][CalStep] = axisValue(JoyConfig.bind[CalTarget == kCalFlaps ? kFlaps : kSpeedbrake], Axes);
	}
	if (CalStep == calStepCount(CalTarget) - 1)
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
	if (CalStep >= 0 && calStepOptional(CalTarget, CalStep))
	{
		FinishCalibration(false);
	}
}

void FA320Joystick::CancelCalibration()
{
	CalStep = -1;
	CalTarget = -1;
	Message.Reset();
}

void FA320Joystick::FinishCalibration(bool bWithLastStep)
{
	std::string Error;
	bool bFlip = false;
	if (CalTarget == kCalFlaps)
	{
		FlapsCal Cal;
		if (!buildFlapsCal(CalRaw[0], Cal, bFlip, Error))
		{
			Message = UTF8_TO_TCHAR(Error.c_str());
			CalStep = 0;
			return;
		}
		JoyConfig.flapsCal = Cal;
		JoyConfig.bind[kFlaps].invert = JoyConfig.bind[kFlaps].invert != bFlip;
		Message = TEXT("Flaps lever calibrated.");
	}
	else if (CalTarget == kCalSpeedbrake)
	{
		SpeedbrakeCal Cal;
		if (!buildSpeedbrakeCal(CalRaw[0][0], CalRaw[0][1], bWithLastStep, CalRaw[0][2], Cal, bFlip, Error))
		{
			Message = UTF8_TO_TCHAR(Error.c_str());
			CalStep = 0;
			return;
		}
		JoyConfig.speedbrakeCal = Cal;
		JoyConfig.bind[kSpeedbrake].invert = JoyConfig.bind[kSpeedbrake].invert != bFlip;
		Message = bWithLastStep ? TEXT("Speedbrake lever calibrated, with ARM.") : TEXT("Speedbrake lever calibrated.");
	}
	else
	{
		const int32 Count = HasSecondLever() ? 2 : 1;
		ThrottleCal Cal[2];
		bool bFlips[2] = {false, false};
		for (int32 L = 0; L < Count; ++L)
		{
			if (!buildCalibration(CalRaw[L], bWithLastStep, Cal[L], bFlips[L], Error))
			{
				Message = FString::Printf(TEXT("Lever %d: %s"), L + 1, UTF8_TO_TCHAR(Error.c_str()));
				CalStep = 0;
				return;
			}
		}
		const Function Levers[2] = {kThrottle, kThrottle2};
		for (int32 L = 0; L < Count; ++L)
		{
			JoyConfig.cal[L] = Cal[L];
			JoyConfig.bind[Levers[L]].invert = JoyConfig.bind[Levers[L]].invert != bFlips[L];
		}
		if (Count == 1)
		{
			JoyConfig.cal[1] = Cal[0];
		}
		Message = bWithLastStep ? TEXT("Thrust levers calibrated, with reverse.")
			: TEXT("Thrust levers calibrated (no reverse range: use the REVERSE key or a button).");
	}
	CalStep = -1;
	CalTarget = -1;
	bHaveLastValues = false;
	Save();
}

#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif
