#pragma once

#include "CoreMinimal.h"
#include "a320/JoystickMapping.h"

#include <vector>

struct FA320JoystickDevice
{
	int32 SystemId = 0;  // Windows joystick id (0-15)
	FString Name;
	int32 NumAxes = 0;
	int32 NumButtons = 0;
	uint32 AxisMin[a320::joy::kAxes] = {};
	uint32 AxisMax[a320::joy::kAxes] = {};
	bool HasAxis[a320::joy::kAxes] = {};  // from the capability flags, not the axis count
	bool bHasPov = false;
};

// USB joysticks, throttles and rudder pedals through the Windows joystick API (winmm), which
// sees any DirectInput/HID game controller. Xbox-type pads are left to Unreal's own gamepad
// support so they are not read twice. Settings persist in Saved/A320Joystick.ini.
class FA320Joystick
{
public:
	void Init(const FString& InConfigPath);
	void Poll(float DeltaSeconds);
	void Rescan();
	void Save() const;

	bool HasDevice() const { return Devices.Num() > 0; }
	const TArray<FA320JoystickDevice>& GetDevices() const { return Devices; }
	const std::vector<a320::joy::AxisValues>& GetAxes() const { return Axes; }
	const a320::joy::Config& GetConfig() const { return JoyConfig; }
	const FString& GetConfigPath() const { return ConfigPath; }

	// Processed value of a function (-1..1, deadzone applied to the flight controls).
	double Value(a320::joy::Function Function) const;
	bool IsBound(a320::joy::Function Function) const;
	// True for one poll after the throttle axis moved, so it does not fight the keyboard.
	bool ThrottleMoved() const { return bThrottleMoved; }

	// Buttons and hat of the device bound to PITCH (the main stick).
	bool IsButtonDown(int32 Button) const;
	bool WasButtonPressed(int32 Button) const;
	int32 GetPov() const { return Pov; }  // hundredths of degrees, -1 = centred
	FString ButtonCommand(int32 Button) const;

	void StartLearn(int32 Function);
	int32 GetLearning() const { return Learning; }
	void CycleAxis(int32 Function);
	void ToggleInvert(int32 Function);

private:
	FString ConfigPath;
	a320::joy::Config JoyConfig = a320::joy::defaults();
	TArray<FA320JoystickDevice> Devices;
	std::vector<a320::joy::AxisValues> Axes;
	std::vector<a320::joy::AxisValues> LearnBaseline;
	uint32 ButtonsNow = 0;
	uint32 ButtonsBefore = 0;
	int32 Pov = -1;
	int32 Learning = -1;
	double LastThrottle = 2.0;
	bool bThrottleMoved = false;
	float RescanTimer = 0.0f;
};
