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
	// True for one poll after a thrust lever axis moved, so it does not fight the keyboard.
	bool ThrottleMoved() const { return bThrottleMoved; }
	// Thrust lever 0 or 1 through its detent calibration; lever 1 is lever 0 unless THRUST 2 is bound.
	a320::joy::LeverPosition Lever(int32 Index) const;
	bool HasSecondLever() const { return IsBound(a320::joy::kThrottle2); }

	// The stick (the PITCH device) and the throttle (the THRUST 1 device, -1 if the same or none).
	int32 GetStickDevice() const;
	int32 GetThrottleDevice() const;
	bool IsButtonDown(int32 Device, int32 Button) const;
	bool WasButtonPressed(int32 Device, int32 Button) const;
	bool WasButtonReleased(int32 Device, int32 Button) const;
	int32 GetPov() const { return Pov; }  // hat of the stick, hundredths of degrees, -1 = centred
	// Command name per button: buttonN for the stick, throttleButtonN for the throttle.
	FString ButtonCommand(int32 Device, int32 Button) const;

	void StartLearn(int32 Function);
	int32 GetLearning() const { return Learning; }
	void CycleAxis(int32 Function);
	void ToggleInvert(int32 Function);

	// Detent calibration: the levers are put in IDLE, CL, FLX/MCT, TOGA and full reverse in turn.
	void StartCalibration();
	void CalibrationSet();
	void CalibrationSkip();  // no reverse range
	void CancelCalibration();
	int32 GetCalibrationStep() const { return CalStep; }  // a320::joy::CalStep, -1 = not calibrating
	const FString& GetMessage() const { return Message; }

private:
	FString ConfigPath;
	a320::joy::Config JoyConfig = a320::joy::defaults();
	TArray<FA320JoystickDevice> Devices;
	std::vector<a320::joy::AxisValues> Axes;
	std::vector<a320::joy::AxisValues> LearnBaseline;
	TArray<uint32> ButtonsNow;
	TArray<uint32> ButtonsBefore;
	int32 Pov = -1;
	int32 Learning = -1;
	double LastThrottle[2] = {2.0, 2.0};
	bool bThrottleMoved = false;
	bool bFirstPoll = true;
	int32 CalStep = -1;
	double CalRaw[2][a320::joy::kCalSteps] = {};
	FString Message;

	void FinishCalibration(bool bWithReverse);
	void ResetLeverCalibration(int32 Function);
	void SetBindingDevice(a320::joy::Binding& B, int32 Device) const;
	float RescanTimer = 0.0f;
};
