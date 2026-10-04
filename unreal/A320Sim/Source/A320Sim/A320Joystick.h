#pragma once

#include "CoreMinimal.h"
#include "a320/JoystickMapping.h"

#include <vector>

struct FA320JoystickDevice
{
	FString Name;
	int32 NumAxes = 0;
	int32 NumButtons = 0;
	bool HasAxis[a320::joy::kAxes] = {};
	bool bHasPov = false;
	void* Handle = nullptr;  // IDirectInputDevice8W
	int32 FailedPolls = 0;
	uint8 Down[a320::joy::kButtons] = {};
	uint8 WasDown[a320::joy::kButtons] = {};
	// Since the last poll, from DirectInput's event buffer: an encoder click shorter than a
	// frame still counts.
	uint8 Presses[a320::joy::kButtons] = {};
	uint8 Releases[a320::joy::kButtons] = {};
};

// USB joysticks, throttle quadrants, add-on modules, rudder pedals and FCU/EFIS panels through
// DirectInput (up to 8 axes and 128 buttons each). Xbox-type pads are left to Unreal's own
// gamepad support so they are not read twice. Settings persist in Saved/A320Joystick.ini.
class FA320Joystick
{
public:
	~FA320Joystick();
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
	// True for one poll after a lever axis moved, so it does not fight the keyboard and mouse.
	bool AxisMoved(a320::joy::Function Function) const { return Moved[Function]; }
	bool ThrottleMoved() const { return Moved[a320::joy::kThrottle] || Moved[a320::joy::kThrottle2]; }
	// Thrust lever 0 or 1 through its detent calibration; lever 1 is lever 0 unless THRUST 2 is bound.
	a320::joy::LeverPosition Lever(int32 Index) const;
	bool HasSecondLever() const { return IsBound(a320::joy::kThrottle2); }
	int32 FlapsLever() const;  // 0..4
	a320::joy::SpeedbrakePosition Speedbrake() const;

	// The stick (the PITCH device) and the throttle (the THRUST 1 device, -1 if the same or none).
	int32 GetStickDevice() const;
	int32 GetThrottleDevice() const;
	// The device a button binding refers to now, or -1 if it is not plugged in.
	int32 BindDevice(const a320::joy::ButtonBind& Bind) const;
	bool IsButtonDown(int32 Device, int32 Button) const;
	int32 ButtonPresses(int32 Device, int32 Button) const;
	bool WasButtonReleased(int32 Device, int32 Button) const;
	int32 GetPov() const { return Pov; }  // hat of the stick, hundredths of degrees, -1 = centred
	// Nothing should act on the buttons this poll (one was just assigned).
	bool ButtonsSuppressed() const { return bSuppressButtons; }
	// "TCA Q-Eng 1&2 #5" for each button bound to a command, for the setup panel.
	FString BindText(const char* Command) const;
	const FString& GetLastPressed() const { return LastPressed; }

	void StartLearn(int32 Function);
	int32 GetLearning() const { return Learning; }
	void CycleAxis(int32 Function);
	void ToggleInvert(int32 Function);
	// SET on the BUTTONS page: the next button pressed on any device gets the command.
	void StartButtonLearn(int32 CommandIndex);
	int32 GetButtonLearning() const { return ButtonLearning; }
	void ClearCommand(int32 CommandIndex);

	// Detent calibration (a320::joy::CalTarget): the lever is put in each detent in turn.
	void StartCalibration(int32 Target);
	void CalibrationSet();
	void CalibrationSkip();  // the optional last step: no reverse range / no ARM position
	void CancelCalibration();
	int32 GetCalibrationTarget() const { return CalTarget; }
	int32 GetCalibrationStep() const { return CalStep; }  // -1 = not calibrating
	const FString& GetMessage() const { return Message; }

private:
	void ReleaseDevices();
	void FinishCalibration(bool bWithLastStep);
	void ResetLeverCalibration(int32 Function);
	void SetBindingDevice(a320::joy::Binding& B, int32 Device) const;

	FString ConfigPath;
	a320::joy::Config JoyConfig = a320::joy::defaults();
	void* DirectInput = nullptr;  // IDirectInput8W
	TArray<FA320JoystickDevice> Devices;
	std::vector<a320::joy::AxisValues> Axes;
	std::vector<a320::joy::AxisValues> LearnBaseline;
	int32 Pov = -1;
	int32 Learning = -1;
	int32 ButtonLearning = -1;
	bool bSuppressButtons = false;
	FString LastPressed;
	double LastValue[a320::joy::kFunctionCount] = {};
	bool Moved[a320::joy::kFunctionCount] = {};
	bool bHaveLastValues = false;
	int32 CalTarget = -1;
	int32 CalStep = -1;
	double CalRaw[2][5] = {};
	FString Message;
	float RescanTimer = 0.0f;
};
