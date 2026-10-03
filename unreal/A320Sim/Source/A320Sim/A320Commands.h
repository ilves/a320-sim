#pragma once

#include "CoreMinimal.h"

// Discrete cockpit actions, shared by keyboard/gamepad bindings and the clickable panel.
enum class EA320Command : uint8
{
	None,
	GearToggle,
	FlapsUp,
	FlapsDown,
	SpeedbrakeToggle,
	ParkBrakeToggle,
	ReverseToggle,
	ThrustIdle,
	ThrustClimb,
	ThrustFlex,
	ThrustToga,
	LsToggle,
	NdRangeDown,
	NdRangeUp,
	PauseToggle,
	SimRateCycle,
	ViewToggle,
	HelpToggle,
	RunwaySwap,
	ResetRunway,
	ResetFinal10,
	ResetFinal4,
	// Autoflight (FCU) and sound.
	FcuAp,
	FcuAthr,
	FcuHdgPull,
	FcuLoc,
	FcuAppr,
	FcuAltPull,
	FcuVsPull,
	SpdDec,
	SpdInc,
	HdgDec,
	HdgInc,
	AltDec,
	AltInc,
	VsDec,
	VsInc,
	MasterWarnAck,
	SoundToggle,
	// Cockpit panels.
	ResetColdDark,
	OverheadToggle,
	NdModeToggle,
	EngMaster1,
	EngMaster2,
	EngModeCrank,
	EngModeNorm,
	EngModeIgnStart,
	ApuMaster,
	ApuStart,
	ApuBleed,
	AutobrakeLo,
	AutobrakeMed,
	AutobrakeMax,
	SpoilerArm,
	LightStrobe,
	LightBeacon,
	LightNav,
	LightLanding,
	LightNose,
	LightRwyTurnoff,
	SignSeatbelts,
	SignNoSmoking,
	// Joystick setup (handled by the player controller). Per-function commands are
	// contiguous: function index = command - first.
	JoystickPanel,
	JoyRescan,
	JoyLearn0, JoyLearn1, JoyLearn2, JoyLearn3, JoyLearn4, JoyLearn5, JoyLearn6,
	JoyAxis0, JoyAxis1, JoyAxis2, JoyAxis3, JoyAxis4, JoyAxis5, JoyAxis6,
	JoyInvert0, JoyInvert1, JoyInvert2, JoyInvert3, JoyInvert4, JoyInvert5, JoyInvert6,
	// Thrust lever detent calibration wizard.
	JoyCalStart,
	JoyCalSet,
	JoyCalSkip,
	JoyCalCancel,
	ApDisconnect,
};

// Levers dragged with the mouse; the value is the handle position, 0 at the top of its slot.
enum class EA320Lever : uint8
{
	None,
	Thrust,
	Flaps,
	Speedbrake,
};
