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
	// Approach scenario, second autopilot and lessons (guides).
	ResetApproach,
	FcuAp2,
	GuideMenu,
	GuideNext,
	GuideBack,
	GuideStop,
	GuideStart0, GuideStart1, GuideStart2, GuideStart3,
	// Joystick setup (handled by the player controller). Per-function commands are
	// contiguous: function index = command - first.
	JoystickPanel,
	JoyRescan,
	JoyLearn0, JoyLearn1, JoyLearn2, JoyLearn3, JoyLearn4, JoyLearn5, JoyLearn6, JoyLearn7, JoyLearn8,
	JoyAxis0, JoyAxis1, JoyAxis2, JoyAxis3, JoyAxis4, JoyAxis5, JoyAxis6, JoyAxis7, JoyAxis8,
	JoyInvert0, JoyInvert1, JoyInvert2, JoyInvert3, JoyInvert4, JoyInvert5, JoyInvert6, JoyInvert7, JoyInvert8,
	// Detent calibration wizards (thrust levers, flaps lever, speedbrake lever).
	JoyCalStart,
	JoyCalFlaps,
	JoyCalSpeedbrake,
	JoyCalSet,
	JoyCalSkip,
	JoyCalCancel,
	// Setup panel pages, and assigning hardware buttons (the button carries the command index).
	JoyPageAxes,
	JoyPageButtons,
	JoyBindSet,
	JoyBindClear,
	FcuVsPush,
	ApDisconnect,
	// MCDU pop-up; McduKey buttons carry the key code (A320McduKey or an ASCII character).
	McduToggle,
	McduKey,
	// RADIO window: VHF 1 radio management panel, transponder and the ATC replies (buttons carry
	// a digit, a mode or a reply number in their Param).
	RadioToggle,
	ComSwap,
	ComMhzDec,
	ComMhzInc,
	ComKhzDec,
	ComKhzInc,
	XpdrDigit,
	XpdrClear,
	XpdrMode,
	AtcToggle,
	AtcReply,
	// FLIGHT menu: departure and arrival runway (Param: runway index), how the flight starts
	// (Param: A320Scenario), the distance out for an airborne start (Param: NM), and go.
	FlightMenu,
	FlightDep,
	FlightArr,
	FlightStart,
	FlightDistance,
	FlightGo,
	FlightPlan,  // Param: A320FlightPlan (entered or not)
};

// Levers dragged with the mouse; the value is the handle position, 0 at the top of its slot.
enum class EA320Lever : uint8
{
	None,
	Thrust,
	Flaps,
	Speedbrake,
};

// Cockpit switches and levers set to a position (hardware switches and selectors), rather
// than toggled like the panel's buttons.
enum class EA320Switch : uint8
{
	Gear,         // 0 up, 1 down
	ParkBrake,    // 0/1
	SpoilersArm,  // 0/1
	Autobrake,    // A320_AUTOBRAKE_*
	EngMaster1,   // 0/1
	EngMaster2,   // 0/1
	EngMode,      // A320_ENG_MODE_*
	Flaps,        // 0..4
	NdMode,       // A320_ND_*
	NdRange,      // NM
};
