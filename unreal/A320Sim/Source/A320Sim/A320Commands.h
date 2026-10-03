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
};
