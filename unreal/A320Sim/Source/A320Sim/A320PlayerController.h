#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"

#include "A320Commands.h"
#include "A320Joystick.h"
#include "A320WingFlex.h"

#include "A320PlayerController.generated.h"

// Polls keyboard, gamepad and mouse every frame (no input assets needed) and drives the
// aircraft; mouse clicks go to the cockpit panel drawn by AA320Hud.
UCLASS()
class AA320PlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AA320PlayerController();

	virtual void PlayerTick(float DeltaTime) override;

	const FA320Joystick& GetJoystick() const { return Joystick; }
	const FA320WingFlex& GetWingFlex() const { return WingFlex; }
	bool IsJoystickPanelVisible() const { return bJoystickPanel; }
	bool IsJoystickButtonsPage() const { return bJoystickButtonsPage; }

protected:
	virtual void BeginPlay() override;

private:
	double KeyStickPitch = 0.0;
	double KeyStickRoll = 0.0;
	double KeyPedals = 0.0;
	FVector2D LastMouse = FVector2D::ZeroVector;
	EA320Lever DraggedLever = EA320Lever::None;
	FA320Joystick Joystick;
	FA320WingFlex WingFlex;
	bool bJoystickPanel = false;
	bool bJoystickButtonsPage = false;
	bool bAltStep1000 = false;  // FCU ALT knob step, from a hardware 100/1000 switch

	// Joystick setup commands are handled here; everything else goes to the aircraft.
	// Param: the command index for JoyBindSet / JoyBindClear.
	bool HandleJoystickCommand(EA320Command Command, int32 Param = 0);
	// MCDU keyboard entry while the MCDU is open.
	bool IsMcduTypingKey(const FKey& Key) const;
	void TypeIntoMcdu(class AA320Aircraft& Aircraft);
	// One hardware button's command (see a320::joy::commandCatalog) for this frame.
	void ApplyHardwareCommand(class AA320Aircraft* Aircraft, struct FA320FlightInputs& Inputs, const FString& Name,
		int32 Presses, bool bReleased, bool bDown);
	void ApplyJoystickButtons(class AA320Aircraft* Aircraft, struct FA320FlightInputs& Inputs, float DeltaTime);
	bool bLooking = false;
};
