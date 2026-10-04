#pragma once

#include "CoreMinimal.h"
#include "a320/WingFlexReports.h"

class AA320Aircraft;
struct FA320HidLink;

// WingFlex FCU Cube and EFIS Cube panels, spoken to directly over USB HID: their buttons and
// knobs drive the FCU and EFIS, and their lights and displays show the sim's values. Close
// WingFlex Bridge while flying this sim, or both programs drive the displays.
class FA320WingFlex
{
public:
	FA320WingFlex();
	~FA320WingFlex();

	void Tick(AA320Aircraft& Aircraft, float DeltaSeconds);

	bool HasFcu() const;
	bool HasEfis() const;

private:
	void Scan();
	void HandleFcu(AA320Aircraft& Aircraft, const a320::wingflex::FcuInput& In);
	void HandleEfis(AA320Aircraft& Aircraft, const a320::wingflex::EfisInput& In);
	void SendOutputs(const AA320Aircraft& Aircraft, double Now);

	TUniquePtr<FA320HidLink> Fcu;
	TUniquePtr<FA320HidLink> Efis;
	a320::wingflex::FcuInput LastFcu;
	a320::wingflex::EfisInput LastEfis;
	bool bHaveFcuInput = false;
	bool bHaveEfisInput = false;
	uint8 Backlight = 0xC0;
	uint8 LcdBrightness = 0xC0;
	float ScanTimer = 0.0f;
	a320::wingflex::Payload LastFcuOut{};
	a320::wingflex::Payload LastEfisOut{};
};
