#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"

#include "A320Commands.h"
#include "a320/a320_api.h"

#include "A320Hud.generated.h"

class AA320Aircraft;
class UFont;

// Glass cockpit drawn on the canvas: PFD, ND (map), E/WD, a clickable panel of buttons,
// and the callout/warning/pause overlays.
UCLASS()
class AA320Hud : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;

	// The panel button under a screen position (from the last drawn frame).
	EA320Command CommandAt(const FVector2D& ScreenPos) const;
	// The extra value of that button (e.g. which command a SET button assigns), 0 if none.
	int32 ParamAt(const FVector2D& ScreenPos) const;
	// The lever slot under a screen position, and the handle position (0 = top) for a drag.
	EA320Lever LeverAt(const FVector2D& ScreenPos) const;
	double LeverPosition(EA320Lever Lever, const FVector2D& ScreenPos) const;

private:
	struct FButton
	{
		FBox2D Box;
		EA320Command Command;
		int32 Param = 0;
	};

	void DrawPfd(const AA320Aircraft& Aircraft, double X, double Y, double S);
	void DrawNd(const AA320Aircraft& Aircraft, double X, double Y, double S);
	void DrawEwd(const AA320Aircraft& Aircraft, double X, double Y, double S);
	struct FLeverSlot
	{
		FBox2D Box;
		EA320Lever Lever;
	};

	void DrawCenterPanel(const AA320Aircraft& Aircraft, double X, double Y, double W, double H);
	void DrawPedestal(const AA320Aircraft& Aircraft, double X, double Y, double W, double H);
	void DrawSimBar(const AA320Aircraft& Aircraft);
	void DrawOverhead(const AA320Aircraft& Aircraft);
	void DrawJoystickPanel(const class AA320PlayerController& Controller);
	// First-start progress (shader/asset compilation) instead of a black screen; also logs
	// progress lines and "READY" for the launcher window (scripts/play.ps1).
	void DrawLoadingStatus(const AA320Aircraft* Aircraft);
	// Airbus-style pushbutton: upper legend (e.g. FAULT/AVAIL) and lower legend (e.g. ON).
	void Pushbutton(double X, double Y, double W, double H, const FString& Name, const FString& Upper,
		const FLinearColor& UpperColor, const FString& Lower, const FLinearColor& LowerColor, EA320Command Command);
	// Toggle/rotary switch: shows the current position; clicking moves it to the next one.
	void Switch(double X, double Y, double W, double H, const FString& Name, const FString& Position, EA320Command Command);
	void LeverSlot(double X, double Y, double W, double H, EA320Lever Lever);
	void DrawFcu(const AA320Aircraft& Aircraft, double X, double Y, double W, double H);
	void DrawFma(const A320State& St, double X, double Y, double S);
	void AddButton(double X, double Y, double W, double H, const FString& Label, EA320Command Command, bool bLit, int32 Param = 0);
	void DrawOverlays(const AA320Aircraft& Aircraft);
	void DrawHelp();
	// Lessons: the picker, and the step panel with the current step's controls highlighted.
	void DrawGuideMenu();
	void DrawGuide(const AA320Aircraft& Aircraft);
	// Remembers where a control or display the guides can point at was drawn this frame.
	void MarkTarget(int32 Target, double X, double Y, double W, double H);
	// Word-wrapped text from Y (top) down; returns the height used. bDraw = false only measures.
	double TextWrapped(const FString& Str, double X, double Y, double MaxW, const FLinearColor& Color, int32 Size, bool bDraw = true);

	void Line(double X1, double Y1, double X2, double Y2, const FLinearColor& Color, double Thickness = 1.5);
	void ClippedLine(double X1, double Y1, double X2, double Y2, double CX0, double CY0, double CX1, double CY1,
		const FLinearColor& Color, double Thickness = 1.5);
	void Fill(double X, double Y, double W, double H, const FLinearColor& Color);
	void Frame(double X, double Y, double W, double H, const FLinearColor& Color, double Thickness = 1.0);
	void Triangle(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FLinearColor& Color);
	void Arc(double CX, double CY, double R, double FromDeg, double ToDeg, const FLinearColor& Color, double Thickness = 1.5);
	// Align: 0 = left, 1 = centre, 2 = right; Y is the text's vertical centre.
	void Text(const FString& Str, double X, double Y, const FLinearColor& Color, int32 Size = 1, int32 Align = 0);

	UFont* FontFor(int32 Size) const;

	TArray<FButton> Buttons;
	TArray<FLeverSlot> Levers;
	int32 MaxPending = 0;
	double NextStatusLog = 0.0;
	double QuietSince = -1.0;
	bool bReadyLogged = false;
	double Scale = 1.0;
	double SpeedTrendKtS = 0.0;
	double LastIas = 0.0;
	double LastSimTime = -1.0;
	uint32 LastCalloutSeq = 0;
	FString Callout;
	double CalloutUntil = 0.0;
	uint32 LastTouchdownSeq = 0;
	FString TouchdownText;
	double TouchdownUntil = 0.0;
	uint32 LastHintSeq = 0;
	FString HintText;
	double HintUntil = 0.0;
	// FMA columns (thrust, vertical, lateral, approach capability, AP): what is shown and since
	// when, to box a mode for 10 s after it engages.
	FString FmaShown[5];
	double FmaChangedAt[5] = {};
	TMap<int32, FBox2D> TargetBoxes;
};
