#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"

#include "A320Commands.h"
#include "a320/a320_api.h"

#include "A320Hud.generated.h"

class AA320Aircraft;
class FA320MapData;
class UFont;
class UTexture2D;

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
	// Whether any button or window was drawn there this frame (nothing to drag beneath it).
	bool IsOverButton(const FVector2D& ScreenPos) const;
	// The extra value of that button (e.g. which command a SET button assigns), 0 if none.
	int32 ParamAt(const FVector2D& ScreenPos) const;
	// The lever slot under a screen position, and the handle position (0 = top) for a drag.
	EA320Lever LeverAt(const FVector2D& ScreenPos) const;
	double LeverPosition(EA320Lever Lever, const FVector2D& ScreenPos) const;

	// The world map (MAP window): the player controller feeds it the wheel, drags, clicks and typing.
	static bool IsMapCommand(EA320Command Command);
	bool IsOverMap(const FVector2D& ScreenPos) const;
	void MapZoom(double Factor, const FVector2D& ScreenPos);
	void MapPan(const FVector2D& DeltaPixels);
	void MapClick(const FVector2D& ScreenPos);
	void MapCommand(EA320Command Command, int32 Param, AA320Aircraft& Aircraft);
	bool IsMapSearchActive() const { return bMapSearchActive; }
	void MapType(TCHAR Char);
	void MapBackspace();
	void MapEnter(AA320Aircraft& Aircraft);

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
	// The MCDU pop-up: its 14 x 24 screen, line select keys, page keys and keypad.
	void DrawMcdu(const AA320Aircraft& Aircraft);
	// The RADIO window: VHF 1 radio management panel, transponder, the radio log and ATC replies.
	void DrawRadio(const AA320Aircraft& Aircraft);
	// The latest ATC call as a subtitle while the RADIO window is closed.
	void DrawAtcSubtitle(const AA320Aircraft& Aircraft);
	void DrawJoystickPanel(const class AA320PlayerController& Controller);
	// First-start progress (shader/asset compilation) instead of a black screen; also logs
	// progress lines and "READY" for the launcher window (scripts/play.ps1).
	void DrawLoadingStatus(const AA320Aircraft* Aircraft);
	// The world map: a window over the cockpit in flight, or the setup screen's map in Area.
	void DrawMap(const AA320Aircraft& Aircraft, const FBox2D& Area, bool bSetup);
	void EnsureMapData(const AA320Aircraft& Aircraft);
	FVector2D MapToScreen(double NorthM, double EastM) const;
	void ScreenToMap(const FVector2D& Screen, double& NorthM, double& EastM) const;
	UTexture2D* MapTile(int32 Level, int32 Row, int32 Col, int32& LoadBudget);
	void SelectPlace(int32 Place, bool bCentre);
	void UpdateMapResults();
	// Airbus-style pushbutton: upper legend (e.g. FAULT/AVAIL) and lower legend (e.g. ON).
	void Pushbutton(double X, double Y, double W, double H, const FString& Name, const FString& Upper,
		const FLinearColor& UpperColor, const FString& Lower, const FLinearColor& LowerColor, EA320Command Command);
	// Toggle/rotary switch: shows the current position; clicking moves it to the next one.
	void Switch(double X, double Y, double W, double H, const FString& Name, const FString& Position, EA320Command Command);
	void LeverSlot(double X, double Y, double W, double H, EA320Lever Lever);
	void DrawFcu(const AA320Aircraft& Aircraft, double X, double Y, double W, double H);
	void DrawFma(const A320State& St, double X, double Y, double S);
	// Rain running over the windscreen in the cockpit view (the part of the screen above Top).
	void DrawWindscreenRain(const AA320Aircraft& Aircraft, double W, double Top);
	void AddButton(double X, double Y, double W, double H, const FString& Label, EA320Command Command, bool bLit, int32 Param = 0);
	void DrawOverlays(const AA320Aircraft& Aircraft);
	void DrawHelp();
	// Lessons: the picker, and the step panel with the current step's controls highlighted.
	void DrawGuideMenu();
	void DrawGuide(const AA320Aircraft& Aircraft);
	// The FLIGHT menu: departure and arrival runways, how the flight starts, and the lessons.
	// Before every flight: the map, the flight's options and the weather, full screen.
	void DrawSetup(const AA320Aircraft& Aircraft);
	void DrawFlightOptions(const AA320Aircraft& Aircraft, double PX, double PY, double PW, double PH);
	// "End this flight?" / "Quit?", over everything.
	void DrawConfirm(const AA320Aircraft& Aircraft);
	// A free space across the screen between this frame's windows over the band Top..Bottom: the
	// left-most gap that fits MaxW, else the widest, at most MaxW wide. When even that is narrower
	// than MinW, the overhead (YieldingBoxes) gives way; RADIO and MCDU never do.
	FBox2D FreeSlot(double Top, double Bottom, double MaxW, double Margin, double MinW) const;
	FBox2D WidestGap(double Top, double Bottom, double MaxW, double Margin, bool bWithYielding) const;
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
	// Text of a given line height in pixels (the large font, scaled), centred on X, Y.
	void TextSized(const FString& Str, double X, double Y, const FLinearColor& Color, double LineH);

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
	// The pop-up windows drawn this frame (RADIO, MCDU, overhead, setup, the lesson panel), which
	// the lesson panel and the ATC subtitle keep clear of.
	TArray<FBox2D> PanelBoxes;
	TArray<FBox2D> YieldingBoxes;

	// World map state: the view (centre and metres per pixel), the search and the selection.
	TSharedPtr<FA320MapData> MapData;
	UPROPERTY() TMap<FString, TObjectPtr<UTexture2D>> MapTextures;
	TArray<FString> MapTextureOrder;  // least recently used first
	FBox2D MapArea = FBox2D(ForceInit);
	TArray<FBox2D> MapBlockers;  // cards drawn over the map: not clicked through
	double MapCentreN = 0.0, MapCentreE = 0.0, MapMetresPerPixel = 600.0;
	bool bMapViewSet = false;
	FString MapSearch;
	bool bMapSearchActive = false;
	TArray<int32> MapResults;
	int32 MapSelected = -1;  // a place index; -2: a point clicked on the map
	double MapPointN = 0.0, MapPointE = 0.0;
};
