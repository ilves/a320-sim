#include "A320Hud.h"

#include "A320Aircraft.h"
#include "A320PlayerController.h"
#include "A320Sim.h"
#include "CanvasItem.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "RenderUtils.h"
#include "a320/Geometry2D.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#include "ShaderCompiler.h"
#endif

namespace
{
	// Airbus display colours.
	const FLinearColor White(0.95f, 0.95f, 0.95f);
	const FLinearColor Grey(0.55f, 0.55f, 0.58f);
	const FLinearColor Green(0.1f, 0.95f, 0.2f);
	const FLinearColor Amber(1.0f, 0.6f, 0.0f);
	const FLinearColor Red(1.0f, 0.1f, 0.1f);
	const FLinearColor Cyan(0.1f, 0.85f, 1.0f);
	const FLinearColor Magenta(1.0f, 0.2f, 1.0f);
	const FLinearColor Yellow(1.0f, 0.9f, 0.0f);
	const FLinearColor SkyBlue(0.08f, 0.42f, 0.85f);
	const FLinearColor Earth(0.45f, 0.26f, 0.08f);
	const FLinearColor Tape(0.2f, 0.2f, 0.23f);
	const FLinearColor Screen(0.01f, 0.01f, 0.015f);
	const FLinearColor Glareshield(0.05f, 0.055f, 0.06f);
	const FLinearColor ButtonFace(0.13f, 0.13f, 0.15f);

	double Wrap360(double Deg)
	{
		Deg = FMath::Fmod(Deg, 360.0);
		return Deg < 0.0 ? Deg + 360.0 : Deg;
	}

	double Wrap180(double Deg)
	{
		Deg = Wrap360(Deg);
		return Deg > 180.0 ? Deg - 360.0 : Deg;
	}

	// Screen point at a compass-style angle (clockwise from up).
	FVector2D Polar(double CX, double CY, double R, double Deg)
	{
		const double Rad = FMath::DegreesToRadians(Deg);
		return FVector2D(CX + R * FMath::Sin(Rad), CY - R * FMath::Cos(Rad));
	}

	FString HeadingLabel(double MagDeg)
	{
		const int32 Tens = FMath::RoundToInt(Wrap360(MagDeg) / 10.0) % 36;
		return FString::Printf(TEXT("%d"), Tens);
	}
}

UFont* AA320Hud::FontFor(int32 Size) const
{
	switch (Size)
	{
	case 0: return GEngine->GetSmallFont();
	case 2: return GEngine->GetLargeFont();
	default: return GEngine->GetMediumFont();
	}
}

void AA320Hud::Line(double X1, double Y1, double X2, double Y2, const FLinearColor& Color, double Thickness)
{
	Canvas->K2_DrawLine(FVector2D(X1, Y1), FVector2D(X2, Y2), (float)FMath::Max(1.0, Thickness * Scale), Color);
}

void AA320Hud::ClippedLine(double X1, double Y1, double X2, double Y2, double CX0, double CY0, double CX1, double CY1,
	const FLinearColor& Color, double Thickness)
{
	a320::geom::Vec2 A{X1, Y1}, B{X2, Y2};
	if (a320::geom::clipSegment({CX0, CY0, CX1, CY1}, A, B))
	{
		Line(A.x, A.y, B.x, B.y, Color, Thickness);
	}
}

void AA320Hud::Fill(double X, double Y, double W, double H, const FLinearColor& Color)
{
	DrawRect(Color, (float)X, (float)Y, (float)W, (float)H);
}

void AA320Hud::Frame(double X, double Y, double W, double H, const FLinearColor& Color, double Thickness)
{
	Line(X, Y, X + W, Y, Color, Thickness);
	Line(X + W, Y, X + W, Y + H, Color, Thickness);
	Line(X + W, Y + H, X, Y + H, Color, Thickness);
	Line(X, Y + H, X, Y, Color, Thickness);
}

void AA320Hud::Triangle(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FLinearColor& Color)
{
	FCanvasTriangleItem Tri(A, B, C, GWhiteTexture);
	Tri.SetColor(Color);
	Canvas->DrawItem(Tri);
}

void AA320Hud::Arc(double CX, double CY, double R, double FromDeg, double ToDeg, const FLinearColor& Color, double Thickness)
{
	const int32 Steps = FMath::Max(2, FMath::CeilToInt(FMath::Abs(ToDeg - FromDeg) / 4.0));
	FVector2D Prev = Polar(CX, CY, R, FromDeg);
	for (int32 i = 1; i <= Steps; ++i)
	{
		const FVector2D P = Polar(CX, CY, R, FromDeg + (ToDeg - FromDeg) * i / Steps);
		Line(Prev.X, Prev.Y, P.X, P.Y, Color, Thickness);
		Prev = P;
	}
}

void AA320Hud::Text(const FString& Str, double X, double Y, const FLinearColor& Color, int32 Size, int32 Align)
{
	UFont* Font = FontFor(Size);
	const float TextScale = (float)Scale;
	float W = 0.0f, H = 0.0f;
	GetTextSize(Str, W, H, Font, TextScale);
	const double Left = Align == 1 ? X - W / 2.0 : (Align == 2 ? X - W : X);
	DrawText(Str, Color, (float)Left, (float)(Y - H / 2.0), Font, TextScale);
}

int32 AA320Hud::ParamAt(const FVector2D& ScreenPos) const
{
	for (int32 i = Buttons.Num() - 1; i >= 0; --i)
	{
		if (Buttons[i].Box.IsInside(ScreenPos))
		{
			return Buttons[i].Param;
		}
	}
	return 0;
}

EA320Command AA320Hud::CommandAt(const FVector2D& ScreenPos) const
{
	// Last drawn is on top (pop-up panels are drawn after the cockpit).
	for (int32 i = Buttons.Num() - 1; i >= 0; --i)
	{
		if (Buttons[i].Box.IsInside(ScreenPos))
		{
			return Buttons[i].Command;
		}
	}
	return EA320Command::None;
}

void AA320Hud::DrawHUD()
{
	Super::DrawHUD();
	Buttons.Reset();
	Levers.Reset();
	TargetBoxes.Reset();
	const APlayerController* PC = GetOwningPlayerController();
	const AA320Aircraft* Aircraft = PC ? Cast<AA320Aircraft>(PC->GetPawn()) : nullptr;
	if (!Canvas)
	{
		return;
	}
	if (!Aircraft)
	{
		DrawLoadingStatus(nullptr);
		return;
	}

	const double W = Canvas->ClipX, H = Canvas->ClipY;
	// The panel fills the lower 42% of the screen (the cockpit camera is pitched down so the
	// runway stays visible above it); the FCU and EFIS controls sit on the glareshield.
	const double PanelTop = H * 0.58;
	const double FcuH = H * 0.055;
	const double DisplayTop = PanelTop + FcuH;
	const double PanelH = H - DisplayTop;
	const double S = FMath::Min(PanelH * 0.92, W * 0.22);
	const double Margin = (PanelH - S) / 2.0;
	Scale = S / 420.0;

	Fill(0.0, PanelTop, W, H - PanelTop, Glareshield);
	Line(0.0, PanelTop, W, PanelTop, FLinearColor(0.2f, 0.2f, 0.22f), 3.0);
	if (Aircraft->IsSimReady())
	{
		DrawFcu(*Aircraft, Margin, PanelTop + FcuH * 0.1, W - 2.0 * Margin, FcuH * 0.8);
	}
	const double Y = DisplayTop + Margin;
	double X = Margin;
	if (Aircraft->IsSimReady())
	{
		DrawPfd(*Aircraft, X, Y, S);
		MarkTarget(A320_GT_FMA, X, Y, S, 0.12 * S);
		MarkTarget(A320_GT_PFD_SPEED, X + 0.03 * S, Y + 0.14 * S, 0.13 * S, 0.64 * S);
		MarkTarget(A320_GT_PFD_ILS, X + 0.2 * S, Y + 0.14 * S, 0.56 * S, 0.71 * S);
		X += S + Margin;
		DrawNd(*Aircraft, X, Y, S);
		MarkTarget(A320_GT_ND, X, Y, S, S);
		X += S + Margin;
		DrawEwd(*Aircraft, X, Y, S);
		MarkTarget(A320_GT_EWD, X, Y, S, S);
		X += S + Margin;
	}
	if (Aircraft->IsSimReady())
	{
		const double Rest = W - X - Margin;
		DrawCenterPanel(*Aircraft, X, Y, Rest * 0.22, S);
		DrawPedestal(*Aircraft, X + Rest * 0.22 + Margin, Y, Rest * 0.78 - Margin, S);
		if (Aircraft->IsOverheadVisible())
		{
			DrawOverhead(*Aircraft);
		}
		if (Aircraft->IsMcduVisible())
		{
			DrawMcdu(*Aircraft);
		}
		if (Aircraft->IsRadioVisible())
		{
			DrawRadio(*Aircraft);
		}
	}
	DrawSimBar(*Aircraft);
	if (const AA320PlayerController* A320PC = Cast<AA320PlayerController>(PC))
	{
		if (A320PC->IsJoystickPanelVisible())
		{
			DrawJoystickPanel(*A320PC);
		}
	}
	DrawOverlays(*Aircraft);
	if (Aircraft->IsSimReady() && !Aircraft->IsRadioVisible())
	{
		DrawAtcSubtitle(*Aircraft);
	}
	if (Aircraft->IsSimReady())
	{
		DrawGuide(*Aircraft);
	}
	if (Aircraft->IsGuideMenuVisible())
	{
		DrawGuideMenu();  // modal: on top of everything, and its clicks win
	}
	DrawLoadingStatus(Aircraft);
}

void AA320Hud::DrawPfd(const AA320Aircraft& Aircraft, double X, double Y, double S)
{
	const A320State& St = Aircraft.GetSimState();
	Fill(X, Y, S, S, Screen);

	// Attitude.
	const double CX = X + 0.45 * S, CY = Y + 0.46 * S;
	const a320::geom::Rect Att{X + 0.20 * S, Y + 0.14 * S, X + 0.70 * S, Y + 0.78 * S};
	const double Ppd = 0.016 * S;  // pixels per degree of pitch
	const double BankRad = FMath::DegreesToRadians(St.bankDeg);
	const FVector2D Down(FMath::Sin(BankRad), FMath::Cos(BankRad));  // "below the horizon" on screen
	const FVector2D Along(FMath::Cos(BankRad), -FMath::Sin(BankRad));
	Fill(Att.x0, Att.y0, Att.x1 - Att.x0, Att.y1 - Att.y0, SkyBlue);
	const FVector2D Horizon = FVector2D(CX, CY) + Down * (St.pitchDeg * Ppd);
	const std::vector<a320::geom::Vec2> GroundPoly = a320::geom::groundPolygon(Att, {Horizon.X, Horizon.Y}, St.bankDeg);
	for (size_t i = 1; i + 1 < GroundPoly.size(); ++i)
	{
		Triangle(FVector2D(GroundPoly[0].x, GroundPoly[0].y), FVector2D(GroundPoly[i].x, GroundPoly[i].y),
			FVector2D(GroundPoly[i + 1].x, GroundPoly[i + 1].y), Earth);
	}
	ClippedLine(Horizon.X - Along.X * S, Horizon.Y - Along.Y * S, Horizon.X + Along.X * S, Horizon.Y + Along.Y * S,
		Att.x0, Att.y0, Att.x1, Att.y1, White, 2.0);
	for (int32 Tenths = -300; Tenths <= 300; Tenths += 25)
	{
		if (Tenths == 0)
		{
			continue;
		}
		const double P = Tenths / 10.0;
		const FVector2D Mid = FVector2D(CX, CY) + Down * ((St.pitchDeg - P) * Ppd);
		const double Half = (Tenths % 100 == 0 ? 0.10 : (Tenths % 50 == 0 ? 0.05 : 0.025)) * S;
		const double Inset = 0.03 * S;
		ClippedLine(Mid.X - Along.X * Half, Mid.Y - Along.Y * Half, Mid.X + Along.X * Half, Mid.Y + Along.Y * Half,
			Att.x0 + Inset, Att.y0 + Inset, Att.x1 - Inset, Att.y1 - Inset, White, 1.5);
		if (Tenths % 100 == 0)
		{
			const FVector2D L = Mid - Along * (Half + 0.03 * S);
			if (L.Y > Att.y0 + Inset && L.Y < Att.y1 - Inset && L.X > Att.x0)
			{
				Text(FString::Printf(TEXT("%d"), FMath::Abs(Tenths / 10)), L.X, L.Y, White, 0, 1);
			}
		}
	}
	// Bank scale moves with the horizon; the yellow index is fixed.
	const double BankR = 0.27 * S;
	for (const double Mark : {-45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0})
	{
		const double A = Mark - St.bankDeg;
		if (FMath::Abs(A) > 60.0)
		{
			continue;
		}
		const FVector2D P0 = Polar(CX, CY, BankR, A);
		const FVector2D P1 = Polar(CX, CY, BankR + (FMath::Abs(Mark) == 30.0 || Mark == 0.0 ? 0.035 : 0.02) * S, A);
		Line(P0.X, P0.Y, P1.X, P1.Y, White, 1.5);
	}
	Triangle(Polar(CX, CY, BankR - 0.002 * S, 0.0), Polar(CX, CY, BankR - 0.035 * S, -4.0), Polar(CX, CY, BankR - 0.035 * S, 4.0), Yellow);
	// Aircraft symbol.
	for (const double Side : {-1.0, 1.0})
	{
		Line(CX + Side * 0.17 * S, CY, CX + Side * 0.07 * S, CY, Yellow, 4.0);
		Line(CX + Side * 0.07 * S, CY, CX + Side * 0.07 * S, CY + 0.025 * S, Yellow, 4.0);
	}
	Fill(CX - 0.008 * S, CY - 0.008 * S, 0.016 * S, 0.016 * S, Yellow);

	// Speed tape: VLS (amber), stall (red) and VMAX (red barber pole).
	const double TapeTop = Y + 0.14 * S, TapeH = 0.64 * S, TapeBottom = TapeTop + TapeH;
	const double SX = X + 0.03 * S, SW = 0.13 * S;
	const double Ppk = TapeH / 80.0;
	auto SpeedY = [&](double Kt) { return CY - (Kt - St.iasKt) * Ppk; };
	Fill(SX, TapeTop, SW, TapeH, Tape);
	const double TapeIas = FMath::Max(St.iasKt, 30.0);
	for (int32 V = FMath::FloorToInt((TapeIas - 45.0) / 10.0) * 10; V <= TapeIas + 45.0; V += 10)
	{
		const double VY = SpeedY(V);
		if (V < 30 || VY < TapeTop || VY > TapeBottom)
		{
			continue;
		}
		Line(SX + SW - 0.025 * S, VY, SX + SW, VY, White, 1.5);
		if (V % 20 == 0)
		{
			Text(FString::Printf(TEXT("%d"), V), SX + SW - 0.035 * S, VY, White, 0, 2);
		}
	}
	if (!St.onGround)
	{
		const double VlsY = FMath::Clamp(SpeedY(St.vlsKt), TapeTop, TapeBottom);
		const double VsY = FMath::Clamp(SpeedY(St.vsKt), TapeTop, TapeBottom);
		Fill(SX + SW - 0.012 * S, VlsY, 0.012 * S, VsY - VlsY, Amber);
		Fill(SX + SW - 0.012 * S, VsY, 0.012 * S, TapeBottom - VsY, Red);
	}
	// Barber pole: alternating red segments from VMAX upwards.
	for (double BY = SpeedY(St.vmaxKt); BY > TapeTop; BY -= 0.03 * S)
	{
		const double Top = FMath::Max(BY - 0.015 * S, TapeTop);
		const double Bottom = FMath::Min(BY, TapeBottom);
		if (Bottom > Top)
		{
			Fill(SX + SW - 0.012 * S, Top, 0.012 * S, Bottom - Top, Red);
		}
	}
	// Takeoff: V1 as a cyan "1" on the tape (as on the A320), and the three speeds below it.
	if (St.onGround && St.v1Kt > 0.0)
	{
		const double V1Y = SpeedY(St.v1Kt);
		if (V1Y > TapeTop && V1Y < TapeBottom)
		{
			Text(TEXT("1"), SX + SW + 0.012 * S, V1Y, Cyan, 1, 1);
		}
		Text(FString::Printf(TEXT("V1 %d  VR %d  V2 %d"), FMath::RoundToInt(St.v1Kt), FMath::RoundToInt(St.vrKt), FMath::RoundToInt(St.v2Kt)),
			SX, TapeBottom + 0.035 * S, Cyan, 0, 0);
	}
	if (FMath::Abs(SpeedTrendKtS) * 10.0 > 2.0)
	{
		const double TipY = FMath::Clamp(SpeedY(St.iasKt + SpeedTrendKtS * 10.0), TapeTop, TapeBottom);
		Line(SX + SW + 0.01 * S, CY, SX + SW + 0.01 * S, TipY, Yellow, 2.0);
	}
	// Selected speed (FCU): cyan triangle, or its value at the tape end when off scale.
	const double BugY = SpeedY(St.fcuSpdKt);
	if (BugY > TapeTop && BugY < TapeBottom)
	{
		Triangle(FVector2D(SX + SW, BugY), FVector2D(SX + SW + 0.02 * S, BugY - 0.012 * S), FVector2D(SX + SW + 0.02 * S, BugY + 0.012 * S), Cyan);
	}
	else
	{
		Text(FString::Printf(TEXT("%d"), FMath::RoundToInt(St.fcuSpdKt)), SX + SW / 2.0, BugY < TapeTop ? TapeTop - 0.02 * S : TapeBottom + 0.02 * S, Cyan, 0, 1);
	}
	Line(SX, CY, SX + SW + 0.02 * S, CY, Yellow, 3.0);
	Fill(SX, CY - 0.03 * S, 0.085 * S, 0.06 * S, Screen);
	Text(FString::Printf(TEXT("%d"), FMath::RoundToInt(St.iasKt)), SX + 0.08 * S, CY, Green, 1, 2);

	// Altitude tape with the ground (field elevation) as a red band.
	const double AX = X + 0.75 * S, AW = 0.13 * S;
	const double Ppf = TapeH / 1000.0;
	auto AltY = [&](double Ft) { return CY - (Ft - St.altitudeFt) * Ppf; };
	Fill(AX, TapeTop, AW, TapeH, Tape);
	for (int32 A = FMath::FloorToInt((St.altitudeFt - 550.0) / 100.0) * 100; A <= St.altitudeFt + 550.0; A += 100)
	{
		const double TY = AltY(A);
		if (TY < TapeTop || TY > TapeBottom)
		{
			continue;
		}
		Line(AX, TY, AX + 0.02 * S, TY, White, 1.5);
		if (A % 500 == 0)
		{
			Text(FString::Printf(TEXT("%d"), A), AX + 0.025 * S, TY, White, 0, 0);
		}
	}
	const double GroundY = AltY(Aircraft.GetFieldElevationFt());
	if (GroundY < TapeBottom)
	{
		Fill(AX, FMath::Max(GroundY, TapeTop), 0.015 * S, TapeBottom - FMath::Max(GroundY, TapeTop), Red);
	}
	const double AltBugY = AltY(St.fcuAltFt);
	if (AltBugY > TapeTop && AltBugY < TapeBottom)
	{
		Frame(AX, AltBugY - 0.012 * S, 0.02 * S, 0.024 * S, Cyan, 2.0);
	}
	Text(FString::Printf(TEXT("%d"), FMath::RoundToInt(St.fcuAltFt)), AX + AW / 2.0, TapeTop - 0.025 * S, Cyan, 0, 1);
	Line(AX - 0.02 * S, CY, AX + AW, CY, Yellow, 3.0);
	Fill(AX, CY - 0.03 * S, AW, 0.06 * S, Screen);
	Text(FString::Printf(TEXT("%d"), FMath::RoundToInt(St.altitudeFt / 10.0) * 10), AX + AW - 0.005 * S, CY, Green, 1, 2);

	// Vertical speed: square-root scale to +-6000 fpm.
	const double VsX = X + 0.90 * S;
	Fill(VsX, CY - 0.25 * S, 0.06 * S, 0.5 * S, Tape);
	const double VsOffset = FMath::Sign(St.verticalSpeedFpm) * FMath::Sqrt(FMath::Min(FMath::Abs(St.verticalSpeedFpm), 6000.0) / 6000.0) * 0.24 * S;
	Line(VsX + 0.06 * S, CY, VsX + 0.005 * S, CY - VsOffset, FMath::Abs(St.verticalSpeedFpm) > 6000.0 ? Amber : Green, 2.5);
	if (FMath::Abs(St.verticalSpeedFpm) >= 200.0)
	{
		Text(FString::Printf(TEXT("%d"), FMath::RoundToInt(FMath::Abs(St.verticalSpeedFpm) / 100.0)), VsX + 0.03 * S,
			CY - VsOffset - FMath::Sign(St.verticalSpeedFpm) * 0.035 * S, Green, 0, 1);
	}

	// Heading tape (magnetic) with the track diamond.
	const double MagVar = Aircraft.GetMagneticVariation();
	const double Hdg = Wrap360(St.headingTrueDeg - MagVar);
	const double HX0 = X + 0.20 * S, HW = 0.50 * S, HY = Y + 0.87 * S, HH = 0.08 * S;
	const double Ppdh = HW / 40.0;
	Fill(HX0, HY, HW, HH, Tape);
	for (int32 D = FMath::FloorToInt((Hdg - 20.0) / 5.0) * 5; D <= Hdg + 20.0; D += 5)
	{
		const double DX = HX0 + HW / 2.0 + Wrap180(D - Hdg) * Ppdh;
		if (DX < HX0 || DX > HX0 + HW)
		{
			continue;
		}
		Line(DX, HY, DX, HY + (D % 10 == 0 ? 0.025 : 0.012) * S, White, 1.5);
		if (D % 10 == 0)
		{
			Text(HeadingLabel(D), DX, HY + 0.05 * S, White, 0, 1);
		}
	}
	const double HdgBugX = HX0 + HW / 2.0 + Wrap180(St.fcuHdgMagDeg - Hdg) * Ppdh;
	if (HdgBugX > HX0 && HdgBugX < HX0 + HW)
	{
		Triangle(FVector2D(HdgBugX, HY), FVector2D(HdgBugX - 0.01 * S, HY - 0.018 * S), FVector2D(HdgBugX + 0.01 * S, HY - 0.018 * S), Cyan);
	}
	Line(HX0 + HW / 2.0, HY - 0.02 * S, HX0 + HW / 2.0, HY + 0.02 * S, Yellow, 3.0);
	const double TrkX = HX0 + HW / 2.0 + Wrap180(St.trackTrueDeg - St.headingTrueDeg) * Ppdh;
	if (St.groundSpeedKt > 30.0 && TrkX > HX0 && TrkX < HX0 + HW)
	{
		Triangle(FVector2D(TrkX, HY + 0.005 * S), FVector2D(TrkX - 0.008 * S, HY + 0.018 * S), FVector2D(TrkX + 0.008 * S, HY + 0.018 * S), Green);
	}

	// ILS deviations (LS pushbutton), localizer below the attitude, glideslope at its right.
	if (Aircraft.IsLsOn())
	{
		const double Dot = 0.055 * S;
		const double LocY = Y + 0.815 * S;
		const double GsX = X + 0.725 * S;
		for (int32 k = -2; k <= 2; ++k)
		{
			if (k == 0)
			{
				Line(CX, LocY - 0.015 * S, CX, LocY + 0.015 * S, Yellow, 2.0);
				Line(GsX - 0.015 * S, CY, GsX + 0.015 * S, CY, Yellow, 2.0);
				continue;
			}
			Fill(CX + k * Dot - 0.004 * S, LocY - 0.004 * S, 0.008 * S, 0.008 * S, White);
			Fill(GsX - 0.004 * S, CY + k * Dot - 0.004 * S, 0.008 * S, 0.008 * S, White);
		}
		auto Diamond = [&](double DX, double DY)
		{
			const double R = 0.016 * S;
			Triangle(FVector2D(DX - R, DY), FVector2D(DX, DY - R), FVector2D(DX + R, DY), Magenta);
			Triangle(FVector2D(DX - R, DY), FVector2D(DX, DY + R), FVector2D(DX + R, DY), Magenta);
		};
		if (St.locValid)
		{
			Diamond(CX + FMath::Clamp(St.locDots, -2.3, 2.3) * Dot, LocY);
		}
		if (St.gsValid)
		{
			Diamond(GsX, CY - FMath::Clamp(St.gsDots, -2.3, 2.3) * Dot);
		}
		// ILS identification as on the PFD: ident (or frequency), course, DME.
		if (St.ilsFreqMHz > 0.0)
		{
			Text(FString::Printf(TEXT("%s %.2f"), UTF8_TO_TCHAR(St.ilsIdent), St.ilsFreqMHz), X + 0.02 * S, Y + 0.84 * S, Magenta, 0, 0);
			Text(FString::Printf(TEXT("%03d"), (FMath::RoundToInt(St.ilsCourseMagDeg) + 359) % 360 + 1), X + 0.02 * S, Y + 0.89 * S, Magenta, 0, 0);
			Text(FString::Printf(TEXT("%.1fNM"), St.dmeNm), X + 0.02 * S, Y + 0.94 * S, Magenta, 0, 0);
		}
		else
		{
			Text(TEXT("NO ILS TUNED"), X + 0.02 * S, Y + 0.89 * S, Amber, 0, 0);
		}
	}

	if (St.radioAltFt < 2500.0 && !St.onGround)
	{
		const int32 Ra = St.radioAltFt > 50.0 ? FMath::RoundToInt(St.radioAltFt / 10.0) * 10 : FMath::RoundToInt(St.radioAltFt);
		Text(FString::Printf(TEXT("%d"), Ra), CX, CY + 0.25 * S, St.radioAltFt < 400.0 ? Amber : Green, 2, 1);
	}

	DrawFma(St, X, Y, S);
	Frame(X, Y, S, S, Grey, 1.0);
}

void AA320Hud::DrawNd(const AA320Aircraft& Aircraft, double X, double Y, double S)
{
	const A320State& St = Aircraft.GetSimState();
	Fill(X, Y, S, S, Screen);
	const double MagVar = Aircraft.GetMagneticVariation();
	const double Hdg = St.headingTrueDeg;
	// ARC: forward 100 degrees with the aircraft low on the screen; ROSE: full compass rose.
	const bool bRose = Aircraft.IsNdRose();
	const bool bLs = Aircraft.GetNdMode() == A320_ND_ROSE_LS;
	const double AcX = X + 0.5 * S, AcY = Y + (bRose ? 0.53 : 0.82) * S;
	const double R = (bRose ? 0.38 : 0.68) * S;
	const double ArcHalf = bRose ? 180.0 : 50.0;
	const double RangeM = Aircraft.GetNdRangeNm() * 1852.0;
	const double Ppm = R / RangeM;
	const double HdgRad = FMath::DegreesToRadians(Hdg);
	auto ToScreen = [&](double North, double East)
	{
		const double Dn = North - St.northM, De = East - St.eastM;
		const double Right = -Dn * FMath::Sin(HdgRad) + De * FMath::Cos(HdgRad);
		const double Fwd = Dn * FMath::Cos(HdgRad) + De * FMath::Sin(HdgRad);
		return FVector2D(AcX + Right * Ppm, AcY - Fwd * Ppm);
	};
	const double CX0 = X + 2.0, CY0 = Y + 0.08 * S, CX1 = X + S - 2.0, CY1 = Y + S - 2.0;

	// Runways, and the active ILS's extended centreline out to 12 NM (no map in ROSE LS).
	const TArray<A320RunwayInfo>& Runways = Aircraft.GetRunways();
	for (int32 i = 0; i < Runways.Num() && !bLs; ++i)
	{
		const A320RunwayInfo& Rw = Runways[i];
		const double C = FMath::DegreesToRadians(Rw.trueCourseDeg);
		const double HalfW = FMath::Max(Rw.widthM / 2.0, 1.5 / Ppm);  // at least ~3 px wide
		const double RN = -FMath::Sin(C) * HalfW, RE = FMath::Cos(C) * HalfW;  // right offset
		const FVector2D A1 = ToScreen(Rw.startNorthM + RN, Rw.startEastM + RE);
		const FVector2D A2 = ToScreen(Rw.endNorthM + RN, Rw.endEastM + RE);
		const FVector2D B1 = ToScreen(Rw.startNorthM - RN, Rw.startEastM - RE);
		const FVector2D B2 = ToScreen(Rw.endNorthM - RN, Rw.endEastM - RE);
		ClippedLine(A1.X, A1.Y, A2.X, A2.Y, CX0, CY0, CX1, CY1, White, 2.0);
		ClippedLine(B1.X, B1.Y, B2.X, B2.Y, CX0, CY0, CX1, CY1, White, 2.0);
		ClippedLine(A1.X, A1.Y, B1.X, B1.Y, CX0, CY0, CX1, CY1, White, 2.0);
		if (i == St.ilsRunwayIndex)
		{
			const FVector2D T = ToScreen(Rw.thresholdNorthM, Rw.thresholdEastM);
			for (double D = 0.0; D < 12.0 * 1852.0; D += 1852.0)
			{
				const FVector2D P0 = ToScreen(Rw.thresholdNorthM - FMath::Cos(C) * D, Rw.thresholdEastM - FMath::Sin(C) * D);
				const FVector2D P1 = ToScreen(Rw.thresholdNorthM - FMath::Cos(C) * (D + 926.0), Rw.thresholdEastM - FMath::Sin(C) * (D + 926.0));
				ClippedLine(P0.X, P0.Y, P1.X, P1.Y, CX0, CY0, CX1, CY1, Magenta, 1.5);
			}
			if (T.X > CX0 && T.X < CX1 && T.Y > CY0 && T.Y < CY1)
			{
				Text(UTF8_TO_TCHAR(Rw.ident), T.X + 0.02 * S, T.Y, White, 0, 0);
			}
		}
	}

	// Compass arc (magnetic), range ring and heading/track marks.
	const double MagHdg = Wrap360(Hdg - MagVar);
	Arc(AcX, AcY, R, -ArcHalf, ArcHalf, White, 1.5);
	Arc(AcX, AcY, R / 2.0, -ArcHalf, ArcHalf, Grey, 1.0);
	Text(FString::Printf(TEXT("%d"), Aircraft.GetNdRangeNm() / 2), AcX - R / 2.0 * 0.7 - 0.03 * S, AcY - R / 2.0 * 0.7, Cyan, 0, 1);
	for (int32 D = static_cast<int32>(FMath::CeilToInt((MagHdg - ArcHalf) / 5.0)) * 5; D <= MagHdg + ArcHalf - (bRose ? 1.0 : 0.0); D += 5)
	{
		const double A = D - MagHdg;
		const FVector2D P0 = Polar(AcX, AcY, R, A);
		const FVector2D P1 = Polar(AcX, AcY, R + (D % 10 == 0 ? 0.03 : 0.015) * S, A);
		Line(P0.X, P0.Y, P1.X, P1.Y, White, 1.5);
		if (D % 30 == 0)
		{
			const FVector2D L = Polar(AcX, AcY, R + 0.06 * S, A);
			Text(HeadingLabel(D), L.X, L.Y, White, 0, 1);
		}
	}
	Line(AcX, AcY - R - 0.045 * S, AcX, AcY - R + 0.01 * S, Yellow, 3.0);
	const double BugA = Wrap180(St.fcuHdgMagDeg - MagHdg);
	if (FMath::Abs(BugA) < ArcHalf)
	{
		const FVector2D B0 = Polar(AcX, AcY, R + 0.005 * S, BugA - 2.5);
		const FVector2D B1 = Polar(AcX, AcY, R + 0.005 * S, BugA + 2.5);
		const FVector2D B2 = Polar(AcX, AcY, R + 0.035 * S, BugA);
		Triangle(B0, B1, B2, Cyan);
	}
	if (St.groundSpeedKt > 30.0)
	{
		const FVector2D T = Polar(AcX, AcY, R - 0.01 * S, Wrap180(St.trackTrueDeg - Hdg));
		Triangle(FVector2D(T.X, T.Y - 0.012 * S), FVector2D(T.X - 0.01 * S, T.Y + 0.01 * S), FVector2D(T.X + 0.01 * S, T.Y + 0.01 * S), Green);
	}
	if (bLs)
	{
		// ROSE LS: the ILS course pointer, the deviation bar (where the localizer is, 2 dots each
		// side) and the glideslope scale on the right, as on the PFD.
		const double CourseA = Wrap180(St.ilsCourseMagDeg - MagHdg);
		const double ARad = FMath::DegreesToRadians(CourseA);
		const FVector2D Dir(FMath::Sin(ARad), -FMath::Cos(ARad));
		const FVector2D Right(FMath::Cos(ARad), FMath::Sin(ARad));
		const FVector2D C(AcX, AcY);
		const FVector2D Head = C + Dir * (0.95 * R), HeadBase = C + Dir * (0.55 * R);
		const FVector2D Tail0 = C - Dir * (0.55 * R), Tail1 = C - Dir * (0.95 * R);
		Line(HeadBase.X, HeadBase.Y, Head.X, Head.Y, Magenta, 3.0);
		Triangle(Head + Dir * (0.06 * R), Head - Right * (0.04 * R), Head + Right * (0.04 * R), Magenta);
		Line(Tail0.X, Tail0.Y, Tail1.X, Tail1.Y, Magenta, 3.0);
		const double DotPx = 0.17 * R;
		for (int32 k = -2; k <= 2; ++k)
		{
			if (k != 0)
			{
				const FVector2D P = C + Right * (k * DotPx);
				Fill(P.X - 0.006 * S, P.Y - 0.006 * S, 0.012 * S, 0.012 * S, White);
			}
		}
		if (St.locValid)
		{
			const FVector2D Off = Right * (FMath::Clamp(St.locDots, -2.4, 2.4) * DotPx);
			const FVector2D B0 = C + Off - Dir * (0.45 * R), B1 = C + Off + Dir * (0.45 * R);
			Line(B0.X, B0.Y, B1.X, B1.Y, Magenta, 3.0);
		}
		const double GsX = X + 0.93 * S, Dot = 0.055 * S;
		for (int32 k = -2; k <= 2; ++k)
		{
			if (k == 0)
			{
				Line(GsX - 0.015 * S, AcY, GsX + 0.015 * S, AcY, Yellow, 2.0);
			}
			else
			{
				Fill(GsX - 0.004 * S, AcY + k * Dot - 0.004 * S, 0.008 * S, 0.008 * S, White);
			}
		}
		if (St.gsValid)
		{
			const double DY = AcY - FMath::Clamp(St.gsDots, -2.3, 2.3) * Dot, DR = 0.016 * S;
			Triangle(FVector2D(GsX - DR, DY), FVector2D(GsX, DY - DR), FVector2D(GsX + DR, DY), Magenta);
			Triangle(FVector2D(GsX - DR, DY), FVector2D(GsX, DY + DR), FVector2D(GsX + DR, DY), Magenta);
		}
		if (Runways.IsValidIndex(St.ilsRunwayIndex))
		{
			Text(FString::Printf(TEXT("CRS %03d"), (FMath::RoundToInt(St.ilsCourseMagDeg) + 359) % 360 + 1), X + S - 0.03 * S, Y + 0.09 * S, Magenta, 0, 2);
		}
		if (!St.locValid)
		{
			Text(TEXT("NO LOC SIGNAL"), AcX, AcY + 0.3 * R, Amber, 0, 1);
		}
	}

	// Own aircraft.
	Line(AcX, AcY - 0.04 * S, AcX, AcY + 0.04 * S, Yellow, 3.0);
	Line(AcX - 0.04 * S, AcY - 0.01 * S, AcX + 0.04 * S, AcY - 0.01 * S, Yellow, 3.0);
	Line(AcX - 0.015 * S, AcY + 0.035 * S, AcX + 0.015 * S, AcY + 0.035 * S, Yellow, 3.0);

	Text(FString::Printf(TEXT("GS %d  TAS %d"), FMath::RoundToInt(St.groundSpeedKt), FMath::RoundToInt(St.tasKt)), X + 0.03 * S, Y + 0.04 * S, White, 0, 0);
	if (Aircraft.IsLsOn() && Runways.IsValidIndex(St.ilsRunwayIndex))
	{
		Text(FString::Printf(TEXT("%s %.2f  %.1f NM"), UTF8_TO_TCHAR(St.ilsIdent), St.ilsFreqMHz, St.dmeNm), X + S - 0.03 * S, Y + 0.04 * S, Magenta, 0, 2);
	}
	Text(FString::Printf(TEXT("%s  %d NM"), bLs ? TEXT("ROSE LS") : (bRose ? TEXT("ROSE NAV") : TEXT("ARC")), Aircraft.GetNdRangeNm()),
		X + 0.03 * S, Y + 0.96 * S, Cyan, 0, 0);
	Frame(X, Y, S, S, Grey, 1.0);
}

void AA320Hud::DrawEwd(const AA320Aircraft& Aircraft, double X, double Y, double S)
{
	const A320State& St = Aircraft.GetSimState();
	Fill(X, Y, S, S, Screen);

	// N1 dials: 0..110 % over 220 degrees, thrust lever position as a cyan mark.
	for (int32 e = 0; e < 2; ++e)
	{
		const double DX = X + (e == 0 ? 0.26 : 0.70) * S, DY = Y + 0.20 * S, DR = 0.12 * S;
		auto Angle = [](double N1) { return -130.0 + FMath::Clamp(N1, 0.0, 110.0) / 110.0 * 220.0; };
		Arc(DX, DY, DR, -130.0, Angle(100.0), White, 2.0);
		Arc(DX, DY, DR, Angle(100.0), 90.0, Red, 2.0);
		const FVector2D Needle = Polar(DX, DY, DR * 0.95, Angle(St.n1[e]));
		Line(DX, DY, Needle.X, Needle.Y, Green, 3.0);
		const FVector2D Lever = Polar(DX, DY, DR + 0.012 * S, Angle((St.reverseEng[e] ? 0.0 : St.thrustLeverEng[e]) * 101.0));
		Fill(Lever.X - 0.008 * S, Lever.Y - 0.008 * S, 0.016 * S, 0.016 * S, Cyan);
		Text(FString::Printf(TEXT("%.1f"), St.n1[e]), DX + DR * 0.9, DY + DR * 0.55, Green, 1, 2);
		Text(FString::Printf(TEXT("N2 %.1f"), St.n2[e]), DX, Y + 0.38 * S, Green, 0, 1);
		Text(FString::Printf(TEXT("FF %d"), FMath::RoundToInt(St.fuelFlowKgH[e] / 10.0) * 10), DX, Y + 0.43 * S, Green, 0, 1);
	}
	Text(TEXT("N1 %"), X + 0.48 * S, Y + 0.20 * S, Cyan, 0, 1);
	Text(UTF8_TO_TCHAR(a320_thrust_detent_name(St.thrustDetent)), X + S - 0.03 * S, Y + 0.04 * S, Cyan, 1, 2);
	if (St.reverse)
	{
		Text(TEXT("REV"), X + 0.48 * S, Y + 0.30 * S, Green, 1, 1);
	}
	else if (St.athrMode == A320_ATHR_AFLOOR)
	{
		Text(TEXT("A FLOOR"), X + 0.48 * S, Y + 0.30 * S, Green, 1, 1);
	}
	Text(FString::Printf(TEXT("FOB %d KG"), FMath::RoundToInt(St.fuelKg / 10.0) * 10), X + 0.04 * S, Y + 0.50 * S, Green, 0, 0);
	Text(FString::Printf(TEXT("GW %d KG"), FMath::RoundToInt(St.grossWeightKg / 100.0) * 100), X + S - 0.04 * S, Y + 0.50 * S, Green, 0, 2);

	// Flaps: lever position (cyan) and actual flaps (white bar, 35 degrees = FULL).
	const double FY = Y + 0.58 * S;
	Text(TEXT("FLAPS"), X + 0.04 * S, FY, White, 0, 0);
	Text(UTF8_TO_TCHAR(a320_flap_config_name(St.flapsLever, St.onePlusF)), X + 0.30 * S, FY, Green, 1, 0);
	const double BarX = X + 0.48 * S, BarW = 0.46 * S;
	Fill(BarX, FY - 0.006 * S, BarW * FMath::Clamp(St.flapDeg / 35.0, 0.0, 1.0), 0.012 * S, White);
	for (int32 k = 0; k <= 4; ++k)
	{
		const double KX = BarX + BarW * k / 4.0;
		Fill(KX - 0.004 * S, FY + 0.012 * S, 0.008 * S, 0.008 * S, k == St.flapsLever ? Cyan : Grey);
	}

	// Landing gear: three green "DN" when locked down, red while in transit.
	const double GY = Y + 0.67 * S;
	Text(TEXT("GEAR"), X + 0.04 * S, GY, White, 0, 0);
	const bool bDown = St.gearPos > 0.99, bUp = St.gearPos < 0.01;
	for (int32 k = 0; k < 3; ++k)
	{
		const double BX = X + (0.30 + k * 0.14) * S;
		const FLinearColor C = bDown ? Green : (bUp ? Grey : Red);
		Frame(BX, GY - 0.022 * S, 0.11 * S, 0.044 * S, C, 1.5);
		Text(bDown ? TEXT("DN") : (bUp ? TEXT("UP") : TEXT("--")), BX + 0.055 * S, GY, C, 0, 1);
	}
	Text(FString::Printf(TEXT("THS %.1f %s"), FMath::Abs(St.thsDeg), St.thsDeg <= 0.0 ? TEXT("UP") : TEXT("DN")), X + S - 0.04 * S, GY, Cyan, 0, 2);

	// Warnings (red), cautions (amber) and memos (green).
	double LineY = Y + 0.76 * S;
	for (int32 Bit = 0; Bit < A320_WARN_COUNT && LineY < Y + S; ++Bit)
	{
		const uint32 Mask = 1u << Bit;
		if (St.warnings & Mask)
		{
			Text(UTF8_TO_TCHAR(a320_warning_text(Mask)), X + 0.04 * S, LineY, (Mask & A320_WARN_CAUTION_MASK) ? Amber : Red, 0, 0);
			LineY += 0.05 * S;
		}
	}
	// LDG MEMO on the way down with the gear down or below 2000 ft (not after take-off):
	// blue = still to do, green = done.
	if (!St.onGround && St.verticalSpeedFpm < 0.0 && (St.radioAltFt < 2000.0 || St.gearLeverDown))
	{
		struct FMemoItem
		{
			const TCHAR* Todo;
			const TCHAR* Done;
			bool bDone;
		};
		const int32 Signs = A320_SIGN_SEATBELTS | A320_SIGN_NO_SMOKING;
		const FMemoItem Items[] = {
			{TEXT("LDG  GEAR ..... DN"), TEXT("LDG  GEAR DN"), St.gearLeverDown != 0},
			{TEXT("     SIGNS .... ON"), TEXT("     SIGNS ON"), (St.signs & Signs) == Signs},
			{TEXT("     SPLRS ... ARM"), TEXT("     SPLRS ARM"), St.spoilersArmed != 0},
			{TEXT("     FLAPS .. FULL"), TEXT("     FLAPS FULL"), St.flapsLever == 4},
		};
		for (const FMemoItem& Item : Items)
		{
			if (LineY < Y + S - 0.02 * S)
			{
				Text(Item.bDone ? Item.Done : Item.Todo, X + 0.04 * S, LineY, Item.bDone ? Green : Cyan, 0, 0);
				LineY += 0.045 * S;
			}
		}
	}
	// Memos (green; blue = selected/armed), bottom right.
	TArray<TPair<FString, FLinearColor>> Memos;
	if (St.apuAvail) Memos.Emplace(TEXT("APU AVAIL"), Green);
	if (St.apuBleed && St.apuAvail) Memos.Emplace(TEXT("APU BLEED"), Green);
	if (St.signs & A320_SIGN_SEATBELTS) Memos.Emplace(TEXT("SEAT BELTS"), Green);
	if (St.signs & A320_SIGN_NO_SMOKING) Memos.Emplace(TEXT("NO SMOKING"), Green);
	if (St.parkBrake) Memos.Emplace(TEXT("PARK BRK"), Green);
	if (St.autobrake) Memos.Emplace(FString::Printf(TEXT("AUTO BRK %s"), St.autobrake == A320_AUTOBRAKE_LO ? TEXT("LO") : (St.autobrake == A320_AUTOBRAKE_MED ? TEXT("MED") : TEXT("MAX"))), Cyan);
	if (St.groundSpoilers) Memos.Emplace(TEXT("GND SPLRS"), Green);
	else if (St.spoilersArmed) Memos.Emplace(TEXT("GND SPLRS ARMED"), Cyan);
	else if (St.speedbrakePos > 0.05) Memos.Emplace(TEXT("SPEED BRK"), Green);
	if (St.lights & A320_LT_LANDING) Memos.Emplace(TEXT("LDG LT"), Green);
	for (int32 i = 0; i < 2; ++i)
	{
		if (St.engStarting[i]) Memos.Emplace(FString::Printf(TEXT("ENG %d START"), i + 1), Amber);
	}
	double MemoY = Y + 0.76 * S;
	for (const TPair<FString, FLinearColor>& M : Memos)
	{
		if (MemoY > Y + S - 0.02 * S)
		{
			break;
		}
		Text(M.Key, X + S - 0.04 * S, MemoY, M.Value, 0, 2);
		MemoY += 0.045 * S;
	}
	if (St.apuMaster || St.apuN > 1.0)
	{
		Text(FString::Printf(TEXT("APU N %d%%"), FMath::RoundToInt(St.apuN)), X + 0.48 * S, Y + 0.43 * S, St.apuAvail ? Green : Amber, 0, 1);
	}
	Frame(X, Y, S, S, Grey, 1.0);
}

void AA320Hud::DrawOverlays(const AA320Aircraft& Aircraft)
{
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	const double Now = GetWorld()->GetRealTimeSeconds();
	const double OverlayScale = FMath::Max(H / 1080.0, 0.6);
	Scale = OverlayScale;

	if (!Aircraft.IsSimReady())
	{
		Text(Aircraft.GetSimError(), W / 2.0, H * 0.3, Red, 2, 1);
		return;
	}
	const A320State& St = Aircraft.GetSimState();

	// Speed trend for the PFD, from sim time so pause and sim rate do not distort it.
	if (LastSimTime >= 0.0 && St.simTimeS > LastSimTime)
	{
		const double Rate = (St.iasKt - LastIas) / (St.simTimeS - LastSimTime);
		SpeedTrendKtS += (Rate - SpeedTrendKtS) * 0.1;
	}
	else if (St.simTimeS < LastSimTime)
	{
		SpeedTrendKtS = 0.0;
	}
	LastSimTime = St.simTimeS;
	LastIas = St.iasKt;

	if (St.calloutSeq != LastCalloutSeq)
	{
		LastCalloutSeq = St.calloutSeq;
		Callout = UTF8_TO_TCHAR(St.callout);
		CalloutUntil = Now + 1.5;
	}
	if (Now < CalloutUntil)
	{
		Text(Callout, W / 2.0, H * 0.16, White, 2, 1);
	}

	if (St.touchdownSeq != LastTouchdownSeq)
	{
		LastTouchdownSeq = St.touchdownSeq;
		const double Fpm = -St.touchdownFpm;
		const TCHAR* Rating = Fpm < 240.0 ? TEXT("smooth") : (Fpm < 600.0 ? TEXT("firm") : TEXT("HARD"));
		TouchdownText = FString::Printf(TEXT("Touchdown %d fpm (%s), %d m past threshold, %.1f m %s of centreline"),
			FMath::RoundToInt(Fpm), Rating, FMath::RoundToInt(St.touchdownDistanceM), FMath::Abs(St.touchdownCenterlineM),
			St.touchdownCenterlineM >= 0.0 ? TEXT("right") : TEXT("left"));
		TouchdownUntil = Now + 10.0;
	}
	if (Now < TouchdownUntil)
	{
		Text(TouchdownText, W / 2.0, H * 0.08, Cyan, 1, 1);
	}

	// Sim tutor: why a press did nothing, or what to watch for.
	if (St.hintSeq != LastHintSeq)
	{
		LastHintSeq = St.hintSeq;
		HintText = UTF8_TO_TCHAR(St.hint);
		HintUntil = Now + 9.0;
	}
	if (Now < HintUntil && !HintText.IsEmpty())
	{
		const double TW = FMath::Min(W * 0.4, 760.0 * Scale), Pad = 10.0 * Scale;
		const double TX = (W - TW) / 2.0, TY = H * 0.2;
		const double TextH = TextWrapped(HintText, TX + Pad, TY + 28.0 * Scale, TW - 2.0 * Pad, White, 0, false);
		Fill(TX, TY, TW, TextH + 36.0 * Scale, FLinearColor(0.0f, 0.0f, 0.0f, 0.75f));
		Frame(TX, TY, TW, TextH + 36.0 * Scale, Amber, 2.0);
		Text(TEXT("SIM TUTOR"), TX + Pad, TY + 14.0 * Scale, Amber, 0, 0);
		TextWrapped(HintText, TX + Pad, TY + 28.0 * Scale, TW - 2.0 * Pad, White, 0);
	}

	const uint32 WarningBits = St.warnings & ~uint32(A320_WARN_CAUTION_MASK);
	const uint32 CautionBits = St.warnings & uint32(A320_WARN_CAUTION_MASK);
	const bool bFlash = FMath::Fmod(Now, 0.8) < 0.5;
	if ((WarningBits || CautionBits) && bFlash)
	{
		const FLinearColor C = WarningBits ? Red : Amber;
		const double BW = 190.0 * Scale, BH = 46.0 * Scale;
		Fill(20.0 * Scale, 20.0 * Scale, BW, BH, FLinearColor(0.0f, 0.0f, 0.0f, 0.6f));
		Frame(20.0 * Scale, 20.0 * Scale, BW, BH, C, 2.0);
		Text(WarningBits ? TEXT("MASTER WARN") : TEXT("MASTER CAUT"), 20.0 * Scale + BW / 2.0, 20.0 * Scale + BH / 2.0, C, 1, 1);
	}
	if (WarningBits || CautionBits)
	{
		// Clicking the master warning light silences the repetitive chime, as on the aircraft.
		Buttons.Add({FBox2D(FVector2D(20.0 * Scale, 20.0 * Scale), FVector2D(210.0 * Scale, 66.0 * Scale)), EA320Command::MasterWarnAck});
	}
	if (St.paused)
	{
		Text(TEXT("PAUSED  -  press P to continue"), W / 2.0, H * 0.32, Yellow, 2, 1);
	}
	if (St.simRate > 1.0)
	{
		Text(FString::Printf(TEXT("SIM RATE x%d"), static_cast<int32>(FMath::RoundToInt(St.simRate))), W - 20.0 * Scale, 56.0 * Scale, Yellow, 1, 2);
	}
	if (Aircraft.IsHelpVisible() && !Aircraft.IsOverheadVisible())
	{
		DrawHelp();
	}
}

void AA320Hud::DrawHelp()
{
	static const TCHAR* Lines[] = {
		TEXT("A320 SIM  -  EETN Tallinn            H / F1: hide this help"),
		TEXT("Sidestick      Arrow keys or numpad (Shift = full deflection), gamepad left stick"),
		TEXT("Rudder/tiller  Q / E (or Z / X), gamepad right stick"),
		TEXT("Thrust         PgUp / PgDn,  Home TOGA,  Del FLX/MCT,  Ins CL,  End IDLE"),
		TEXT("Reverse        R (on ground, then PgUp for more reverse)"),
		TEXT("Brakes         B (hold),  N parking brake"),
		TEXT("Flaps          F retract / V extend      Gear  G      Speedbrake  /"),
		TEXT("ILS on PFD     L (LS button)             ND range  , and ."),
		TEXT("Pause          P                         Sim rate  ="),
		TEXT("View           C cockpit / outside,  right mouse drag to look, middle click to reset"),
		TEXT("Scenarios      F5 lined up 26,  Shift+F5 cold and dark,  F4 20 NM intercept,  F6 10 NM final,  F7 4 NM final,  F9 swap rwy"),
		TEXT("Cockpit        drag the thrust, flaps and speedbrake levers;  click switches;  O overhead panel"),
		TEXT("Joystick       F2 (or JOYSTICK, top right): pick axes with LEARN; trigger = AP disconnect, hat = look"),
		TEXT("Autopilot      A AP1,  Shift+A AP2,  T A/THR (thrust levers in CL: Ins),  K APPR (autoland),  J LOC"),
		TEXT("Lessons        F3 (or LESSONS, top right): step-by-step guides, e.g. ILS approach and autoland"),
		TEXT("MCDU           Tab (or MCDU, top right): arrival ILS, RAD NAV, PERF; type on the keyboard, Backspace = CLR"),
		TEXT("Radio / ATC    F10 (or RADIO, top right): COM 1, transponder, ATC log; keys 1-6 pick a reply while it's open"),
		TEXT("FCU            1/2 SPD,  3/4 HDG,  5/6 ALT,  7/8 V/S  (Shift = x10);  U fly HDG,  9 climb/descend to ALT,  0 hold V/S"),
		TEXT("Sound          - (minus) on/off,  M silence master warning"),
		TEXT("Takeoff        N (release brake), Home (TOGA), rotate ~150 kt with Down arrow, G at positive climb"),
		TEXT("Landing        Vapp = VLS + 5 (amber strip), keep diamonds centred, flare ~30 ft, End at RETARD"),
		TEXT("Quit           Esc (standalone game)"),
	};
	const double LineH = 24.0 * Scale;
	const int32 NumLines = UE_ARRAY_COUNT(Lines);
	const double BoxW = 900.0 * Scale, BoxH = LineH * (NumLines + 1);
	const double X = 20.0 * Scale, Y = 80.0 * Scale;
	Fill(X, Y, BoxW, BoxH, FLinearColor(0.0f, 0.0f, 0.0f, 0.65f));
	for (int32 i = 0; i < NumLines; ++i)
	{
		Text(Lines[i], X + 14.0 * Scale, Y + LineH * (i + 1), i == 0 ? Cyan : White, 0, 0);
	}
}

void AA320Hud::AddButton(double X, double Y, double W, double H, const FString& Label, EA320Command Command, bool bLit, int32 Param)
{
	Fill(X, Y, W, H, ButtonFace);
	Frame(X, Y, W, H, bLit ? Green : FLinearColor(0.3f, 0.3f, 0.33f), bLit ? 2.0 : 1.0);
	if (bLit)
	{
		Fill(X + W * 0.25, Y + H - 4.0 * Scale, W * 0.5, 2.5 * Scale, Green);
	}
	Text(Label, X + W / 2.0, Y + H / 2.0, bLit ? Green : White, 0, 1);
	Buttons.Add({FBox2D(FVector2D(X, Y), FVector2D(X + W, Y + H)), Command, Param});
}

void AA320Hud::DrawFcu(const AA320Aircraft& Aircraft, double X, double Y, double W, double H)
{
	const A320State& St = Aircraft.GetSimState();
	// Airbus FCU order: SPD | HDG | LOC | AP1 A/THR | ALT | APPR | V/S. Each window has -/+ and
	// a "pull" button (fly the selected value).
	struct FWindow
	{
		const TCHAR* Label;
		FString Value;
		EA320Command Dec, Inc, Pull;
		const TCHAR* PullLabel;
		bool bPullLit;
	};
	const bool bVs = St.vertMode == A320_VERT_VS;
	const FWindow FcuWindows[] = {
		{TEXT("SPD"), FString::Printf(TEXT("%03d"), FMath::RoundToInt(St.fcuSpdKt)), EA320Command::SpdDec, EA320Command::SpdInc, EA320Command::None, TEXT(""), false},
		{TEXT("HDG"), FString::Printf(TEXT("%03d"), (FMath::RoundToInt(St.fcuHdgMagDeg) + 359) % 360 + 1), EA320Command::HdgDec, EA320Command::HdgInc, EA320Command::FcuHdgPull, TEXT("HDG"), St.latMode == A320_LAT_HDG},
		{TEXT("ALT"), FString::Printf(TEXT("%05d"), FMath::RoundToInt(St.fcuAltFt)), EA320Command::AltDec, EA320Command::AltInc, EA320Command::FcuAltPull, TEXT("LVL/CH"), St.vertMode == A320_VERT_OP_CLB || St.vertMode == A320_VERT_OP_DES},
		{TEXT("V/S"), bVs ? FString::Printf(TEXT("%+05d"), FMath::RoundToInt(St.fcuVsFpm)) : FString(TEXT("-----")), EA320Command::VsDec, EA320Command::VsInc, EA320Command::FcuVsPull, TEXT("V/S"), bVs},
	};
	Fill(X, Y, W, H, FLinearColor(0.09f, 0.095f, 0.1f));
	const double Gap = 6.0 * Scale;
	const double WindowW = W * 0.118, ButtonW = W * 0.047;
	double CX = X + Gap;
	// Captain's EFIS control panel: LS, ND mode and range.
	const double EfisW = W * 0.045;
	AddButton(CX, Y + H * 0.15, EfisW, H * 0.75, TEXT("LS"), EA320Command::LsToggle, Aircraft.IsLsOn());
	CX += EfisW + Gap;
	const int32 NdMode = Aircraft.GetNdMode();
	AddButton(CX, Y + H * 0.15, EfisW * 1.5, H * 0.75,
		NdMode == A320_ND_ROSE_LS ? TEXT("ND LS") : (NdMode == A320_ND_ROSE_NAV ? TEXT("ND NAV") : TEXT("ND ARC")), EA320Command::NdModeToggle, false);
	CX += EfisW * 1.5 + Gap;
	AddButton(CX, Y + H * 0.15, EfisW * 0.8, H * 0.75, TEXT("RNG-"), EA320Command::NdRangeDown, false);
	CX += EfisW * 0.8 + 2.0;
	AddButton(CX, Y + H * 0.15, EfisW * 0.8, H * 0.75, TEXT("RNG+"), EA320Command::NdRangeUp, false);
	CX += EfisW * 0.8 + Gap * 3.0;
	auto DrawWindow = [&](const FWindow& Win)
	{
		const double BoxW = WindowW * 0.5;
		Text(Win.Label, CX + 2.0 * Scale, Y + H * 0.22, White, 0, 0);
		Fill(CX, Y + H * 0.42, BoxW, H * 0.5, Screen);
		Text(Win.Value, CX + BoxW / 2.0, Y + H * 0.67, Amber, 1, 1);
		const double SmallW = WindowW * 0.15;
		AddButton(CX + BoxW + 2.0, Y + H * 0.42, SmallW, H * 0.5, TEXT("-"), Win.Dec, false);
		AddButton(CX + BoxW + SmallW + 4.0, Y + H * 0.42, SmallW, H * 0.5, TEXT("+"), Win.Inc, false);
		if (Win.Pull != EA320Command::None)
		{
			AddButton(CX + BoxW + 2.0 * SmallW + 6.0, Y + H * 0.42, WindowW - BoxW - 2.0 * SmallW - 8.0, H * 0.5, Win.PullLabel, Win.Pull, Win.bPullLit);
		}
		const int32 Target = Win.Dec == EA320Command::SpdDec ? A320_GT_FCU_SPD
			: (Win.Dec == EA320Command::HdgDec ? A320_GT_FCU_HDG : (Win.Dec == EA320Command::AltDec ? A320_GT_FCU_ALT : A320_GT_NONE));
		if (Target != A320_GT_NONE)
		{
			MarkTarget(Target, CX, Y, WindowW, H);
		}
		CX += WindowW + Gap;
	};
	auto DrawButton = [&](const TCHAR* Label, EA320Command Command, bool bLit)
	{
		AddButton(CX, Y + H * 0.15, ButtonW, H * 0.75, Label, Command, bLit);
		CX += ButtonW + Gap;
	};
	const bool bLocLit = St.latMode == A320_LAT_LOC || St.latMode == A320_LAT_LOC_STAR || (St.armed & A320_ARMED_LOC);
	const bool bApprLit = (St.armed & A320_ARMED_GS) || St.vertMode == A320_VERT_GS_STAR || St.vertMode == A320_VERT_GS ||
		St.vertMode == A320_VERT_LAND || St.vertMode == A320_VERT_FLARE;
	DrawWindow(FcuWindows[0]);
	DrawWindow(FcuWindows[1]);
	DrawButton(TEXT("LOC"), EA320Command::FcuLoc, bLocLit && !bApprLit);
	DrawButton(TEXT("AP1"), EA320Command::FcuAp, St.ap1Engaged != 0);
	DrawButton(TEXT("AP2"), EA320Command::FcuAp2, St.ap2Engaged != 0);
	DrawButton(TEXT("A/THR"), EA320Command::FcuAthr, St.athrEngaged != 0);
	DrawWindow(FcuWindows[2]);
	DrawButton(TEXT("APPR"), EA320Command::FcuAppr, bApprLit);
	DrawWindow(FcuWindows[3]);
}

void AA320Hud::DrawFma(const A320State& St, double X, double Y, double S)
{
	// LAND, FLARE and ROLL OUT span the vertical and lateral columns.
	const bool bCombined = St.vertMode == A320_VERT_LAND || St.vertMode == A320_VERT_FLARE || St.latMode == A320_LAT_ROLLOUT;
	for (int32 i = 1; i < 5; ++i)
	{
		if (i == 2 && bCombined)
		{
			continue;
		}
		Line(X + 0.2 * S * i, Y + 0.01 * S, X + 0.2 * S * i, Y + 0.11 * S, Grey, 1.0);
	}
	const double Row1 = Y + 0.03 * S, Row2 = Y + 0.075 * S;
	const double Now = GetWorld()->GetRealTimeSeconds();
	// A mode that just engaged is boxed in white for 10 s, as on the aircraft.
	auto Mode = [&](int32 Column, const FString& Str, double MX, double BoxW, const FLinearColor& Color)
	{
		if (Str != FmaShown[Column])
		{
			FmaShown[Column] = Str;
			FmaChangedAt[Column] = Now;
		}
		Text(Str, MX, Row1, Color, 0, 1);
		if (!Str.IsEmpty() && Now - FmaChangedAt[Column] < 10.0)
		{
			Frame(MX - BoxW / 2.0, Row1 - 0.018 * S, BoxW, 0.036 * S, White, 1.0);
		}
	};

	// Column 1: thrust.
	FString Thrust;
	FLinearColor ThrustColor = Green;
	if (St.reverse) Thrust = TEXT("REV");
	else if (St.athrEngaged && St.athrActive) Thrust = UTF8_TO_TCHAR(a320_athr_mode_name(St.athrMode));
	else if (St.thrustDetent == 3) { Thrust = TEXT("MAN TOGA"); ThrustColor = White; }
	else if (St.thrustDetent == 2) { Thrust = St.onGround ? TEXT("MAN FLX") : TEXT("MAN MCT"); ThrustColor = White; }
	if (!Thrust.IsEmpty() || !(St.athrEngaged && !St.onGround))
	{
		Mode(0, Thrust, X + 0.1 * S, 0.18 * S, ThrustColor);
		if (St.athrMode == A320_ATHR_AFLOOR && FMath::Fmod(Now, 0.8) < 0.5)
		{
			// Alpha floor: amber flashing box around A.FLOOR.
			Frame(X + 0.01 * S, Row1 - 0.02 * S, 0.18 * S, 0.04 * S, Amber, 2.0);
		}
	}
	else if (FMath::Fmod(Now, 1.0) < 0.6)
	{
		// A/THR armed with the levers out of the CL detent: LVR CLB flashes.
		Text(TEXT("LVR CLB"), X + 0.1 * S, Row1, White, 0, 1);
	}

	// Columns 2-3: vertical and lateral, armed modes in cyan.
	if (bCombined)
	{
		Mode(1, St.latMode == A320_LAT_ROLLOUT ? FString(TEXT("ROLL OUT")) : FString(UTF8_TO_TCHAR(a320_vert_mode_name(St.vertMode))),
			X + 0.4 * S, 0.3 * S, Green);
		FmaShown[2].Reset();
	}
	else
	{
		Mode(1, UTF8_TO_TCHAR(a320_vert_mode_name(St.vertMode)), X + 0.3 * S, 0.18 * S, Green);
		Mode(2, UTF8_TO_TCHAR(a320_lat_mode_name(St.latMode)), X + 0.5 * S, 0.18 * S, Green);
		FString ArmedV;
		if (St.armed & A320_ARMED_GS) ArmedV += TEXT("G/S ");
		else if (St.armed & A320_ARMED_ALT) ArmedV += TEXT("ALT");
		Text(ArmedV, X + 0.3 * S, Row2, Cyan, 0, 1);
		Text((St.armed & A320_ARMED_LOC) ? TEXT("LOC") : TEXT(""), X + 0.5 * S, Row2, Cyan, 0, 1);
	}

	// Column 4: approach capability once APPR is armed or engaged.
	const bool bAppr = (St.armed & A320_ARMED_GS) || St.vertMode == A320_VERT_GS_STAR || St.vertMode == A320_VERT_GS || bCombined;
	FString Cat, CatDetail;
	if (bAppr)
	{
		Cat = St.apEngaged && St.athrEngaged ? TEXT("CAT 3") : TEXT("CAT 1");
		CatDetail = St.ap1Engaged && St.ap2Engaged && St.athrEngaged ? TEXT("DUAL") : (St.apEngaged && St.athrEngaged ? TEXT("SINGLE") : TEXT(""));
	}
	Mode(3, Cat, X + 0.7 * S, 0.18 * S, White);
	Text(CatDetail, X + 0.7 * S, Row2, White, 0, 1);
	// Third line: the minimums from the MCDU PERF APPR page.
	if (bAppr && St.dhFt > 0)
	{
		Text(FString::Printf(TEXT("DH %d"), St.dhFt), X + 0.7 * S, Row2 + (Row2 - Row1), Cyan, 0, 1);
	}
	else if (bAppr && St.mdaFt > 0)
	{
		Text(FString::Printf(TEXT("BARO %d"), St.mdaFt), X + 0.7 * S, Row2 + (Row2 - Row1), Cyan, 0, 1);
	}
	else if (bAppr && St.dhFt == 0)
	{
		Text(TEXT("NO DH"), X + 0.7 * S, Row2 + (Row2 - Row1), Cyan, 0, 1);
	}

	// Column 5: engagement status.
	Mode(4, St.ap1Engaged && St.ap2Engaged ? TEXT("AP1+2") : (St.ap1Engaged ? TEXT("AP1") : (St.ap2Engaged ? TEXT("AP2") : TEXT(""))),
		X + 0.9 * S, 0.18 * S, White);
	if (St.athrEngaged)
	{
		Text(TEXT("A/THR"), X + 0.9 * S, Row2, St.athrActive ? White : Cyan, 0, 1);
	}
}

EA320Lever AA320Hud::LeverAt(const FVector2D& ScreenPos) const
{
	// Only called when no button was hit, so a containing button is a pop-up panel's background
	// (the MCDU sits over the pedestal): no lever behind it.
	for (const FButton& Button : Buttons)
	{
		if (Button.Box.IsInside(ScreenPos))
		{
			return EA320Lever::None;
		}
	}
	for (const FLeverSlot& Slot : Levers)
	{
		if (Slot.Box.IsInside(ScreenPos))
		{
			return Slot.Lever;
		}
	}
	return EA320Lever::None;
}

double AA320Hud::LeverPosition(EA320Lever Lever, const FVector2D& ScreenPos) const
{
	for (const FLeverSlot& Slot : Levers)
	{
		if (Slot.Lever == Lever)
		{
			const double Height = FMath::Max(Slot.Box.Max.Y - Slot.Box.Min.Y, 1.0);
			return FMath::Clamp((ScreenPos.Y - Slot.Box.Min.Y) / Height, 0.0, 1.0);
		}
	}
	return 0.0;
}

void AA320Hud::LeverSlot(double X, double Y, double W, double H, EA320Lever Lever)
{
	Fill(X + W * 0.42, Y, W * 0.16, H, Screen);
	Levers.Add({FBox2D(FVector2D(X, Y), FVector2D(X + W, Y + H)), Lever});
}

void AA320Hud::Pushbutton(double X, double Y, double W, double H, const FString& Name, const FString& Upper,
	const FLinearColor& UpperColor, const FString& Lower, const FLinearColor& LowerColor, EA320Command Command)
{
	Text(Name, X + W / 2.0, Y - 0.012 * Canvas->ClipY, White, 0, 1);
	Fill(X, Y, W, H, FLinearColor(0.07f, 0.07f, 0.08f));
	Frame(X, Y, W, H, FLinearColor(0.45f, 0.47f, 0.5f), 1.5);
	Text(Upper, X + W / 2.0, Y + H * 0.3, UpperColor, 0, 1);
	Text(Lower, X + W / 2.0, Y + H * 0.72, LowerColor, 0, 1);
	Buttons.Add({FBox2D(FVector2D(X, Y), FVector2D(X + W, Y + H)), Command});
}

void AA320Hud::Switch(double X, double Y, double W, double H, const FString& Name, const FString& Position, EA320Command Command)
{
	Text(Name, X + W / 2.0, Y - 0.012 * Canvas->ClipY, White, 0, 1);
	Fill(X, Y, W, H, FLinearColor(0.12f, 0.13f, 0.14f));
	Frame(X, Y, W, H, FLinearColor(0.45f, 0.47f, 0.5f), 1.0);
	// The toggle itself: a short bar, up for ON/positions other than OFF.
	const bool bOff = Position == TEXT("OFF");
	Fill(X + W * 0.44, bOff ? Y + H * 0.55 : Y + H * 0.12, W * 0.12, H * 0.33, FLinearColor(0.75f, 0.76f, 0.78f));
	Text(Position, X + W / 2.0, bOff ? Y + H * 0.3 : Y + H * 0.75, bOff ? Grey : White, 0, 1);
	Buttons.Add({FBox2D(FVector2D(X, Y), FVector2D(X + W, Y + H)), Command});
}

void AA320Hud::DrawCenterPanel(const AA320Aircraft& Aircraft, double X, double Y, double W, double H)
{
	const A320State& St = Aircraft.GetSimState();
	const A320Controls& Ctl = Aircraft.GetSimControls();
	Fill(X, Y, W, H, FLinearColor(0.09f, 0.095f, 0.1f));

	// LDG GEAR indicator: green triangles when down and locked, red UNLK in transit.
	const bool bDown = St.gearPos > 0.99, bUp = St.gearPos < 0.01;
	for (int32 k = 0; k < 3; ++k)
	{
		const double GX = X + W * (0.2 + 0.3 * k);
		const double GY = Y + H * 0.07;
		if (bDown)
		{
			Triangle(FVector2D(GX - 0.06 * W, GY - 0.02 * H), FVector2D(GX + 0.06 * W, GY - 0.02 * H), FVector2D(GX, GY + 0.03 * H), Green);
		}
		else if (!bUp)
		{
			Text(TEXT("UNLK"), GX, GY, Red, 0, 1);
		}
	}

	// Gear lever: the wheel-shaped handle sits at the top (UP) or bottom (DOWN) of its slot.
	const double LX = X + W * 0.3, LY = Y + H * 0.14, LW = W * 0.4, LH = H * 0.46;
	Fill(LX + LW * 0.42, LY, LW * 0.16, LH, Screen);
	const double HandleY = Ctl.gearDown ? LY + LH - 0.06 * H : LY + 0.06 * H;
	Fill(LX, HandleY - 0.045 * H, LW, 0.09 * H, FLinearColor(0.8f, 0.8f, 0.82f));
	Text(Ctl.gearDown ? TEXT("DOWN") : TEXT("UP"), LX + LW / 2.0, HandleY, FLinearColor(0.05f, 0.05f, 0.05f), 0, 1);
	Text(TEXT("L/G"), X + W * 0.12, LY + LH / 2.0, White, 0, 1);
	Buttons.Add({FBox2D(FVector2D(LX, LY), FVector2D(LX + LW, LY + LH)), EA320Command::GearToggle});

	// AUTO/BRK: LO, MED, MAX.
	Text(TEXT("AUTO/BRK"), X + W / 2.0, Y + H * 0.66, White, 0, 1);
	const EA320Command Cmds[] = {EA320Command::AutobrakeLo, EA320Command::AutobrakeMed, EA320Command::AutobrakeMax};
	const TCHAR* Names[] = {TEXT("LO"), TEXT("MED"), TEXT("MAX")};
	const double BW = W * 0.29, BH = H * 0.2;
	for (int32 k = 0; k < 3; ++k)
	{
		const bool bOn = St.autobrake == k + 1;
		const bool bDecel = bOn && St.autobrakeDecel;
		Pushbutton(X + W * 0.03 + k * (BW + W * 0.03), Y + H * 0.76, BW, BH, Names[k], bDecel ? TEXT("DECEL") : TEXT(""), Green,
			bOn ? TEXT("ON") : TEXT(""), Cyan, Cmds[k]);
	}
}

void AA320Hud::DrawPedestal(const AA320Aircraft& Aircraft, double X, double Y, double W, double H)
{
	const A320State& St = Aircraft.GetSimState();
	const A320Controls& Ctl = Aircraft.GetSimControls();
	Fill(X, Y, W, H, FLinearColor(0.09f, 0.095f, 0.1f));
	const FLinearColor Handle(0.8f, 0.8f, 0.82f);
	const FLinearColor HandleText(0.05f, 0.05f, 0.05f);
	const double TopY = Y + H * 0.08, SlotH = H * 0.55;

	// Speedbrake lever: RET at the top, then 1/2 and FULL; ARM is the lever pulled up at RET.
	{
		const double SX = X + W * 0.02, SW = W * 0.16;
		Text(TEXT("SPEED BRAKE"), SX + SW / 2.0, Y + H * 0.03, White, 0, 1);
		LeverSlot(SX, TopY, SW, SlotH, EA320Lever::Speedbrake);
		Text(TEXT("RET"), SX, TopY + 0.02 * H, Grey, 0, 0);
		Text(TEXT("1/2"), SX, TopY + SlotH * 0.5, Grey, 0, 0);
		Text(TEXT("FULL"), SX, TopY + SlotH - 0.02 * H, Grey, 0, 0);
		const double HY = TopY + Ctl.speedbrake * SlotH;
		Fill(SX + SW * 0.15, HY - 0.025 * H, SW * 0.7, 0.05 * H, Ctl.spoilersArmed ? Cyan : Handle);
		Pushbutton(SX, TopY + SlotH + 0.025 * H, SW, 0.09 * H, TEXT(""), TEXT(""), Green, Ctl.spoilersArmed ? TEXT("ARMED") : TEXT("ARM"),
			Ctl.spoilersArmed ? Cyan : White, EA320Command::SpoilerArm);
	}

	// Thrust levers: TOGA, FLX/MCT, CL, IDLE detents; below IDLE the reverse range (on ground).
	{
		const double TX = X + W * 0.22, TW = W * 0.42;
		constexpr double ForwardSpan = 0.72;
		Text(TEXT("THRUST LEVERS"), TX + TW / 2.0, Y + H * 0.03, White, 0, 1);
		LeverSlot(TX, TopY, TW, SlotH, EA320Lever::Thrust);
		Fill(TX + TW * 0.42, TopY + SlotH * ForwardSpan, TW * 0.16, SlotH * (1.0 - ForwardSpan), FLinearColor(0.25f, 0.05f, 0.05f));
		struct FDetent
		{
			double Lever;
			const TCHAR* Name;
		};
		const FDetent Detents[] = {{1.0, TEXT("TOGA")}, {0.88, TEXT("FLX/MCT")}, {0.75, TEXT("CL")}, {0.0, TEXT("IDLE")}};
		for (const FDetent& D : Detents)
		{
			const double DY = TopY + (1.0 - D.Lever) * ForwardSpan * SlotH;
			Line(TX + TW * 0.3, DY, TX + TW * 0.7, DY, Grey, 1.0);
			Text(D.Name, TX + TW * 0.28, DY, Grey, 0, 2);
		}
		Text(TEXT("REV"), TX + TW * 0.28, TopY + SlotH * 0.9, Grey, 0, 2);
		// One handle per engine; they only separate with a two-lever hardware quadrant.
		double HY = TopY + SlotH;
		for (int32 e = 0; e < 2; ++e)
		{
			const double Lev = St.thrustLeverEng[e];
			const double Pos = St.reverseEng[e] ? ForwardSpan + Lev * (1.0 - ForwardSpan) : (1.0 - Lev) * ForwardSpan;
			const double EngY = TopY + Pos * SlotH;
			const double Side = e == 0 ? 0.33 : 0.67;
			Fill(TX + TW * (Side - 0.13), EngY - 0.03 * H, TW * 0.26, 0.06 * H, Handle);
			HY = FMath::Min(HY, EngY);
		}
		Text(St.athrActive ? TEXT("A/THR") : TEXT(""), TX + TW * 0.5, HY, HandleText, 0, 1);
		Text(FString::Printf(TEXT("%s"), UTF8_TO_TCHAR(a320_thrust_detent_name(St.thrustDetent))), TX + TW * 0.86, HY, Cyan, 0, 1);
	}

	// Flaps lever: 0, 1, 2, 3, FULL.
	{
		const double FX = X + W * 0.68, FW = W * 0.3;
		Text(TEXT("FLAPS"), FX + FW / 2.0, Y + H * 0.03, White, 0, 1);
		LeverSlot(FX, TopY, FW, SlotH, EA320Lever::Flaps);
		const TCHAR* Labels[] = {TEXT("0"), TEXT("1"), TEXT("2"), TEXT("3"), TEXT("FULL")};
		for (int32 k = 0; k <= 4; ++k)
		{
			const double DY = TopY + SlotH * k / 4.0;
			Line(FX + FW * 0.3, DY, FX + FW * 0.7, DY, Grey, 1.0);
			Text(Labels[k], FX + FW * 0.27, DY, Grey, 0, 2);
		}
		const double HY = TopY + SlotH * Ctl.flapsLever / 4.0;
		Fill(FX + FW * 0.25, HY - 0.025 * H, FW * 0.5, 0.05 * H, Handle);
		Text(UTF8_TO_TCHAR(a320_flap_config_name(St.flapsLever, St.onePlusF)), FX + FW * 0.5, HY, HandleText, 0, 1);
	}

	// Engine masters, ENG MODE selector and the parking brake.
	const double RowY = Y + H * 0.8, RowH = H * 0.16;
	const double Cell = W / 6.0;
	auto Master = [&](int32 Index, double CellX)
	{
		const bool bOn = Ctl.engMaster[Index] != 0;
		Switch(CellX + Cell * 0.1, RowY, Cell * 0.8, RowH, FString::Printf(TEXT("ENG %d"), Index + 1), bOn ? TEXT("ON") : TEXT("OFF"),
			Index == 0 ? EA320Command::EngMaster1 : EA320Command::EngMaster2);
	};
	Master(0, X);
	const TCHAR* Modes[] = {TEXT("CRANK"), TEXT("NORM"), TEXT("IGN/START")};
	const EA320Command ModeCmds[] = {EA320Command::EngModeCrank, EA320Command::EngModeNorm, EA320Command::EngModeIgnStart};
	Text(TEXT("ENG MODE"), X + Cell * 2.5, RowY - 0.012 * Canvas->ClipY, White, 0, 1);
	for (int32 k = 0; k < 3; ++k)
	{
		AddButton(X + Cell * (1.05 + k * 0.97), RowY, Cell * 0.92, RowH, Modes[k], ModeCmds[k], Ctl.engMode == k);
	}
	Master(1, X + Cell * 4.0);
	Switch(X + Cell * 5.05, RowY, Cell * 0.9, RowH, TEXT("PARK BRK"), Ctl.parkBrake ? TEXT("ON") : TEXT("OFF"), EA320Command::ParkBrakeToggle);
}

void AA320Hud::DrawSimBar(const AA320Aircraft& Aircraft)
{
	// Simulator functions (not cockpit controls), top right.
	const A320State& St = Aircraft.GetSimState();
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	Scale = FMath::Max(H / 1080.0, 0.6);
	struct FItem
	{
		FString Label;
		EA320Command Command;
		bool bLit;
	};
	const FItem Items[] = {
		{St.paused ? TEXT("PAUSED") : TEXT("PAUSE"), EA320Command::PauseToggle, St.paused != 0},
		{FString::Printf(TEXT("SIM x%d"), FMath::Max(1, FMath::RoundToInt(St.simRate))), EA320Command::SimRateCycle, St.simRate > 1.0},
		{Aircraft.IsCockpitView() ? TEXT("VIEW: CKPT") : TEXT("VIEW: EXT"), EA320Command::ViewToggle, false},
		{TEXT("OVERHEAD"), EA320Command::OverheadToggle, Aircraft.IsOverheadVisible()},
		{TEXT("MCDU"), EA320Command::McduToggle, Aircraft.IsMcduVisible()},
		{TEXT("RADIO"), EA320Command::RadioToggle, Aircraft.IsRadioVisible() || Aircraft.GetSimState().atcAwaitingReadback != 0},
		{TEXT("JOYSTICK"), EA320Command::JoystickPanel, false},
		{Aircraft.IsSoundOn() ? TEXT("SOUND ON") : TEXT("SOUND OFF"), EA320Command::SoundToggle, !Aircraft.IsSoundOn()},
		{TEXT("HELP"), EA320Command::HelpToggle, Aircraft.IsHelpVisible()},
		{TEXT("LESSONS"), EA320Command::GuideMenu, Aircraft.IsGuideMenuVisible() || Aircraft.GetGuideStatus().active != 0},
		{TEXT("SWAP RWY"), EA320Command::RunwaySwap, false},
		{TEXT("LINE UP"), EA320Command::ResetRunway, false},
		{TEXT("COLD+DARK"), EA320Command::ResetColdDark, false},
		{TEXT("APPROACH"), EA320Command::ResetApproach, false},
		{TEXT("FINAL 10"), EA320Command::ResetFinal10, false},
		{TEXT("FINAL 4"), EA320Command::ResetFinal4, false},
	};
	const int32 Count = UE_ARRAY_COUNT(Items);
	const double BW = FMath::Min(W * 0.055, 120.0 * Scale), BH = 30.0 * Scale, Gap = 4.0 * Scale;
	double BX = W - Count * (BW + Gap) - 8.0 * Scale;
	for (const FItem& Item : Items)
	{
		if (Item.Command == EA320Command::McduToggle && !Aircraft.IsMcduVisible())
		{
			MarkTarget(A320_GT_MCDU, BX, 8.0 * Scale, BW, BH);  // the open MCDU marks itself
		}
		if (Item.Command == EA320Command::RadioToggle && !Aircraft.IsRadioVisible())
		{
			MarkTarget(A320_GT_RADIO, BX, 8.0 * Scale, BW, BH);  // the open window marks its parts
			MarkTarget(A320_GT_ATC_REPLY, BX, 8.0 * Scale, BW, BH);
		}
		AddButton(BX, 8.0 * Scale, BW, BH, Item.Label, Item.Command, Item.bLit);
		BX += BW + Gap;
	}
}

void AA320Hud::DrawOverhead(const AA320Aircraft& Aircraft)
{
	const A320State& St = Aircraft.GetSimState();
	const A320Controls& Ctl = Aircraft.GetSimControls();
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	const double OX = W * 0.17, OY = H * 0.075, OW = W * 0.66, OH = H * 0.47;
	Fill(OX, OY, OW, OH, FLinearColor(0.2f, 0.23f, 0.26f, 0.97f));
	Buttons.Add({FBox2D(FVector2D(OX, OY), FVector2D(OX + OW, OY + OH)), EA320Command::None});  // swallows clicks
	Frame(OX, OY, OW, OH, FLinearColor(0.5f, 0.52f, 0.55f), 2.0);
	Text(TEXT("OVERHEAD PANEL"), OX + OW / 2.0, OY + 0.025 * H, White, 1, 1);
	AddButton(OX + OW - 0.04 * W, OY + 0.008 * H, 0.032 * W, 0.034 * H, TEXT("X"), EA320Command::OverheadToggle, false);

	const double PB = FMath::Min(OW * 0.085, OH * 0.2);  // pushbutton size
	auto Group = [&](const TCHAR* Title, double BoxX, double BoxY, double BoxW, double BoxH)
	{
		Frame(BoxX, BoxY, BoxW, BoxH, FLinearColor(0.85f, 0.85f, 0.85f), 1.0);
		Text(Title, BoxX + 6.0, BoxY + 0.012 * H, White, 0, 0);
	};
	const FLinearColor PbBlue = Cyan;  // Airbus "ON" legends are blue

	// APU and bleed.
	double GY = OY + 0.07 * H;
	Group(TEXT("APU"), OX + OW * 0.03, GY, OW * 0.36, OH * 0.3);
	Pushbutton(OX + OW * 0.06, GY + 0.05 * H, PB, PB * 0.8, TEXT("MASTER SW"), TEXT(""), Amber,
		Ctl.apuMaster ? TEXT("ON") : TEXT(""), PbBlue, EA320Command::ApuMaster);
	Pushbutton(OX + OW * 0.17, GY + 0.05 * H, PB, PB * 0.8, TEXT("START"), St.apuAvail ? TEXT("AVAIL") : TEXT(""), Green,
		St.apuStarting ? TEXT("ON") : TEXT(""), PbBlue, EA320Command::ApuStart);
	Text(FString::Printf(TEXT("N %d%%"), FMath::RoundToInt(St.apuN)), OX + OW * 0.32, GY + 0.05 * H + PB * 0.4,
		St.apuAvail ? Green : (St.apuN > 1.0 ? Amber : Grey), 1, 1);
	Group(TEXT("AIR COND"), OX + OW * 0.42, GY, OW * 0.17, OH * 0.3);
	Pushbutton(OX + OW * 0.46, GY + 0.05 * H, PB, PB * 0.8, TEXT("APU BLEED"), TEXT(""), Amber,
		Ctl.apuBleed ? TEXT("ON") : TEXT(""), PbBlue, EA320Command::ApuBleed);
	Group(TEXT("SIGNS"), OX + OW * 0.62, GY, OW * 0.35, OH * 0.3);
	const double SwW = OW * 0.11, SwH = OH * 0.16;
	Switch(OX + OW * 0.66, GY + 0.05 * H, SwW, SwH, TEXT("SEAT BELTS"), (Ctl.signs & A320_SIGN_SEATBELTS) ? TEXT("ON") : TEXT("OFF"), EA320Command::SignSeatbelts);
	Switch(OX + OW * 0.82, GY + 0.05 * H, SwW, SwH, TEXT("NO SMOKING"), (Ctl.signs & A320_SIGN_NO_SMOKING) ? TEXT("ON") : TEXT("OFF"), EA320Command::SignNoSmoking);

	// Exterior lights.
	GY += OH * 0.36;
	Group(TEXT("EXT LT"), OX + OW * 0.03, GY, OW * 0.94, OH * 0.34);
	struct FLight
	{
		const TCHAR* Name;
		int32 Bit;
		EA320Command Command;
	};
	const FLight LightSwitches[] = {
		{TEXT("STROBE"), A320_LT_STROBE, EA320Command::LightStrobe},
		{TEXT("BEACON"), A320_LT_BEACON, EA320Command::LightBeacon},
		{TEXT("NAV & LOGO"), A320_LT_NAV, EA320Command::LightNav},
		{TEXT("RWY TURN OFF"), A320_LT_RWY_TURNOFF, EA320Command::LightRwyTurnoff},
		{TEXT("LAND"), A320_LT_LANDING, EA320Command::LightLanding},
	};
	double SX = OX + OW * 0.06;
	for (const FLight& Light : LightSwitches)
	{
		Switch(SX, GY + 0.05 * H, SwW, SwH, Light.Name, (Ctl.lights & Light.Bit) ? TEXT("ON") : TEXT("OFF"), Light.Command);
		SX += OW * 0.15;
	}
	const TCHAR* Nose = (Ctl.lights & A320_LT_TAKEOFF) ? TEXT("T.O") : ((Ctl.lights & A320_LT_TAXI) ? TEXT("TAXI") : TEXT("OFF"));
	Switch(SX, GY + 0.05 * H, SwW, SwH, TEXT("NOSE"), Nose, EA320Command::LightNose);

	Text(TEXT("Cold start: APU MASTER SW, START, wait for AVAIL, APU BLEED; ENG MODE IGN/START, ENG 1 then ENG 2 ON; ENG MODE NORM."),
		OX + OW / 2.0, OY + OH - 0.03 * H, FLinearColor(0.8f, 0.85f, 0.9f), 0, 1);
}

void AA320Hud::DrawMcdu(const AA320Aircraft& Aircraft)
{
	A320McduDisplay D;
	Aircraft.GetMcduDisplay(D);
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	// On the right, over the E/WD side, in the unit's tall proportions.
	const double MH = H * 0.9, MW = FMath::Min(MH * 0.62, W * 0.42);
	const double MX = W - MW - 0.01 * W, MY = 0.05 * H;
	Fill(MX, MY, MW, MH, FLinearColor(0.16f, 0.17f, 0.19f, 0.98f));
	MarkTarget(A320_GT_MCDU, MX, MY, MW, MH);
	Buttons.Add({FBox2D(FVector2D(MX, MY), FVector2D(MX + MW, MY + MH)), EA320Command::None});  // swallows clicks
	Frame(MX, MY, MW, MH, FLinearColor(0.5f, 0.52f, 0.55f), 2.0);
	AddButton(MX + MW - 0.07 * MW, MY + 0.005 * MH, 0.06 * MW, 0.03 * MH, TEXT("X"), EA320Command::McduToggle, false);

	// Screen: 24 columns x 14 rows, one cell per character.
	const double Pad = 0.02 * MW, LskW = 0.07 * MW, Gap = 0.015 * MW;
	const double SX = MX + Pad + LskW + Gap, SW = MW - 2.0 * (Pad + LskW + Gap);
	const double SY = MY + 0.04 * MH, SH = SW * 0.8;
	Fill(SX, SY, SW, SH, Screen);
	Frame(SX, SY, SW, SH, Grey, 1.0);
	const double CellW = SW / A320_MCDU_COLS, CellH = SH / A320_MCDU_ROWS;
	const FLinearColor Colors[] = {White, Cyan, Green, Amber, Magenta, Yellow};
	const int32 ColorCount = static_cast<int32>(UE_ARRAY_COUNT(Colors));
	for (int32 Row = 0; Row < A320_MCDU_ROWS; ++Row)
	{
		for (int32 Col = 0; Col < A320_MCDU_COLS; ++Col)
		{
			const ANSICHAR Ch = D.text[Row][Col];
			if (Ch == ' ' || Ch == '\0')
			{
				continue;
			}
			const FLinearColor& Color = Colors[FMath::Min<int32>(D.color[Row][Col], ColorCount - 1)];
			const double CX = SX + (Col + 0.5) * CellW, CY = SY + (Row + 0.5) * CellH;
			if (Ch == '#')
			{
				Frame(CX - 0.38 * CellW, CY - 0.36 * CellH, 0.76 * CellW, 0.72 * CellH, Amber, 1.0);  // entry box
				continue;
			}
			const TCHAR Glyph = Ch == '`' ? TEXT('\u00B0') : static_cast<TCHAR>(Ch);
			Text(FString(1, &Glyph), CX, CY, Color, D.small[Row][Col] ? 0 : 1, 1);
		}
	}

	// Line select keys beside the data lines (rows 2, 4 .. 12).
	const double LskH = 1.3 * CellH;
	for (int32 Lsk = 0; Lsk < 6; ++Lsk)
	{
		const double KY = SY + (2 * Lsk + 2.5) * CellH - LskH / 2.0;
		AddButton(MX + Pad, KY, LskW, LskH, TEXT("-"), EA320Command::McduKey, false, A320_MCDU_LSK1L + Lsk);
		AddButton(MX + MW - Pad - LskW, KY, LskW, LskH, TEXT("-"), EA320Command::McduKey, false, A320_MCDU_LSK1R + Lsk);
	}

	// Page keys (as on the unit), then the keypad.
	struct FKeyDef
	{
		const TCHAR* Label;
		int32 Key;
	};
	const FKeyDef PageKeys[] = {
		{TEXT("DIR"), A320_MCDU_DIR}, {TEXT("PROG"), A320_MCDU_PROG}, {TEXT("PERF"), A320_MCDU_PERF},
		{TEXT("INIT"), A320_MCDU_INIT}, {TEXT("DATA"), A320_MCDU_DATA}, {TEXT(""), 0},
		{TEXT("F-PLN"), A320_MCDU_FPLN}, {TEXT("RAD NAV"), A320_MCDU_RADNAV}, {TEXT("FUEL PRED"), A320_MCDU_FUELPRED},
		{TEXT("SEC F-PLN"), A320_MCDU_SECFPLN}, {TEXT("ATC COMM"), A320_MCDU_ATCCOMM}, {TEXT("MENU"), A320_MCDU_MENU},
		{TEXT("AIRPORT"), A320_MCDU_AIRPORT}, {TEXT(""), 0}, {TEXT(""), 0},
		{TEXT("UP"), A320_MCDU_UP}, {TEXT("NEXT PAGE"), A320_MCDU_NEXTPAGE}, {TEXT("DOWN"), A320_MCDU_DOWN},
	};
	const double KeysTop = SY + SH + 0.02 * MH;
	const double PkW = (MW - 2.0 * Pad) / 6.0, PkH = 0.045 * MH;
	const int32 PageKeyCount = static_cast<int32>(UE_ARRAY_COUNT(PageKeys));
	for (int32 i = 0; i < PageKeyCount; ++i)
	{
		if (PageKeys[i].Key == 0)
		{
			continue;
		}
		const double KX = MX + Pad + (i % 6) * PkW, KY = KeysTop + (i / 6) * (PkH + 0.006 * MH);
		AddButton(KX + 2.0, KY, PkW - 4.0, PkH, PageKeys[i].Label, EA320Command::McduKey, false, PageKeys[i].Key);
	}

	// Keypad: digits on the left, letters and CLR on the right.
	const double PadTop = KeysTop + 3.0 * (PkH + 0.006 * MH) + 0.015 * MH;
	const double KeyH = FMath::Min(0.045 * MH, (MY + MH - 0.035 * MH - PadTop) / 6.0 - 0.006 * MH);
	const double KeyW = (MW - 2.0 * Pad) / 8.4;
	const TCHAR* Digits[] = {TEXT("1"), TEXT("2"), TEXT("3"), TEXT("4"), TEXT("5"), TEXT("6"), TEXT("7"), TEXT("8"), TEXT("9"),
		TEXT("."), TEXT("0"), TEXT("+/-")};
	const int32 DigitCount = static_cast<int32>(UE_ARRAY_COUNT(Digits));
	for (int32 i = 0; i < DigitCount; ++i)
	{
		const int32 Code = i == 11 ? A320_MCDU_PLUSMINUS : static_cast<int32>(Digits[i][0]);
		const double KX = MX + Pad + (i % 3) * KeyW, KY = PadTop + (i / 3) * (KeyH + 0.006 * MH);
		AddButton(KX + 2.0, KY, KeyW - 4.0, KeyH, Digits[i], EA320Command::McduKey, false, Code);
	}
	const double LettersX = MX + Pad + 3.4 * KeyW;
	for (int32 i = 0; i < 30; ++i)
	{
		FString Label;
		int32 Code = 0;
		if (i < 26)
		{
			Label = FString::Chr(static_cast<TCHAR>('A' + i));
			Code = 'A' + i;
		}
		else
		{
			const TCHAR* Extra[] = {TEXT("SP"), TEXT("/"), TEXT("OVFY"), TEXT("CLR")};
			const int32 ExtraCodes[] = {' ', '/', A320_MCDU_OVFY, A320_MCDU_CLR};
			Label = Extra[i - 26];
			Code = ExtraCodes[i - 26];
		}
		const double KX = LettersX + (i % 5) * KeyW, KY = PadTop + (i / 5) * (KeyH + 0.006 * MH);
		AddButton(KX + 2.0, KY, KeyW - 4.0, KeyH, Label, EA320Command::McduKey, false, Code);
	}
	Text(TEXT("Keyboard types here   Backspace = CLR   Tab / Esc closes"), MX + MW / 2.0, MY + MH - 0.015 * MH,
		FLinearColor(0.75f, 0.8f, 0.85f), 0, 1);
}

namespace
{
	FString FreqText(int32 Khz)
	{
		return FString::Printf(TEXT("%03d.%03d"), Khz / 1000, Khz % 1000);
	}
}

void AA320Hud::DrawRadio(const AA320Aircraft& Aircraft)
{
	const A320State& St = Aircraft.GetSimState();
	const A320Controls& Ctl = Aircraft.GetSimControls();
	const A320AtcStatus Atc = Aircraft.GetAtcStatus();
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	// Left side, so the MCDU (right) can stay open with it.
	const double RX = 0.01 * W, RY = 0.06 * H, RW = FMath::Min(0.42 * W, 860.0 * Scale), RH = 0.62 * H;
	const double Pad = 10.0 * Scale;
	Fill(RX, RY, RW, RH, FLinearColor(0.08f, 0.09f, 0.1f, 0.97f));
	Buttons.Add({FBox2D(FVector2D(RX, RY), FVector2D(RX + RW, RY + RH)), EA320Command::None});  // swallows clicks
	Frame(RX, RY, RW, RH, FLinearColor(0.5f, 0.52f, 0.55f), 2.0);
	Text(FString::Printf(TEXT("RADIO   callsign %s"), UTF8_TO_TCHAR(Atc.callsign)), RX + Pad, RY + 16.0 * Scale, White, 1, 0);
	const double SmallBtn = 30.0 * Scale;
	AddButton(RX + RW - 1.2 * SmallBtn - Pad, RY + 4.0 * Scale, 1.2 * SmallBtn, 0.9 * SmallBtn, TEXT("X"), EA320Command::RadioToggle, false);
	AddButton(RX + RW - 4.4 * SmallBtn - Pad, RY + 4.0 * Scale, 3.0 * SmallBtn, 0.9 * SmallBtn,
		St.atcEnabled ? TEXT("ATC ON") : TEXT("ATC OFF"), EA320Command::AtcToggle, St.atcEnabled != 0);

	// VHF 1 radio management panel: ACTIVE <-> STBY, set STBY with the knobs.
	const double PanelY = RY + 36.0 * Scale, PanelH = 96.0 * Scale;
	const double RmpW = RW * 0.58;
	Frame(RX + Pad, PanelY, RmpW - Pad, PanelH, Grey, 1.0);
	MarkTarget(A320_GT_RADIO, RX + Pad, PanelY, RW - 2.0 * Pad, PanelH);
	Text(TEXT("VHF 1   ACTIVE"), RX + 2.0 * Pad, PanelY + 12.0 * Scale, White, 0, 0);
	Text(TEXT("STBY"), RX + RmpW * 0.62, PanelY + 12.0 * Scale, White, 0, 0);
	const double DigitsY = PanelY + 38.0 * Scale;
	Fill(RX + 2.0 * Pad, DigitsY - 16.0 * Scale, RmpW * 0.36, 32.0 * Scale, Screen);
	Text(FreqText(Ctl.com1ActiveKhz), RX + 2.0 * Pad + RmpW * 0.18, DigitsY, Amber, 2, 1);
	Fill(RX + RmpW * 0.62, DigitsY - 16.0 * Scale, RmpW * 0.33, 32.0 * Scale, Screen);
	Text(FreqText(Ctl.com1StandbyKhz), RX + RmpW * 0.62 + RmpW * 0.165, DigitsY, Amber, 2, 1);
	AddButton(RX + RmpW * 0.47, DigitsY - 14.0 * Scale, RmpW * 0.12, 28.0 * Scale, TEXT("<->"), EA320Command::ComSwap, false);
	const FString StationName = UTF8_TO_TCHAR(Atc.station);
	Text(StationName.IsEmpty() ? FString(TEXT("no station")) : StationName, RX + 2.0 * Pad + RmpW * 0.18, DigitsY + 24.0 * Scale,
		StationName.IsEmpty() ? Grey : Green, 0, 1);
	const double KnobY = PanelY + PanelH - 30.0 * Scale, KnobW = RmpW * 0.085;
	const struct
	{
		const TCHAR* Label;
		EA320Command Command;
	} Knobs[] = {{TEXT("MHz-"), EA320Command::ComMhzDec}, {TEXT("MHz+"), EA320Command::ComMhzInc},
		{TEXT("kHz-"), EA320Command::ComKhzDec}, {TEXT("kHz+"), EA320Command::ComKhzInc}};
	double KX = RX + RmpW * 0.6;
	for (const auto& Knob : Knobs)
	{
		AddButton(KX, KnobY, KnobW, 24.0 * Scale, Knob.Label, Knob.Command, false);
		KX += KnobW + 3.0 * Scale;
	}

	// Transponder: four octal digits, then the mode.
	const double XX = RX + RmpW + Pad, XW = RW - RmpW - 2.0 * Pad;
	Frame(XX, PanelY, XW, PanelH, Grey, 1.0);
	Text(TEXT("ATC XPDR"), XX + Pad, PanelY + 12.0 * Scale, White, 0, 0);
	const FString& Entry = Aircraft.GetXpdrEntry();
	const FString Code = Entry.IsEmpty() ? FString::Printf(TEXT("%04d"), Ctl.xpdrCode) : (Entry + FString::ChrN(4 - Entry.Len(), TEXT('-')));
	Fill(XX + XW * 0.45, PanelY + 3.0 * Scale, XW * 0.5, 22.0 * Scale, Screen);
	Text(Code, XX + XW * 0.7, PanelY + 14.0 * Scale, Entry.IsEmpty() ? Amber : Cyan, 1, 1);
	const double KeyW = (XW - Pad) / 9.0 - 2.0 * Scale, KeyY = PanelY + 32.0 * Scale;
	for (int32 Digit = 0; Digit < 8; ++Digit)
	{
		AddButton(XX + Pad * 0.5 + Digit * (KeyW + 2.0 * Scale), KeyY, KeyW, 24.0 * Scale, FString::FromInt(Digit),
			EA320Command::XpdrDigit, false, Digit);
	}
	AddButton(XX + Pad * 0.5 + 8 * (KeyW + 2.0 * Scale), KeyY, KeyW, 24.0 * Scale, TEXT("CLR"), EA320Command::XpdrClear, false);
	const TCHAR* Modes[] = {TEXT("STBY"), TEXT("AUTO"), TEXT("ON")};
	const double ModeW = (XW - Pad) / 3.0 - 3.0 * Scale;
	for (int32 Mode = 0; Mode < 3; ++Mode)
	{
		AddButton(XX + Pad * 0.5 + Mode * (ModeW + 3.0 * Scale), KnobY, ModeW, 24.0 * Scale, Modes[Mode], EA320Command::XpdrMode,
			Ctl.xpdrMode == Mode, Mode);
	}
	if (St.atcSquawk > 0 && Ctl.xpdrCode != St.atcSquawk)
	{
		Text(FString::Printf(TEXT("cleared squawk %04d"), St.atcSquawk), XX + Pad, PanelY + PanelH + 10.0 * Scale, Amber, 0, 0);
	}

	// Replies at the bottom, the radio log above them.
	const double ReplyW = RW - 2.0 * Pad;
	double RepliesH = 26.0 * Scale;
	TArray<double> OptionH;
	for (int32 i = 0; i < Atc.optionCount; ++i)
	{
		const FString Option = FString::Printf(TEXT("%d  %s"), i + 1, UTF8_TO_TCHAR(Atc.options[i]));
		OptionH.Add(TextWrapped(Option, 0.0, 0.0, ReplyW - 2.0 * Pad, White, 0, false) + 8.0 * Scale);
		RepliesH += OptionH.Last() + 4.0 * Scale;
	}
	if (Atc.optionCount == 0)
	{
		RepliesH += 22.0 * Scale;
	}
	double Y = RY + RH - Pad - RepliesH;
	MarkTarget(A320_GT_ATC_REPLY, RX + Pad, Y, ReplyW, RepliesH);
	const bool bFlash = FMath::Fmod(GetWorld()->GetRealTimeSeconds(), 1.0) < 0.6;
	Text(Atc.awaitingReadback ? TEXT("READBACK REQUIRED  (keys 1-6 or click)") : TEXT("SAY  (keys 1-6 or click)"),
		RX + Pad, Y + 10.0 * Scale, Atc.awaitingReadback && bFlash ? Amber : Grey, 0, 0);
	Y += 22.0 * Scale;
	for (int32 i = 0; i < Atc.optionCount; ++i)
	{
		const FString Option = FString::Printf(TEXT("%d  %s"), i + 1, UTF8_TO_TCHAR(Atc.options[i]));
		Fill(RX + Pad, Y, ReplyW, OptionH[i], ButtonFace);
		Frame(RX + Pad, Y, ReplyW, OptionH[i], FLinearColor(0.3f, 0.3f, 0.33f), 1.0);
		TextWrapped(Option, RX + 2.0 * Pad, Y + 4.0 * Scale, ReplyW - 2.0 * Pad, Cyan, 0);
		Buttons.Add({FBox2D(FVector2D(RX + Pad, Y), FVector2D(RX + Pad + ReplyW, Y + OptionH[i])), EA320Command::AtcReply, i});
		Y += OptionH[i] + 4.0 * Scale;
	}
	if (Atc.optionCount == 0)
	{
		const TCHAR* Why = !St.atcEnabled ? TEXT("ATC is off.")
			: (StationName.IsEmpty() ? TEXT("Nobody on this frequency.") : TEXT("Nothing to say now: listen."));
		Text(Why, RX + 2.0 * Pad, Y + 10.0 * Scale, Grey, 0, 0);
	}

	// Radio log, newest at the bottom, as much as fits.
	const double LogTop = PanelY + PanelH + (Aircraft.HasVoices() ? 24.0 : 42.0) * Scale, LogBottom = RY + RH - Pad - RepliesH - 6.0 * Scale;
	const TArray<A320AtcMessage>& RadioLog = Aircraft.GetRadioLog();
	double Bottom = LogBottom;
	for (int32 i = RadioLog.Num() - 1; i >= 0; --i)
	{
		const A320AtcMessage& M = RadioLog[i];
		const FLinearColor Color = M.speaker == A320_ATC_SPEAKER_PILOT ? Cyan : (M.speaker == A320_ATC_SPEAKER_ATIS ? Grey : White);
		const FString LogLine = FString::Printf(TEXT("%s: %s"), UTF8_TO_TCHAR(M.station), UTF8_TO_TCHAR(M.text));
		const double LineH = TextWrapped(LogLine, 0.0, 0.0, RW - 2.0 * Pad, Color, 0, false);
		if (Bottom - LineH < LogTop)
		{
			break;
		}
		Bottom -= LineH + 3.0 * Scale;
		TextWrapped(LogLine, RX + Pad, Bottom, RW - 2.0 * Pad, Color, 0);
	}
	if (!Aircraft.HasVoices())
	{
		Text(TEXT("No Windows voices installed: ATC as text only."), RX + Pad, LogTop - 14.0 * Scale, Grey, 0, 0);
	}
}

void AA320Hud::DrawAtcSubtitle(const AA320Aircraft& Aircraft)
{
	double Age = 0.0;
	const FString& Subtitle = Aircraft.GetAtcSubtitle(Age);
	const bool bAwaiting = Aircraft.GetSimState().atcAwaitingReadback != 0;
	if (Subtitle.IsEmpty() || (Age > 8.0 && !bAwaiting))
	{
		return;
	}
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	const double TW = FMath::Min(W * 0.5, 900.0 * Scale), Pad = 8.0 * Scale;
	const double TX = (W - TW) / 2.0, TY = H * 0.105;
	const double TextH = TextWrapped(Subtitle, TX + Pad, TY + Pad, TW - 2.0 * Pad, White, 0, false);
	const double BoxH = TextH + 2.0 * Pad + (bAwaiting ? 18.0 * Scale : 0.0);
	Fill(TX, TY, TW, BoxH, FLinearColor(0.0f, 0.0f, 0.0f, 0.6f));
	TextWrapped(Subtitle, TX + Pad, TY + Pad, TW - 2.0 * Pad, White, 0);
	if (bAwaiting)
	{
		Text(TEXT("Readback required: F10 (RADIO)"), TX + Pad, TY + BoxH - 12.0 * Scale, Amber, 0, 0);
	}
}

void AA320Hud::DrawJoystickPanel(const AA320PlayerController& Controller)
{
	using namespace a320::joy;
	const FA320Joystick& Joy = Controller.GetJoystick();
	const bool bButtonsPage = Controller.IsJoystickButtonsPage();
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	// Over the top of the cockpit panel; the pedestal's levers stay visible below it.
	const double PX = W * 0.2, PY = H * 0.025, PW = W * 0.6, PH = H * 0.645;
	Fill(PX, PY, PW, PH, FLinearColor(0.08f, 0.09f, 0.1f, 0.97f));
	Buttons.Add({FBox2D(FVector2D(PX, PY), FVector2D(PX + PW, PY + PH)), EA320Command::None});  // swallows clicks
	Frame(PX, PY, PW, PH, FLinearColor(0.5f, 0.52f, 0.55f), 2.0);
	const double LeftX = PX + 0.02 * W, LineH = 0.022 * H;
	Text(TEXT("JOYSTICK, THROTTLE AND PANEL SETUP"), LeftX, PY + 0.025 * H, White, 1, 0);
	AddButton(PX + PW - 0.04 * W, PY + 0.008 * H, 0.032 * W, 0.034 * H, TEXT("X"), EA320Command::JoystickPanel, false);
	AddButton(PX + PW - 0.12 * W, PY + 0.008 * H, 0.07 * W, 0.034 * H, TEXT("RESCAN"), EA320Command::JoyRescan, false);
	AddButton(PX + PW - 0.27 * W, PY + 0.008 * H, 0.07 * W, 0.034 * H, TEXT("AXES"), EA320Command::JoyPageAxes, !bButtonsPage);
	AddButton(PX + PW - 0.195 * W, PY + 0.008 * H, 0.07 * W, 0.034 * H, TEXT("BUTTONS"), EA320Command::JoyPageButtons, bButtonsPage);
	double RowY = PY + 0.06 * H;
	const TArray<FA320JoystickDevice>& Devices = Joy.GetDevices();

	if (bButtonsPage)
	{
		// Every cockpit command a hardware button, switch position or knob click can drive.
		const int32 Learning = Joy.GetButtonLearning();
		Text(TEXT("Click SET, then press the button, flip the switch or turn the knob one click on your hardware. X clears it."),
			LeftX, RowY, Grey, 0, 0);
		RowY += LineH;
		Text(FString::Printf(TEXT("Last pressed: %s"), Joy.GetLastPressed().IsEmpty() ? TEXT("-") : *Joy.GetLastPressed()), LeftX, RowY, Cyan, 0, 0);
		if (!Joy.GetMessage().IsEmpty())
		{
			Text(Joy.GetMessage(), PX + PW * 0.45, RowY, Amber, 0, 0);
		}
		RowY += LineH * 1.3;
		const std::vector<CommandInfo>& Catalog = commandCatalog();
		const int32 PerColumn = (static_cast<int32>(Catalog.size()) + 2) / 3;
		const double ColW = (PW - 0.04 * W) / 3.0, RowH = 0.0215 * H;
		for (int32 i = 0; i < static_cast<int32>(Catalog.size()); ++i)
		{
			const double CX = LeftX + (i / PerColumn) * ColW, CY = RowY + (i % PerColumn) * RowH;
			const CommandInfo& Info = Catalog[static_cast<size_t>(i)];
			Text(UTF8_TO_TCHAR(Info.label), CX, CY + RowH / 2.0, White, 0, 0);
			FString Bound = Joy.BindText(Info.name);
			if (Bound.Len() > 18)
			{
				Bound = Bound.Left(17) + TEXT("~");
			}
			Text(Bound.IsEmpty() ? FString(TEXT("-")) : Bound, CX + ColW * 0.42, CY + RowH / 2.0, Bound.IsEmpty() ? Grey : Green, 0, 0);
			AddButton(CX + ColW * 0.79, CY + 1.0, ColW * 0.12, RowH - 2.0, Learning == i ? TEXT("PRESS") : TEXT("SET"), EA320Command::JoyBindSet, Learning == i, i);
			AddButton(CX + ColW * 0.92, CY + 1.0, ColW * 0.06, RowH - 2.0, TEXT("X"), EA320Command::JoyBindClear, false, i);
		}
		Text(FString::Printf(TEXT("Saved to %s"), *Joy.GetConfigPath()), LeftX, PY + PH - 0.02 * H, Grey, 0, 0);
		return;
	}

	// Devices, and which one is the stick and the throttle.
	const int32 StickDevice = Joy.GetStickDevice(), ThrottleDevice = Joy.GetThrottleDevice();
	if (Devices.Num() == 0)
	{
		Text(TEXT("No joystick found. Plug it in (it is picked up within 5 s) or press RESCAN."), LeftX, RowY, Amber, 0, 0);
		RowY += LineH;
	}
	const FA320WingFlex& Panels = Controller.GetWingFlex();
	if (Panels.HasFcu() || Panels.HasEfis())
	{
		Text(FString::Printf(TEXT("WingFlex panels: %s%s  (buttons, knobs, lights and displays work directly; close WingFlex Bridge)"),
			Panels.HasFcu() ? TEXT("FCU Cube ") : TEXT(""), Panels.HasEfis() ? TEXT("EFIS Cube") : TEXT("")), LeftX, RowY, Cyan, 0, 0);
		RowY += LineH;
	}
	for (int32 D = 0; D < Devices.Num(); ++D)
	{
		const TCHAR* RoleText = D == StickDevice ? TEXT("   [stick]") : (D == ThrottleDevice ? TEXT("   [throttle]") : TEXT(""));
		Text(FString::Printf(TEXT("Device %d: %s  (%d axes, %d buttons)%s"), D, *Devices[D].Name, Devices[D].NumAxes, Devices[D].NumButtons, RoleText),
			LeftX, RowY, Green, 0, 0);
		RowY += LineH;
	}

	// One row per function: binding (click to cycle), INV, LEARN, CAL for levers with detents,
	// and the live value.
	RowY += 0.008 * H;
	const double ColAxis = PX + 0.12 * W, ColInv = PX + 0.225 * W, ColLearn = PX + 0.28 * W, ColCal = PX + 0.355 * W, ColBar = PX + 0.4 * W;
	const double BtnH = 0.022 * H, BarW = 0.14 * W;
	Text(TEXT("FUNCTION"), LeftX, RowY, Grey, 0, 0);
	Text(TEXT("AXIS (click)"), ColAxis, RowY, Grey, 0, 0);
	Text(TEXT("VALUE"), ColBar, RowY, Grey, 0, 0);
	RowY += 0.025 * H;
	const int32 Order[kFunctionCount] = {kPitch, kRoll, kRudder, kThrottle, kThrottle2, kFlaps, kSpeedbrake, kBrakeLeft, kBrakeRight};
	for (const int32 F : Order)
	{
		const Binding& B = Joy.GetConfig().bind[F];
		const bool bBound = Joy.IsBound(static_cast<a320::joy::Function>(F));
		Text(UTF8_TO_TCHAR(functionName(F)), LeftX, RowY + BtnH / 2.0, White, 0, 0);
		const FString AxisText = bBound ? FString::Printf(TEXT("dev %d  %s"), B.device, UTF8_TO_TCHAR(axisName(B.axis)))
			: (B.deviceName.empty() ? FString(TEXT("none")) : FString(TEXT("not connected")));
		AddButton(ColAxis, RowY, 0.1 * W, BtnH, AxisText, static_cast<EA320Command>(static_cast<int32>(EA320Command::JoyAxis0) + F), bBound);
		AddButton(ColInv, RowY, 0.05 * W, BtnH, TEXT("INV"), static_cast<EA320Command>(static_cast<int32>(EA320Command::JoyInvert0) + F), B.invert);
		const bool bLearning = Joy.GetLearning() == F;
		AddButton(ColLearn, RowY, 0.07 * W, BtnH, bLearning ? TEXT("MOVE IT") : TEXT("LEARN"),
			static_cast<EA320Command>(static_cast<int32>(EA320Command::JoyLearn0) + F), bLearning);
		const EA320Command Cal = F == kThrottle ? EA320Command::JoyCalStart
			: (F == kFlaps ? EA320Command::JoyCalFlaps : (F == kSpeedbrake ? EA320Command::JoyCalSpeedbrake : EA320Command::None));
		if (Cal != EA320Command::None)
		{
			AddButton(ColCal, RowY, 0.04 * W, BtnH, TEXT("CAL"), Cal, false);
		}
		// Live value: -1..1 bar, and what the lever gives in the cockpit.
		Fill(ColBar, RowY + BtnH * 0.3, BarW, BtnH * 0.4, Screen);
		if (bBound)
		{
			const double V = Joy.Value(static_cast<a320::joy::Function>(F));
			const double Mid = ColBar + BarW / 2.0;
			const double End = Mid + FMath::Clamp(V, -1.0, 1.0) * BarW / 2.0;
			Fill(FMath::Min(Mid, End), RowY + BtnH * 0.3, FMath::Max(FMath::Abs(End - Mid), 2.0), BtnH * 0.4, Cyan);
			FString ValueText = FString::Printf(TEXT("%+.2f"), V);
			if (F == kThrottle || F == kThrottle2)
			{
				const a320::joy::LeverPosition L = Joy.Lever(F == kThrottle ? 0 : 1);
				ValueText += L.reverse ? FString::Printf(TEXT("   REV %d%%"), FMath::RoundToInt(L.lever * 100.0))
					: FString::Printf(TEXT("   %s %d%%"), UTF8_TO_TCHAR(leverDetentName(L.lever)), FMath::RoundToInt(L.lever * 100.0));
			}
			else if (F == kFlaps)
			{
				static const TCHAR* FlapNames[5] = {TEXT("0"), TEXT("1"), TEXT("2"), TEXT("3"), TEXT("FULL")};
				ValueText += FString::Printf(TEXT("   FLAPS %s"), FlapNames[Joy.FlapsLever()]);
			}
			else if (F == kSpeedbrake)
			{
				const SpeedbrakePosition P = Joy.Speedbrake();
				ValueText += P.armed ? FString(TEXT("   ARM")) : FString::Printf(TEXT("   %d%%"), FMath::RoundToInt(P.amount * 100.0));
			}
			Text(ValueText, ColBar + BarW + 0.01 * W, RowY + BtnH / 2.0, Cyan, 0, 0);
		}
		else if (F == kThrottle2)
		{
			Text(TEXT("unbound: THRUST 1 moves both levers"), ColBar + BarW + 0.01 * W, RowY + BtnH / 2.0, Grey, 0, 0);
		}
		Line(ColBar + BarW / 2.0, RowY, ColBar + BarW / 2.0, RowY + BtnH, Grey, 1.0);
		RowY += BtnH + 0.004 * H;
	}

	// Detent calibration wizard.
	RowY += 0.01 * H;
	const int32 Target = Joy.GetCalibrationTarget(), Step = Joy.GetCalibrationStep();
	if (Step >= 0)
	{
		const TCHAR* What = Target == kCalFlaps ? TEXT("flaps lever") : (Target == kCalSpeedbrake ? TEXT("speedbrake lever")
			: (Joy.HasSecondLever() ? TEXT("thrust levers") : TEXT("thrust lever")));
		Text(FString::Printf(TEXT("Step %d of %d: put the %s in %s, then press SET."), Step + 1, calStepCount(Target), What,
			UTF8_TO_TCHAR(calStepLabel(Target, Step))), LeftX, RowY + BtnH / 2.0, Amber, 0, 0);
		double BX = PX + 0.4 * W;
		AddButton(BX, RowY, 0.05 * W, BtnH, TEXT("SET"), EA320Command::JoyCalSet, true);
		BX += 0.055 * W;
		if (calStepOptional(Target, Step))
		{
			AddButton(BX, RowY, 0.09 * W, BtnH, Target == kCalSpeedbrake ? TEXT("NO ARM") : TEXT("NO REVERSE"), EA320Command::JoyCalSkip, false);
			BX += 0.095 * W;
		}
		AddButton(BX, RowY, 0.06 * W, BtnH, TEXT("CANCEL"), EA320Command::JoyCalCancel, false);
	}
	else
	{
		Text(TEXT("CAL teaches the game where your levers' detents are (thrust: IDLE CL FLX TOGA REV; flaps: 0-FULL; speedbrake: RET FULL ARM)."),
			LeftX, RowY + BtnH / 2.0, Grey, 0, 0);
	}
	RowY += BtnH + 0.006 * H;
	if (!Joy.GetMessage().IsEmpty())
	{
		Text(Joy.GetMessage(), LeftX, RowY + LineH / 2.0, Amber, 0, 0);
	}
	RowY += LineH;
	Text(TEXT("Buttons, switches and FCU/EFIS knobs: BUTTONS page. Hat switch looks around."), LeftX, RowY + LineH / 2.0, Grey, 0, 0);
	Text(TEXT("Pitch: pull back = +. Thrust: forward = +1 (TOGA). Use INV if a bar moves the wrong way."),
		LeftX, PY + PH - 0.02 * H, FLinearColor(0.8f, 0.85f, 0.9f), 0, 0);
}

void AA320Hud::DrawLoadingStatus(const AA320Aircraft* Aircraft)
{
	int32 Shaders = 0;
	int32 Assets = 0;
#if WITH_EDITOR
	// Only uncooked runs (Play.bat, the editor) compile on the fly; packaged builds never do.
	if (GShaderCompilingManager)
	{
		Shaders = GShaderCompilingManager->GetNumRemainingJobs();
	}
	Assets = FAssetCompilingManager::Get().GetNumRemainingAssets();
#endif
	const int32 Pending = Shaders + Assets;
	const double Now = GetWorld()->GetRealTimeSeconds();
	MaxPending = FMath::Max(MaxPending, Pending);

	if (Now >= NextStatusLog && Pending > 0)
	{
		NextStatusLog = Now + 2.0;
		UE_LOG(LogA320, Log, TEXT("Preparing graphics: %d shaders, %d assets remaining (of %d)"), Shaders, Assets, MaxPending);
	}
	// Ready once nothing has been compiling for a moment and the aircraft exists.
	if (!bReadyLogged)
	{
		if (Pending > 0 || !Aircraft)
		{
			QuietSince = -1.0;
		}
		else if (QuietSince < 0.0)
		{
			QuietSince = Now;
		}
		else if (Now - QuietSince > 1.5)
		{
			bReadyLogged = true;
			UE_LOG(LogA320, Log, TEXT("READY"));
		}
	}

	const double W = Canvas->ClipX, H = Canvas->ClipY;
	Scale = FMath::Max(H / 1080.0, 0.6);
	if (Pending == 0 && Aircraft)
	{
		return;
	}
	if (bReadyLogged || (Aircraft && MaxPending < 100))
	{
		// Small on-demand compiles (later starts, new views): just a quiet note.
		Text(FString::Printf(TEXT("Compiling shaders: %d"), Pending), 20.0 * Scale, H * 0.55, Grey, 0, 0);
		return;
	}

	// First start: a progress panel over the (still incomplete) view.
	const double BW = 760.0 * Scale, BH = 230.0 * Scale;
	const double BX = (W - BW) / 2.0, BY = H * 0.18;
	Fill(BX, BY, BW, BH, FLinearColor(0.02f, 0.03f, 0.05f, 0.92f));
	Frame(BX, BY, BW, BH, FLinearColor(0.4f, 0.45f, 0.5f), 2.0);
	Text(TEXT("A320 SIM  -  preparing graphics"), BX + BW / 2.0, BY + 32.0 * Scale, White, 2, 1);
	FString Detail;
	if (!Aircraft)
	{
		Detail = TEXT("Loading the flight model and the airport...");
	}
	else
	{
		Detail = FString::Printf(TEXT("Compiling shaders: %d remaining"), Shaders);
		if (Assets > 0)
		{
			Detail += FString::Printf(TEXT(",  building assets: %d"), Assets);
		}
	}
	Text(Detail, BX + BW / 2.0, BY + 82.0 * Scale, Amber, 1, 1);
	const double Progress = MaxPending > 0 ? 1.0 - static_cast<double>(Pending) / MaxPending : 0.0;
	const double BarX = BX + 40.0 * Scale, BarY = BY + 112.0 * Scale, BarW = BW - 80.0 * Scale, BarH = 18.0 * Scale;
	Fill(BarX, BarY, BarW, BarH, FLinearColor(0.12f, 0.13f, 0.15f));
	Fill(BarX, BarY, BarW * FMath::Clamp(Progress, 0.0, 1.0), BarH, Cyan);
	const int32 Elapsed = static_cast<int32>(Now);
	Text(FString::Printf(TEXT("%d%%   -   %d:%02d elapsed"), static_cast<int32>(Progress * 100.0), Elapsed / 60, Elapsed % 60),
		BX + BW / 2.0, BarY + 36.0 * Scale, White, 0, 1);
	Text(TEXT("This happens once (results are cached); the next start takes seconds."), BX + BW / 2.0, BarY + 64.0 * Scale, Grey, 0, 1);
	Text(TEXT("The aircraft is parked with the brakes set, so it is safe to wait."), BX + BW / 2.0, BarY + 86.0 * Scale, Grey, 0, 1);
}

namespace
{
	// The guide target a cockpit control stands for.
	int32 GuideTargetFor(EA320Command Command)
	{
		switch (Command)
		{
		case EA320Command::FcuAp: return A320_GT_FCU_AP1;
		case EA320Command::FcuAp2: return A320_GT_FCU_AP2;
		case EA320Command::FcuAthr: return A320_GT_FCU_ATHR;
		case EA320Command::FcuLoc: return A320_GT_FCU_LOC;
		case EA320Command::FcuAppr: return A320_GT_FCU_APPR;
		case EA320Command::LsToggle: return A320_GT_EFIS_LS;
		case EA320Command::NdModeToggle: return A320_GT_EFIS_ND_MODE;
		case EA320Command::GearToggle: return A320_GT_GEAR;
		case EA320Command::ParkBrakeToggle: return A320_GT_PARK_BRAKE;
		case EA320Command::SpoilerArm: return A320_GT_SPOILERS;
		case EA320Command::AutobrakeLo:
		case EA320Command::AutobrakeMed:
		case EA320Command::AutobrakeMax: return A320_GT_AUTOBRAKE;
		default: return A320_GT_NONE;
		}
	}
}

void AA320Hud::MarkTarget(int32 Target, double X, double Y, double W, double H)
{
	const FBox2D Box(FVector2D(X, Y), FVector2D(X + W, Y + H));
	if (FBox2D* Existing = TargetBoxes.Find(Target))
	{
		*Existing += Box;
	}
	else
	{
		TargetBoxes.Add(Target, Box);
	}
}

double AA320Hud::TextWrapped(const FString& Str, double X, double Y, double MaxW, const FLinearColor& Color, int32 Size, bool bDraw)
{
	UFont* Font = FontFor(Size);
	float SampleW = 0.0f, SampleH = 0.0f;
	GetTextSize(TEXT("Ag"), SampleW, SampleH, Font, (float)Scale);
	const double Step = SampleH * 1.12;
	TArray<FString> Words;
	Str.ParseIntoArray(Words, TEXT(" "), true);
	FString Current;
	double LineTop = Y;
	auto Flush = [&]()
	{
		if (!Current.IsEmpty())
		{
			if (bDraw)
			{
				Text(Current, X, LineTop + Step / 2.0, Color, Size, 0);
			}
			LineTop += Step;
			Current.Reset();
		}
	};
	for (const FString& Word : Words)
	{
		const FString Candidate = Current.IsEmpty() ? Word : Current + TEXT(" ") + Word;
		float CandidateW = 0.0f, CandidateH = 0.0f;
		GetTextSize(Candidate, CandidateW, CandidateH, Font, (float)Scale);
		if (CandidateW > MaxW && !Current.IsEmpty())
		{
			Flush();
			Current = Word;
		}
		else
		{
			Current = Candidate;
		}
	}
	Flush();
	return LineTop - Y;
}

void AA320Hud::DrawGuideMenu()
{
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	Scale = FMath::Max(H / 1080.0, 0.6);
	const int32 Count = FMath::Min(a320_guide_count(), 4);
	const double PW = FMath::Min(W * 0.5, 900.0 * Scale), PX = (W - PW) / 2.0, PY = H * 0.1;
	const double Pad = 16.0 * Scale, RowH = 120.0 * Scale;
	const double PH = 90.0 * Scale + Count * RowH;
	Fill(PX, PY, PW, PH, FLinearColor(0.06f, 0.07f, 0.08f, 0.97f));
	Buttons.Add({FBox2D(FVector2D(PX, PY), FVector2D(PX + PW, PY + PH)), EA320Command::None});  // swallows clicks
	Frame(PX, PY, PW, PH, FLinearColor(0.5f, 0.52f, 0.55f), 2.0);
	Text(TEXT("LESSONS"), PX + Pad, PY + 24.0 * Scale, White, 1, 0);
	AddButton(PX + PW - 44.0 * Scale, PY + 8.0 * Scale, 36.0 * Scale, 30.0 * Scale, TEXT("X"), EA320Command::GuideMenu, false);
	Text(TEXT("A lesson resets the aircraft, then guides you step by step and ticks each step off when you have done it."),
		PX + Pad, PY + 56.0 * Scale, Grey, 0, 0);
	for (int32 i = 0; i < Count; ++i)
	{
		const double RY = PY + 80.0 * Scale + i * RowH;
		Line(PX + Pad, RY, PX + PW - Pad, RY, FLinearColor(0.25f, 0.27f, 0.3f), 1.0);
		Text(UTF8_TO_TCHAR(a320_guide_name(i)), PX + Pad, RY + 22.0 * Scale, Cyan, 1, 0);
		Text(FString::Printf(TEXT("%d steps"), a320_guide_step_count(i)), PX + PW - 150.0 * Scale, RY + 22.0 * Scale, Grey, 0, 2);
		TextWrapped(UTF8_TO_TCHAR(a320_guide_summary(i)), PX + Pad, RY + 40.0 * Scale, PW - 190.0 * Scale, White, 0);
		AddButton(PX + PW - 130.0 * Scale, RY + 10.0 * Scale, 110.0 * Scale, 40.0 * Scale, TEXT("START"),
			static_cast<EA320Command>(static_cast<int32>(EA320Command::GuideStart0) + i), true);
	}
}

void AA320Hud::DrawGuide(const AA320Aircraft& Aircraft)
{
	const A320GuideStatus G = Aircraft.GetGuideStatus();
	if (!G.active)
	{
		return;
	}
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	const double Now = GetWorld()->GetRealTimeSeconds();
	Scale = FMath::Max(H / 1080.0, 0.6);

	// Where this frame drew the controls the steps point at.
	for (const FButton& B : Buttons)
	{
		const int32 Target = GuideTargetFor(B.Command);
		if (Target != A320_GT_NONE)
		{
			MarkTarget(Target, B.Box.Min.X, B.Box.Min.Y, B.Box.Max.X - B.Box.Min.X, B.Box.Max.Y - B.Box.Min.Y);
		}
	}
	for (const FLeverSlot& Slot : Levers)
	{
		const int32 Target = Slot.Lever == EA320Lever::Thrust ? A320_GT_THRUST_LEVERS : (Slot.Lever == EA320Lever::Flaps ? A320_GT_FLAPS : A320_GT_NONE);
		if (Target != A320_GT_NONE)
		{
			MarkTarget(Target, Slot.Box.Min.X, Slot.Box.Min.Y, Slot.Box.Max.X - Slot.Box.Min.X, Slot.Box.Max.Y - Slot.Box.Min.Y);
		}
	}

	// The panel: left, above the cockpit panel. Laid out twice: measure, then draw.
	const double PX = 12.0 * Scale, PY = 76.0 * Scale, PW = FMath::Min(W * 0.34, 680.0 * Scale);
	const double Pad = 12.0 * Scale, TW = PW - 2.0 * Pad, Gap = 6.0 * Scale;
	const FString Alert = Aircraft.GetGuideAlert();
	const int32 Guide = G.guide, Step = G.step;
	auto StepText = [Guide](int32 Index, A320GuideText Field) { return FString(UTF8_TO_TCHAR(a320_guide_step_text(Guide, Index, Field))); };
	struct FSection
	{
		const TCHAR* Label;
		FString Body;
		FLinearColor Color;
	};
	TArray<FSection> Sections;
	if (G.complete)
	{
		Sections.Add({TEXT("LANDED - LESSON COMPLETE"), TEXT("You flew an ILS approach and a CAT 3 autoland, then stopped the aircraft yourself. The same buttons and the same FMA work in the MSFS 2024 A320neo."), Green});
		if (!TouchdownText.IsEmpty())
		{
			Sections.Add({TEXT("YOUR LANDING"), TouchdownText, Cyan});
		}
	}
	else
	{
		Sections.Add({TEXT("DO"), StepText(Step, A320_GUIDE_ACTION), White});
		Sections.Add({TEXT("LOOK FOR"), StepText(Step, A320_GUIDE_LOOK), Green});
		Sections.Add({TEXT("HOW IT WORKS"), StepText(Step, A320_GUIDE_WHY), FLinearColor(0.75f, 0.78f, 0.82f)});
		Sections.Add({TEXT("IN MSFS 2024"), StepText(Step, A320_GUIDE_MSFS), Cyan});
	}
	auto Layout = [&](bool bDraw)
	{
		double CY = PY + Pad;
		if (bDraw)
		{
			Text(UTF8_TO_TCHAR(a320_guide_name(Guide)), PX + Pad, CY + 8.0 * Scale, Cyan, 0, 0);
			Text(G.complete ? FString(TEXT("done")) : FString::Printf(TEXT("step %d of %d"), Step + 1, G.stepCount),
				PX + PW - 52.0 * Scale, CY + 8.0 * Scale, Grey, 0, 2);
			AddButton(PX + PW - 44.0 * Scale, CY - 4.0 * Scale, 34.0 * Scale, 24.0 * Scale, TEXT("X"), EA320Command::GuideStop, false);
		}
		CY += 26.0 * Scale;
		// Progress: one segment per step.
		if (bDraw && G.stepCount > 0)
		{
			const double SegW = TW / G.stepCount;
			for (int32 i = 0; i < G.stepCount; ++i)
			{
				const bool bDone = ((G.doneMask >> i) & 1u) != 0;
				const FLinearColor C = i == Step ? Yellow : (bDone ? Green : (i < Step ? Amber : FLinearColor(0.25f, 0.27f, 0.3f)));
				Fill(PX + Pad + i * SegW + 1.0, CY, SegW - 2.0, 6.0 * Scale, C);
			}
		}
		CY += 14.0 * Scale;
		if (!G.complete)
		{
			if (bDraw)
			{
				Text(StepText(Step, A320_GUIDE_PHASE), PX + Pad, CY + 8.0 * Scale, Grey, 0, 0);
			}
			CY += 20.0 * Scale;
			CY += TextWrapped(StepText(Step, A320_GUIDE_TITLE), PX + Pad, CY, TW, Yellow, 1, bDraw) + Gap;
		}
		for (const FSection& Sec : Sections)
		{
			if (bDraw)
			{
				Text(Sec.Label, PX + Pad, CY + 8.0 * Scale, Grey, 0, 0);
			}
			CY += 18.0 * Scale;
			CY += TextWrapped(Sec.Body, PX + Pad, CY, TW, Sec.Color, 0, bDraw) + Gap;
		}
		if (!Alert.IsEmpty() && !G.complete)
		{
			const double AlertH = TextWrapped(Alert, PX + 2.0 * Pad, CY + Gap, TW - 2.0 * Pad, Amber, 0, false) + 2.0 * Gap;
			if (bDraw)
			{
				Fill(PX + Pad, CY, TW, AlertH, FLinearColor(0.25f, 0.15f, 0.0f, 0.9f));
				Frame(PX + Pad, CY, TW, AlertH, Amber, 2.0);
				TextWrapped(Alert, PX + 2.0 * Pad, CY + Gap, TW - 2.0 * Pad, Amber, 0, true);
			}
			CY += AlertH + Gap;
		}
		// Buttons: a check step is confirmed with NEXT; any other step ticks itself off.
		const double BH = 30.0 * Scale, BW = 110.0 * Scale;
		if (bDraw)
		{
			if (G.complete)
			{
				AddButton(PX + Pad, CY, BW * 1.3, BH, TEXT("FLY IT AGAIN"), static_cast<EA320Command>(static_cast<int32>(EA320Command::GuideStart0) + Guide), true);
				AddButton(PX + Pad + BW * 1.3 + Gap, CY, BW, BH, TEXT("CLOSE"), EA320Command::GuideStop, false);
			}
			else
			{
				AddButton(PX + Pad, CY, BW * 0.7, BH, TEXT("BACK"), EA320Command::GuideBack, false);
				AddButton(PX + Pad + BW * 0.7 + Gap, CY, BW * 1.2, BH, G.manual ? TEXT("CHECKED: NEXT") : TEXT("SKIP"), EA320Command::GuideNext, G.manual != 0);
				Text(G.manual ? TEXT("Check it, then press NEXT.") : TEXT("Ticks itself off when done."),
					PX + Pad + BW * 1.9 + 2.0 * Gap, CY + BH / 2.0, Grey, 0, 0);
			}
		}
		CY += BH + Gap;
		if (!G.complete && Step + 1 < G.stepCount)
		{
			if (bDraw)
			{
				Text(FString::Printf(TEXT("Next: %s"), *StepText(Step + 1, A320_GUIDE_TITLE)), PX + Pad, CY + 8.0 * Scale, Grey, 0, 0);
			}
			CY += 20.0 * Scale;
		}
		return CY + Pad - PY;
	};
	// Must stay clear of the FCU (at 58 % of the height): drop the MSFS note, then the explanation.
	double PH = Layout(false);
	while (PH > H * 0.58 - PY - 6.0 * Scale && Sections.Num() > 2)
	{
		Sections.Pop();
		PH = Layout(false);
	}
	Fill(PX, PY, PW, PH, FLinearColor(0.03f, 0.04f, 0.05f, 0.9f));
	Buttons.Add({FBox2D(FVector2D(PX, PY), FVector2D(PX + PW, PY + PH)), EA320Command::None});  // swallows clicks
	Frame(PX, PY, PW, PH, Yellow, 2.0);
	Layout(true);

	// Highlight the controls and displays of the current step, with a line from the panel.
	if (G.complete)
	{
		return;
	}
	const double Pulse = 0.5 + 0.5 * FMath::Sin(Now * 6.0);
	for (int32 i = 0; i < A320_GUIDE_MAX_TARGETS; ++i)
	{
		const FBox2D* Box = TargetBoxes.Find(G.targets[i]);
		if (G.targets[i] == A320_GT_NONE || !Box)
		{
			continue;
		}
		const double Grow = (4.0 + 3.0 * Pulse) * Scale;
		const FVector2D Min = Box->Min - FVector2D(Grow, Grow), Max = Box->Max + FVector2D(Grow, Grow);
		Frame(Min.X, Min.Y, Max.X - Min.X, Max.Y - Min.Y, Yellow, 2.0 + 2.0 * Pulse);
		const FVector2D Centre = (Min + Max) / 2.0;
		const FVector2D Edge(FMath::Clamp(Centre.X, Min.X, Max.X), Min.Y);
		Line(PX + PW, PY + 40.0 * Scale, Edge.X, Edge.Y, FLinearColor(1.0f, 0.85f, 0.1f, 0.8f), 2.0);
	}
}
