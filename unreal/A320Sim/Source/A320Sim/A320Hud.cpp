#include "A320Hud.h"

#include "A320Aircraft.h"
#include "A320PlayerController.h"
#include "CanvasItem.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "RenderUtils.h"
#include "a320/Geometry2D.h"

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
	const APlayerController* PC = GetOwningPlayerController();
	const AA320Aircraft* Aircraft = PC ? Cast<AA320Aircraft>(PC->GetPawn()) : nullptr;
	if (!Canvas || !Aircraft)
	{
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
		X += S + Margin;
		DrawNd(*Aircraft, X, Y, S);
		X += S + Margin;
		DrawEwd(*Aircraft, X, Y, S);
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
		const TArray<A320RunwayInfo>& Runways = Aircraft.GetRunways();
		if (Runways.IsValidIndex(St.ilsRunwayIndex))
		{
			const A320RunwayInfo& Rw = Runways[St.ilsRunwayIndex];
			Text(FString::Printf(TEXT("ILS %s"), UTF8_TO_TCHAR(Rw.ident)), X + 0.02 * S, Y + 0.84 * S, Magenta, 0, 0);
			Text(FString::Printf(TEXT("%03d"), FMath::RoundToInt(Wrap360(Rw.trueCourseDeg - MagVar))), X + 0.02 * S, Y + 0.89 * S, Magenta, 0, 0);
			Text(FString::Printf(TEXT("%.1fNM"), St.dmeNm), X + 0.02 * S, Y + 0.94 * S, Magenta, 0, 0);
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

	// Runways, and the active ILS's extended centreline out to 12 NM.
	const TArray<A320RunwayInfo>& Runways = Aircraft.GetRunways();
	for (int32 i = 0; i < Runways.Num(); ++i)
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
	for (int32 D = FMath::CeilToInt((MagHdg - ArcHalf) / 5.0) * 5; D <= MagHdg + ArcHalf - (bRose ? 1.0 : 0.0); D += 5)
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
	// Own aircraft.
	Line(AcX, AcY - 0.04 * S, AcX, AcY + 0.04 * S, Yellow, 3.0);
	Line(AcX - 0.04 * S, AcY - 0.01 * S, AcX + 0.04 * S, AcY - 0.01 * S, Yellow, 3.0);
	Line(AcX - 0.015 * S, AcY + 0.035 * S, AcX + 0.015 * S, AcY + 0.035 * S, Yellow, 3.0);

	Text(FString::Printf(TEXT("GS %d  TAS %d"), FMath::RoundToInt(St.groundSpeedKt), FMath::RoundToInt(St.tasKt)), X + 0.03 * S, Y + 0.04 * S, White, 0, 0);
	if (Aircraft.IsLsOn() && Runways.IsValidIndex(St.ilsRunwayIndex))
	{
		Text(FString::Printf(TEXT("ILS %s  %.1f NM"), UTF8_TO_TCHAR(Runways[St.ilsRunwayIndex].ident), St.dmeNm), X + S - 0.03 * S, Y + 0.04 * S, Magenta, 0, 2);
	}
	Text(FString::Printf(TEXT("%s  %d NM"), bRose ? TEXT("ROSE") : TEXT("ARC"), Aircraft.GetNdRangeNm()), X + 0.03 * S, Y + 0.96 * S, Cyan, 0, 0);
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
		const FVector2D Lever = Polar(DX, DY, DR + 0.012 * S, Angle(St.thrustLever * 101.0));
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
		Text(FString::Printf(TEXT("SIM RATE x%d"), FMath::RoundToInt(St.simRate)), W - 20.0 * Scale, 30.0 * Scale, Yellow, 1, 2);
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
		TEXT("Scenarios      F5 lined up 26,  Shift+F5 cold and dark,  F6 10 NM final,  F7 4 NM final,  F9 swap runway"),
		TEXT("Cockpit        drag the thrust, flaps and speedbrake levers;  click switches;  O overhead panel"),
		TEXT("Joystick       F2 (or JOYSTICK, top right): pick axes with LEARN; trigger = AP disconnect, hat = look"),
		TEXT("Autopilot      A AP1,  T A/THR (thrust levers in CL: Ins),  K APPR (autoland),  J LOC"),
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

void AA320Hud::AddButton(double X, double Y, double W, double H, const FString& Label, EA320Command Command, bool bLit)
{
	Fill(X, Y, W, H, ButtonFace);
	Frame(X, Y, W, H, bLit ? Green : FLinearColor(0.3f, 0.3f, 0.33f), bLit ? 2.0 : 1.0);
	if (bLit)
	{
		Fill(X + W * 0.25, Y + H - 4.0 * Scale, W * 0.5, 2.5 * Scale, Green);
	}
	Text(Label, X + W / 2.0, Y + H / 2.0, bLit ? Green : White, 0, 1);
	Buttons.Add({FBox2D(FVector2D(X, Y), FVector2D(X + W, Y + H)), Command});
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
	const double WindowW = W * 0.125, ButtonW = W * 0.055;
	double CX = X + Gap;
	// Captain's EFIS control panel: LS, ND mode and range.
	const double EfisW = W * 0.045;
	AddButton(CX, Y + H * 0.15, EfisW, H * 0.75, TEXT("LS"), EA320Command::LsToggle, Aircraft.IsLsOn());
	CX += EfisW + Gap;
	AddButton(CX, Y + H * 0.15, EfisW * 1.2, H * 0.75, Aircraft.IsNdRose() ? TEXT("ROSE") : TEXT("ARC"), EA320Command::NdModeToggle, false);
	CX += EfisW * 1.2 + Gap;
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
		CX += WindowW + Gap;
	};
	auto DrawButton = [&](const TCHAR* Label, EA320Command Command, bool bLit)
	{
		AddButton(CX, Y + H * 0.15, ButtonW, H * 0.75, Label, Command, bLit);
		CX += ButtonW + Gap;
	};
	const bool bLocLit = St.latMode == A320_LAT_LOC || St.latMode == A320_LAT_LOC_STAR || (St.armed & A320_ARMED_LOC);
	const bool bApprLit = (St.armed & A320_ARMED_GS) || St.vertMode == A320_VERT_GS || St.vertMode == A320_VERT_LAND || St.vertMode == A320_VERT_FLARE;
	DrawWindow(FcuWindows[0]);
	DrawWindow(FcuWindows[1]);
	DrawButton(TEXT("LOC"), EA320Command::FcuLoc, bLocLit && !bApprLit);
	DrawButton(TEXT("AP1"), EA320Command::FcuAp, St.apEngaged != 0);
	DrawButton(TEXT("A/THR"), EA320Command::FcuAthr, St.athrEngaged != 0);
	DrawWindow(FcuWindows[2]);
	DrawButton(TEXT("APPR"), EA320Command::FcuAppr, bApprLit);
	DrawWindow(FcuWindows[3]);
}

void AA320Hud::DrawFma(const A320State& St, double X, double Y, double S)
{
	for (int32 i = 1; i < 5; ++i)
	{
		Line(X + 0.2 * S * i, Y + 0.01 * S, X + 0.2 * S * i, Y + 0.11 * S, Grey, 1.0);
	}
	const double Row1 = Y + 0.03 * S, Row2 = Y + 0.075 * S;
	// Column 1: thrust.
	FString Thrust;
	FLinearColor ThrustColor = Green;
	if (St.reverse) Thrust = TEXT("REV");
	else if (St.athrEngaged && St.athrActive) Thrust = UTF8_TO_TCHAR(a320_athr_mode_name(St.athrMode));
	else if (St.thrustDetent == 3) { Thrust = TEXT("MAN TOGA"); ThrustColor = White; }
	else if (St.thrustDetent == 2) { Thrust = St.onGround ? TEXT("MAN FLX") : TEXT("MAN MCT"); ThrustColor = White; }
	else if (St.athrEngaged && !St.onGround && FMath::Fmod(GetWorld()->GetRealTimeSeconds(), 1.0) < 0.6) { Thrust = TEXT("LVR CLB"); ThrustColor = White; }
	Text(Thrust, X + 0.1 * S, Row1, ThrustColor, 0, 1);

	// Columns 2-3: vertical and lateral (LAND, FLARE and ROLL OUT span both), armed modes in cyan.
	const bool bCombined = St.vertMode == A320_VERT_LAND || St.vertMode == A320_VERT_FLARE || St.latMode == A320_LAT_ROLLOUT;
	if (bCombined)
	{
		Text(St.latMode == A320_LAT_ROLLOUT ? TEXT("ROLL OUT") : UTF8_TO_TCHAR(a320_vert_mode_name(St.vertMode)), X + 0.4 * S, Row1, Green, 0, 1);
	}
	else
	{
		Text(UTF8_TO_TCHAR(a320_vert_mode_name(St.vertMode)), X + 0.3 * S, Row1, Green, 0, 1);
		Text(UTF8_TO_TCHAR(a320_lat_mode_name(St.latMode)), X + 0.5 * S, Row1, Green, 0, 1);
		FString ArmedV;
		if (St.armed & A320_ARMED_GS) ArmedV += TEXT("G/S ");
		else if (St.armed & A320_ARMED_ALT) ArmedV += TEXT("ALT");
		Text(ArmedV, X + 0.3 * S, Row2, Cyan, 0, 1);
		Text((St.armed & A320_ARMED_LOC) ? TEXT("LOC") : TEXT(""), X + 0.5 * S, Row2, Cyan, 0, 1);
	}
	// Column 5: engagement status.
	Text(St.apEngaged ? TEXT("AP1") : TEXT(""), X + 0.9 * S, Row1, White, 0, 1);
	if (St.athrEngaged)
	{
		Text(TEXT("A/THR"), X + 0.9 * S, Row2, St.athrActive ? White : Cyan, 0, 1);
	}
}

EA320Lever AA320Hud::LeverAt(const FVector2D& ScreenPos) const
{
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
		Pushbutton(SX, TopY + SlotH + 0.06 * H, SW, 0.1 * H, TEXT(""), TEXT(""), Green, Ctl.spoilersArmed ? TEXT("ARMED") : TEXT("ARM"),
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
		const double Pos = St.reverse ? ForwardSpan + St.thrustLever * (1.0 - ForwardSpan) : (1.0 - St.thrustLever) * ForwardSpan;
		const double HY = TopY + Pos * SlotH;
		for (const double Side : {0.33, 0.67})
		{
			Fill(TX + TW * (Side - 0.13), HY - 0.03 * H, TW * 0.26, 0.06 * H, Handle);
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
		{TEXT("JOYSTICK"), EA320Command::JoystickPanel, false},
		{Aircraft.IsSoundOn() ? TEXT("SOUND ON") : TEXT("SOUND OFF"), EA320Command::SoundToggle, !Aircraft.IsSoundOn()},
		{TEXT("HELP"), EA320Command::HelpToggle, Aircraft.IsHelpVisible()},
		{TEXT("SWAP RWY"), EA320Command::RunwaySwap, false},
		{TEXT("LINE UP"), EA320Command::ResetRunway, false},
		{TEXT("COLD+DARK"), EA320Command::ResetColdDark, false},
		{TEXT("FINAL 10"), EA320Command::ResetFinal10, false},
		{TEXT("FINAL 4"), EA320Command::ResetFinal4, false},
	};
	const int32 Count = UE_ARRAY_COUNT(Items);
	const double BW = FMath::Min(W * 0.058, 120.0 * Scale), BH = 30.0 * Scale, Gap = 4.0 * Scale;
	double BX = W - Count * (BW + Gap) - 8.0 * Scale;
	for (const FItem& Item : Items)
	{
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

void AA320Hud::DrawJoystickPanel(const AA320PlayerController& Controller)
{
	using namespace a320::joy;
	const FA320Joystick& Joy = Controller.GetJoystick();
	const double W = Canvas->ClipX, H = Canvas->ClipY;
	// Above the cockpit panel (which starts at 58 % of the height), like the overhead panel.
	const double PX = W * 0.2, PY = H * 0.055, PW = W * 0.6, PH = H * 0.52;
	Fill(PX, PY, PW, PH, FLinearColor(0.08f, 0.09f, 0.1f, 0.97f));
	Frame(PX, PY, PW, PH, FLinearColor(0.5f, 0.52f, 0.55f), 2.0);
	Text(TEXT("JOYSTICK SETUP"), PX + PW / 2.0, PY + 0.025 * H, White, 1, 1);
	AddButton(PX + PW - 0.04 * W, PY + 0.008 * H, 0.032 * W, 0.034 * H, TEXT("X"), EA320Command::JoystickPanel, false);
	AddButton(PX + PW - 0.12 * W, PY + 0.008 * H, 0.07 * W, 0.034 * H, TEXT("RESCAN"), EA320Command::JoyRescan, false);

	// Devices.
	double RowY = PY + 0.07 * H;
	const TArray<FA320JoystickDevice>& Devices = Joy.GetDevices();
	if (Devices.Num() == 0)
	{
		Text(TEXT("No joystick found. Plug it in (it is picked up within 5 s) or press RESCAN."), PX + 0.02 * W, RowY, Amber, 0, 0);
		RowY += 0.035 * H;
	}
	for (int32 D = 0; D < Devices.Num(); ++D)
	{
		Text(FString::Printf(TEXT("Device %d: %s  (%d axes, %d buttons)"), D, *Devices[D].Name, Devices[D].NumAxes, Devices[D].NumButtons),
			PX + 0.02 * W, RowY, Green, 0, 0);
		RowY += 0.03 * H;
	}

	// One row per function: binding (click to cycle), INV, LEARN and the live value.
	RowY += 0.02 * H;
	const double ColAxis = PX + 0.14 * W, ColInv = PX + 0.25 * W, ColLearn = PX + 0.31 * W, ColBar = PX + 0.4 * W;
	const double BtnH = 0.028 * H, BarW = 0.17 * W;
	Text(TEXT("FUNCTION"), PX + 0.02 * W, RowY, Grey, 0, 0);
	Text(TEXT("AXIS (click)"), ColAxis, RowY, Grey, 0, 0);
	Text(TEXT("VALUE"), ColBar, RowY, Grey, 0, 0);
	RowY += 0.035 * H;
	for (int32 F = 0; F < kFunctionCount; ++F)
	{
		const Binding& B = Joy.GetConfig().bind[F];
		const bool bBound = Joy.IsBound(static_cast<Function>(F));
		Text(UTF8_TO_TCHAR(functionName(F)), PX + 0.02 * W, RowY + BtnH / 2.0, White, 0, 0);
		const FString AxisText = B.device < 0 ? FString(TEXT("none")) : FString::Printf(TEXT("dev %d  %s"), B.device, UTF8_TO_TCHAR(axisName(B.axis)));
		AddButton(ColAxis, RowY, 0.1 * W, BtnH, AxisText, static_cast<EA320Command>(static_cast<int32>(EA320Command::JoyAxis0) + F), bBound);
		AddButton(ColInv, RowY, 0.05 * W, BtnH, TEXT("INV"), static_cast<EA320Command>(static_cast<int32>(EA320Command::JoyInvert0) + F), B.invert);
		const bool bLearning = Joy.GetLearning() == F;
		AddButton(ColLearn, RowY, 0.08 * W, BtnH, bLearning ? TEXT("MOVE IT...") : TEXT("LEARN"),
			static_cast<EA320Command>(static_cast<int32>(EA320Command::JoyLearn0) + F), bLearning);
		// Live value: -1..1 bar (throttle and brakes shown the same way).
		Fill(ColBar, RowY + BtnH * 0.3, BarW, BtnH * 0.4, Screen);
		if (bBound)
		{
			const double V = Joy.Value(static_cast<Function>(F));
			const double Mid = ColBar + BarW / 2.0;
			const double End = Mid + FMath::Clamp(V, -1.0, 1.0) * BarW / 2.0;
			Fill(FMath::Min(Mid, End), RowY + BtnH * 0.3, FMath::Max(FMath::Abs(End - Mid), 2.0), BtnH * 0.4, Cyan);
			Text(FString::Printf(TEXT("%+.2f"), V), ColBar + BarW + 0.01 * W, RowY + BtnH / 2.0, Cyan, 0, 0);
		}
		Line(ColBar + BarW / 2.0, RowY, ColBar + BarW / 2.0, RowY + BtnH, Grey, 1.0);
		RowY += BtnH + 0.008 * H;
	}

	// Buttons currently pressed and what they do.
	RowY += 0.01 * H;
	FString Pressed;
	for (int32 Btn = 0; Btn < kButtons; ++Btn)
	{
		if (Joy.IsButtonDown(Btn))
		{
			const FString Cmd = Joy.ButtonCommand(Btn);
			Pressed += FString::Printf(TEXT("%d%s  "), Btn + 1, Cmd.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(" (%s)"), *Cmd));
		}
	}
	Text(FString::Printf(TEXT("Buttons pressed: %s"), Pressed.IsEmpty() ? TEXT("-") : *Pressed), PX + 0.02 * W, RowY, White, 0, 0);
	RowY += 0.03 * H;
	FString Mapping;
	for (int32 Btn = 0; Btn < 8; ++Btn)
	{
		const FString Cmd = Joy.ButtonCommand(Btn);
		if (!Cmd.IsEmpty())
		{
			Mapping += FString::Printf(TEXT("%d=%s  "), Btn + 1, *Cmd);
		}
	}
	Text(FString::Printf(TEXT("Button mapping: %s"), *Mapping), PX + 0.02 * W, RowY, Grey, 0, 0);
	RowY += 0.03 * H;
	Text(FString::Printf(TEXT("Hat switch looks around. Settings are saved to %s (edit it to change buttons)."), *Joy.GetConfigPath()),
		PX + 0.02 * W, RowY, Grey, 0, 0);
	Text(TEXT("Pitch: pull back = +. Throttle: full forward = +1 (TOGA). Use INV if a bar moves the wrong way."),
		PX + 0.02 * W, PY + PH - 0.025 * H, FLinearColor(0.8f, 0.85f, 0.9f), 0, 0);
}
