// AA320World's weather: the sun or the moon, the sky light, the fog, a cloud deck, rain around the
// camera and the exposure, from the core's A320WeatherInfo (a320_weather_info).
#include "A320World.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionTextureCoordinate.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Math/RandomStream.h"

namespace
{
	constexpr double FtToCm = 30.48;
	constexpr int32 CloudPx = 512;
	constexpr double CloudTileCm = 500000.0;   // one texture tile: 5 km of sky
	constexpr double CloudDeckCm = 20000000.0; // the deck: 200 km across, following the camera by whole tiles
	constexpr double RainBoxCm = 5000.0;       // drops in 50 x 50 m around the camera...
	constexpr double RainRepeatCm = 6000.0;    // ...three stacked copies of a 60 m tall block
	constexpr int32 RainDrops = 400;

	// Exponential height fog density for a visibility (5% contrast) in metres: the fog is
	// 1 - 2^(-density / 1000 per cm), and 5% is left after log2(20) = 4.32.
	double FogDensityFor(double VisibilityM)
	{
		return 4.32 / (VisibilityM * 100.0) * 1000.0;
	}

	// Tileable value noise, a few octaves: the cloud texture repeats without seams.
	float Hash(int32 X, int32 Y, int32 Seed)
	{
		uint32 H = static_cast<uint32>(X) * 374761393u + static_cast<uint32>(Y) * 668265263u + static_cast<uint32>(Seed) * 2246822519u;
		H = (H ^ (H >> 13)) * 1274126177u;
		return static_cast<float>((H ^ (H >> 16)) & 0xFFFFu) / 65535.0f;
	}

	float PeriodicNoise(float X, float Y, int32 Period, int32 Seed)
	{
		const int32 X0 = FMath::FloorToInt(X), Y0 = FMath::FloorToInt(Y);
		const float Fx = X - X0, Fy = Y - Y0;
		const float Sx = Fx * Fx * (3.0f - 2.0f * Fx), Sy = Fy * Fy * (3.0f - 2.0f * Fy);
		auto At = [&](int32 Ix, int32 Iy) { return Hash(((Ix % Period) + Period) % Period, ((Iy % Period) + Period) % Period, Seed); };
		const float A = FMath::Lerp(At(X0, Y0), At(X0 + 1, Y0), Sx);
		const float B = FMath::Lerp(At(X0, Y0 + 1), At(X0 + 1, Y0 + 1), Sx);
		return FMath::Lerp(A, B, Sy);
	}

	// A translucent, lit, two-sided material: a tiled texture parameter times a tint, its alpha the
	// opacity. Built in memory, so only with the editor's material compiler (Play.bat).
	UMaterialInterface* MakeCloudMaterial(UTexture2D* Texture)
	{
#if WITH_EDITOR
		UMaterial* Material = NewObject<UMaterial>(GetTransientPackage(),
			MakeUniqueObjectName(GetTransientPackage(), UMaterial::StaticClass(), TEXT("M_A320Clouds")), RF_Transient);
		UMaterialExpressionTextureCoordinate* Coord = NewObject<UMaterialExpressionTextureCoordinate>(Material);
		Coord->UTiling = static_cast<float>(CloudDeckCm / CloudTileCm);
		Coord->VTiling = Coord->UTiling;
		UMaterialExpressionTextureSampleParameter2D* Sample = NewObject<UMaterialExpressionTextureSampleParameter2D>(Material);
		Sample->ParameterName = TEXT("Clouds");
		Sample->Texture = Texture;
		Sample->SamplerType = SAMPLERTYPE_Color;
		Sample->Coordinates.Connect(0, Coord);
		UMaterialExpressionVectorParameter* Tint = NewObject<UMaterialExpressionVectorParameter>(Material);
		Tint->ParameterName = TEXT("Tint");
		Tint->DefaultValue = FLinearColor::White;
		UMaterialExpressionScalarParameter* Opacity = NewObject<UMaterialExpressionScalarParameter>(Material);
		Opacity->ParameterName = TEXT("Opacity");
		Opacity->DefaultValue = 1.0f;
		UMaterialExpressionMultiply* Colour = NewObject<UMaterialExpressionMultiply>(Material);
		Colour->A.Connect(0, Sample);
		Colour->B.Connect(0, Tint);
		UMaterialExpressionMultiply* Alpha = NewObject<UMaterialExpressionMultiply>(Material);
		Alpha->A.Connect(4, Sample);  // the sample's alpha output
		Alpha->B.Connect(0, Opacity);
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Coord, Sample, Tint, Opacity, Colour, Alpha})
		{
			E->Material = Material;
			Material->GetExpressionCollection().AddExpression(E);
		}
		UMaterialEditorOnlyData* Inputs = Material->GetEditorOnlyData();
		Inputs->BaseColor.Connect(0, Colour);
		Inputs->Opacity.Connect(0, Alpha);
		Material->BlendMode = BLEND_Translucent;
		Material->TwoSided = true;
		Material->PostEditChange();
		return Material;
#else
		(void)Texture;
		return nullptr;
#endif
	}
}

UMaterialInterface* AA320World::Glow(const FLinearColor& Color)
{
	const uint32 Key = Color.ToFColor(true).ToPackedARGB();
	if (TObjectPtr<UMaterialInstanceDynamic>* Found = GlowCache.Find(Key))
	{
		return *Found;
	}
	UTexture2D* Texture = UnlitTextureMaterial ? UTexture2D::CreateTransient(1, 1, PF_B8G8R8A8) : nullptr;
	if (!Texture)
	{
		return Shapes.Tint(this, Color);
	}
	Texture->SRGB = true;
	Texture->NeverStream = true;
	FTexture2DMipMap& Mip = Texture->GetPlatformData()->Mips[0];
	Mip.BulkData.Lock(LOCK_READ_WRITE);
	const FColor Pixel = Color.ToFColor(true);
	FMemory::Memcpy(Mip.BulkData.Realloc(sizeof(FColor)), &Pixel, sizeof(FColor));
	Mip.BulkData.Unlock();
	Texture->UpdateResource();
	GlowTextures.Add(Texture);
	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(UnlitTextureMaterial, this);
	Mid->SetTextureParameterValue(TEXT("SlateUI"), Texture);
	GlowCache.Add(Key, Mid);
	return Mid;
}

void AA320World::MakeCloudTexture(double Cover)
{
	// White to light grey puffs; alpha from the noise above a threshold set by the cover.
	if (!CloudTexture)
	{
		CloudTexture = UTexture2D::CreateTransient(CloudPx, CloudPx, PF_B8G8R8A8);
		if (!CloudTexture)
		{
			return;
		}
		CloudTexture->SRGB = true;
		CloudTexture->NeverStream = true;
		CloudTexture->AddressX = TA_Wrap;
		CloudTexture->AddressY = TA_Wrap;
		// The mips, so the far deck does not shimmer.
		for (int32 Size = CloudPx / 2; Size >= 1; Size /= 2)
		{
			FTexture2DMipMap* Mip = new FTexture2DMipMap(Size, Size, 1);
			CloudTexture->GetPlatformData()->Mips.Add(Mip);
		}
	}
	const float Threshold = static_cast<float>(1.0 - Cover);
	TArray<FColor> Level;
	Level.SetNumUninitialized(CloudPx * CloudPx);
	for (int32 Y = 0; Y < CloudPx; ++Y)
	{
		for (int32 X = 0; X < CloudPx; ++X)
		{
			float N = 0.0f, Amp = 0.5f, Sum = 0.0f;
			for (int32 Octave = 0, Period = 4; Octave < 5; ++Octave, Period *= 2, Amp *= 0.5f)
			{
				const float Scale = static_cast<float>(Period) / CloudPx;
				N += Amp * PeriodicNoise(X * Scale, Y * Scale, Period, 17 + Octave);
				Sum += Amp;
			}
			N /= Sum;
			const float A = Cover >= 0.99 ? 1.0f : FMath::SmoothStep(Threshold - 0.1f, Threshold + 0.1f, N);
			const uint8 Shade = static_cast<uint8>(FMath::Clamp(190.0f + 65.0f * N, 0.0f, 255.0f));
			Level[Y * CloudPx + X] = FColor(Shade, Shade, Shade, static_cast<uint8>(A * 255.0f));
		}
	}
	int32 Size = CloudPx;
	for (int32 m = 0; m < CloudTexture->GetPlatformData()->Mips.Num(); ++m)
	{
		FTexture2DMipMap& Mip = CloudTexture->GetPlatformData()->Mips[m];
		Mip.BulkData.Lock(LOCK_READ_WRITE);
		void* Dest = Mip.BulkData.Realloc(static_cast<int64>(Size) * Size * sizeof(FColor));
		FMemory::Memcpy(Dest, Level.GetData(), static_cast<SIZE_T>(Size) * Size * sizeof(FColor));
		Mip.BulkData.Unlock();
		if (Size == 1)
		{
			break;
		}
		// The next level: 2 x 2 averages.
		const int32 Half = Size / 2;
		TArray<FColor> Next;
		Next.SetNumUninitialized(Half * Half);
		for (int32 Y = 0; Y < Half; ++Y)
		{
			for (int32 X = 0; X < Half; ++X)
			{
				uint32 C[4] = {0, 0, 0, 0};
				for (int32 k = 0; k < 4; ++k)
				{
					const FColor P = Level[(2 * Y + k / 2) * Size + 2 * X + k % 2];
					C[0] += P.B;
					C[1] += P.G;
					C[2] += P.R;
					C[3] += P.A;
				}
				Next[Y * Half + X] = FColor(static_cast<uint8>(C[2] / 4), static_cast<uint8>(C[1] / 4), static_cast<uint8>(C[0] / 4), static_cast<uint8>(C[3] / 4));
			}
		}
		Level = MoveTemp(Next);
		Size = Half;
	}
	CloudTexture->UpdateResource();
}

void AA320World::BuildWeather()
{
	// The cloud deck: two sheets (underside and top) that follow the camera by whole texture tiles.
	MakeCloudTexture(0.6);
	if (UMaterialInterface* Base = (PlaneMesh && CloudTexture) ? MakeCloudMaterial(CloudTexture) : nullptr)
	{
		for (int32 i = 0; i < 2; ++i)
		{
			UStaticMeshComponent* Sheet = NewObject<UStaticMeshComponent>(this);
			Sheet->SetMobility(EComponentMobility::Movable);
			Sheet->SetStaticMesh(PlaneMesh);
			Sheet->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Sheet->SetCastShadow(false);
			Sheet->SetAbsolute(true, true, true);
			Sheet->SetupAttachment(Root);
			Sheet->SetWorldScale3D(FVector(CloudDeckCm / 100.0, CloudDeckCm / 100.0, 1.0));
			UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, this);
			Mid->SetTextureParameterValue(TEXT("Clouds"), CloudTexture);
			Sheet->SetMaterial(0, Mid);
			Sheet->SetVisibility(false);
			Sheet->RegisterComponent();
			CloudSheets.Add(Sheet);
		}
	}
	// Rain: thin streaks in a block around the camera, stacked three high so it falls seamlessly.
	Rain = NewObject<UInstancedStaticMeshComponent>(this);
	Rain->SetMobility(EComponentMobility::Movable);
	Rain->SetStaticMesh(Shapes.Cube);
	Rain->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Rain->SetCastShadow(false);
	Rain->SetAbsolute(true, true, true);
	Rain->SetupAttachment(Root);
	Rain->RegisterComponent();
	FRandomStream Random(320);
	for (int32 Copy = 0; Copy < 3; ++Copy)
	{
		Random.Initialize(320);
		for (int32 i = 0; i < RainDrops; ++i)
		{
			const FVector At(Random.FRandRange(-RainBoxCm, RainBoxCm), Random.FRandRange(-RainBoxCm, RainBoxCm),
				Random.FRandRange(0.0, RainRepeatCm) + Copy * RainRepeatCm);
			Rain->AddInstance(FTransform(FRotator::ZeroRotator, At, FVector(0.012, 0.012, 0.8)));
		}
	}
	Rain->SetVisibility(false);
	SetWeather(Weather, bNight);
}

void AA320World::SetWeather(int32 InWeather, bool bInNight)
{
	Weather = InWeather;
	bNight = bInNight;
	if (!a320_weather_info(Weather, &WeatherInfo))
	{
		Weather = A320_WEATHER_SUNNY;
		a320_weather_info(Weather, &WeatherInfo);
	}
	const bool bOvercast = WeatherInfo.cloudCover >= 0.99;

	// The sun by day; by night a dim, bluish moon (the sky goes dark with it).
	Sun->SetRelativeRotation(bNight ? FRotator(-40.0, 120.0, 0.0) : FRotator(-35.0, 200.0, 0.0));
	const double Daylight = bOvercast ? 2.2 : WeatherInfo.cloudCover > 0.0 ? 5.5 : 8.0;
	Sun->SetIntensity(static_cast<float>(bNight ? Daylight * 0.015 : Daylight));
	Sun->SetLightColor(bNight ? FLinearColor(0.55f, 0.65f, 1.0f) : FLinearColor::White);

	// Fog: the visibility near the ground; a fog bank is shallow, above it the air is clear.
	ClearFogDensity = FogDensityFor(WeatherInfo.visibilityM);
	Fog->SetFogDensity(static_cast<float>(ClearFogDensity));
	// Height falloff per 10 m: a fog bank halves every 15 m; otherwise every ~200 m.
	Fog->SetFogHeightFalloff(WeatherInfo.fogTopFt > 0.0 ? 0.67f : 0.05f);
	const FLinearColor FogColour = WeatherInfo.fogTopFt > 0.0 ? FLinearColor(0.55f, 0.57f, 0.6f)
		: bOvercast ? FLinearColor(0.35f, 0.37f, 0.4f) : FLinearColor(0.45f, 0.55f, 0.7f);
	Fog->SetFogInscatteringColor(bNight ? FogColour * 0.03f : FogColour);

	// Clouds: a lighter top, a darker underside; grey for rain.
	const double BaseCm = WeatherInfo.cloudBaseFt * FtToCm, TopCm = WeatherInfo.cloudTopFt * FtToCm;
	if (WeatherInfo.cloudCover > 0.0)
	{
		MakeCloudTexture(WeatherInfo.cloudCover);
	}
	for (int32 i = 0; i < CloudSheets.Num(); ++i)
	{
		UStaticMeshComponent* Sheet = CloudSheets[i];
		Sheet->SetVisibility(WeatherInfo.cloudCover > 0.0);
		const FVector At = Sheet->GetComponentLocation();
		Sheet->SetWorldLocation(FVector(At.X, At.Y, i == 0 ? BaseCm : TopCm));
		if (UMaterialInstanceDynamic* Mid = Cast<UMaterialInstanceDynamic>(Sheet->GetMaterial(0)))
		{
			const float Grey = (bOvercast ? 0.55f : 0.9f) * (i == 0 ? 0.8f : 1.0f);
			Mid->SetVectorParameterValue(TEXT("Tint"), FLinearColor(Grey, Grey, Grey));
			Mid->SetScalarParameterValue(TEXT("Opacity"), bOvercast ? 1.0f : 0.92f);
		}
	}

	// Rain: light grey streaks by day, faint ones at night.
	if (Rain)
	{
		Rain->SetMaterial(0, Glow(bNight ? FLinearColor(0.05f, 0.055f, 0.06f) : FLinearColor(0.55f, 0.58f, 0.62f)));
		Rain->SetVisibility(WeatherInfo.rain != 0);
	}

	// Night: the exposure stays dark (auto exposure would make it day); lights and the sky's last
	// light stand out.
	if (!Exposure && GetWorld())
	{
		FActorSpawnParameters Params;
		Params.Owner = this;
		Exposure = GetWorld()->SpawnActor<APostProcessVolume>(APostProcessVolume::StaticClass(), FTransform::Identity, Params);
		if (Exposure)
		{
			Exposure->bUnbound = true;
		}
	}
	if (Exposure)
	{
		Exposure->Settings.bOverride_AutoExposureBias = true;
		Exposure->Settings.AutoExposureBias = bNight ? -3.0f : (bOvercast ? -0.3f : 0.0f);
	}
}

void AA320World::TickWeather(float DeltaSeconds, const FVector& CameraCm)
{
	// The deck under and over the camera, moved by whole texture tiles so the clouds stay put.
	const double SnapX = FMath::RoundToDouble(CameraCm.X / CloudTileCm) * CloudTileCm;
	const double SnapY = FMath::RoundToDouble(CameraCm.Y / CloudTileCm) * CloudTileCm;
	for (UStaticMeshComponent* Sheet : CloudSheets)
	{
		const FVector At = Sheet->GetComponentLocation();
		if (At.X != SnapX || At.Y != SnapY)
		{
			Sheet->SetWorldLocation(FVector(SnapX, SnapY, At.Z));
		}
	}
	// Inside the cloud the world goes white: the fog closes in (100 m overcast, ~800 m broken).
	const double BaseCm = WeatherInfo.cloudBaseFt * FtToCm, TopCm = WeatherInfo.cloudTopFt * FtToCm;
	double Inside = 0.0;
	if (WeatherInfo.cloudCover > 0.0 && TopCm > BaseCm)
	{
		const double Edge = 3000.0;  // 30 m to fade in and out
		Inside = FMath::Clamp(FMath::Min(CameraCm.Z - BaseCm, TopCm - CameraCm.Z) / Edge + 0.5, 0.0, 1.0);
	}
	const double InCloud = FogDensityFor(WeatherInfo.cloudCover >= 0.99 ? 100.0 : 800.0);
	const float Density = static_cast<float>(FMath::Lerp(ClearFogDensity, InCloud, Inside));
	if (!FMath::IsNearlyEqual(Fog->FogDensity, Density, Density * 0.01f))
	{
		Fog->SetFogDensity(Density);
	}
	// Rain falls from the cloud base down, 9 m/s, around the camera.
	if (Rain && WeatherInfo.rain)
	{
		const bool bBelow = CameraCm.Z < BaseCm + 5000.0;
		Rain->SetVisibility(bBelow);
		RainFallCm += DeltaSeconds * 900.0;
		const double Offset = FMath::Fmod(RainFallCm, RainRepeatCm);
		Rain->SetWorldLocation(FVector(CameraCm.X, CameraCm.Y, CameraCm.Z - 2.0 * RainRepeatCm + RainRepeatCm - Offset));
	}
}
