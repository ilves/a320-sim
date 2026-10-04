#include "A320Fx.h"

#include "Components/PointLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	constexpr int32 FlameCount = 8;
	constexpr double GravityCm = 981.0;

	const FLinearColor FireYellow(1.0f, 0.85f, 0.25f);
	const FLinearColor FireOrange(1.0f, 0.45f, 0.05f);
	const FLinearColor FireRed(0.8f, 0.15f, 0.02f);
	const FLinearColor Smoke(0.07f, 0.07f, 0.075f);
	const FLinearColor DebrisLight(0.6f, 0.62f, 0.65f);
	const FLinearColor DebrisDark(0.08f, 0.08f, 0.08f);

	double SmoothStep(double A, double B, double X)
	{
		const double T = FMath::Clamp((X - A) / (B - A), 0.0, 1.0);
		return T * T * (3.0 - 2.0 * T);
	}

	// Pool members not in use are hidden and scaled to nothing.
	const FVector Nothing(0.001);
}

AA320Fx::AA320Fx()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
	Shapes.LoadInConstructor();
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> UnlitFinder(
		TEXT("/Engine/EngineMaterials/Widget3DPassThrough_Opaque.Widget3DPassThrough_Opaque"));
	UnlitMaterial = UnlitFinder.Object;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Root->SetMobility(EComponentMobility::Movable);
	SetRootComponent(Root);

	Light = CreateDefaultSubobject<UPointLightComponent>(TEXT("Light"));
	Light->SetMobility(EComponentMobility::Movable);  // moved and dimmed every frame
	Light->SetupAttachment(Root);
	Light->SetIntensityUnits(ELightUnits::Candelas);
	Light->SetIntensity(0.0f);
	Light->SetLightColor(FLinearColor(1.0f, 0.55f, 0.2f));
	Light->SetAttenuationRadius(150000.0f);
	Light->SetCastShadows(false);
}

UStaticMeshComponent* AA320Fx::AddMesh(UStaticMesh* Mesh, UMaterialInterface* Material)
{
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
	C->SetMobility(EComponentMobility::Movable);
	C->SetStaticMesh(Mesh);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetCastShadow(false);
	C->SetupAttachment(Root);
	C->SetUsingAbsoluteLocation(true);
	C->SetUsingAbsoluteRotation(true);
	C->SetUsingAbsoluteScale(true);
	C->SetWorldScale3D(Nothing);
	C->SetMaterial(0, Material);
	C->RegisterComponent();
	return C;
}

UMaterialInterface* AA320Fx::Glow(const FLinearColor& Color)
{
	const uint32 Key = Color.ToFColor(true).ToPackedARGB();
	if (TObjectPtr<UMaterialInstanceDynamic>* Found = GlowCache.Find(Key))
	{
		return *Found;
	}
	UTexture2D* Texture = UnlitMaterial ? UTexture2D::CreateTransient(1, 1, PF_B8G8R8A8) : nullptr;
	if (!Texture)
	{
		return Shapes.Tint(this, Color);
	}
	Texture->SRGB = true;
	Texture->NeverStream = true;
	FTexture2DMipMap& Mip = Texture->GetPlatformData()->Mips[0];
	Mip.BulkData.Lock(LOCK_READ_WRITE);
	const FColor Pixel = Color.ToFColor(true);  // BGRA in memory, as PF_B8G8R8A8
	FMemory::Memcpy(Mip.BulkData.Realloc(sizeof(FColor)), &Pixel, sizeof(FColor));
	Mip.BulkData.Unlock();
	Texture->UpdateResource();
	GlowTextures.Add(Texture);
	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(UnlitMaterial, this);
	Mid->SetTextureParameterValue(TEXT("SlateUI"), Texture);
	GlowCache.Add(Key, Mid);
	return Mid;
}

void AA320Fx::Start(const FA320FxSpec& InSpec)
{
	Spec = InSpec;
	Source = GetActorLocation();
	Random.Initialize(static_cast<int32>(Source.X + Source.Y) ^ 0x320);
	const double S = Spec.SizeM / 30.0;  // effects are designed for a 30 m fireball

	if (Spec.bExplosion)
	{
		FireballOuter = AddMesh(Shapes.Sphere, Glow(FireOrange));
		FireballInner = AddMesh(Shapes.Sphere, Glow(FireYellow));
	}
	if (Spec.bFire)
	{
		const FLinearColor Colors[] = {FireOrange, FireYellow, FireRed, FireOrange};
		for (int32 i = 0; i < FlameCount; ++i)
		{
			Flames.Add(AddMesh(Shapes.Cone, Glow(Colors[i % 4])));
			const double R = Random.FRandRange(0.0, 9.0 * S) * 100.0, A = Random.FRandRange(0.0, 2.0 * UE_DOUBLE_PI);
			FlameOffsets.Add(FVector(R * FMath::Cos(A), R * FMath::Sin(A), 0.0));
		}
	}
	if (Spec.bSmoke)
	{
		UMaterialInterface* SmokeMaterial = Shapes.Tint(this, Smoke);
		Puffs.SetNum(Spec.PuffPool);
		for (int32 i = 0; i < Spec.PuffPool; ++i)
		{
			UStaticMeshComponent* Puff = AddMesh(Shapes.Sphere, SmokeMaterial);
			Puff->SetVisibility(false);
			PuffMeshes.Add(Puff);
		}
	}
	// Debris: thrown out of the fireball, keeping part of the wreck's motion.
	for (int32 i = 0; i < Spec.DebrisCount; ++i)
	{
		FPiece P;
		P.Pos = Source + FVector(Random.FRandRange(-300.0, 300.0), Random.FRandRange(-300.0, 300.0), Random.FRandRange(0.0, 300.0)) * S;
		const FVector Dir = FVector(Random.FRandRange(-1.0, 1.0), Random.FRandRange(-1.0, 1.0), Random.FRandRange(0.3, 1.2)).GetSafeNormal();
		P.Vel = Spec.DebrisVelocity * Random.FRandRange(0.2, 0.6) + Dir * Random.FRandRange(1500.0, 4500.0) * FMath::Sqrt(S);
		const double Big = Random.FRand() < 0.2 ? 3.0 : 1.0;
		P.SizeM = FVector(Random.FRandRange(0.4, 1.8), Random.FRandRange(0.3, 1.2), Random.FRandRange(0.1, 0.5)) * Big;
		P.Rot = FRotator(Random.FRandRange(0.0, 360.0), Random.FRandRange(0.0, 360.0), Random.FRandRange(0.0, 360.0));
		P.Spin = FRotator(Random.FRandRange(-400.0, 400.0), Random.FRandRange(-400.0, 400.0), Random.FRandRange(-400.0, 400.0));
		Pieces.Add(P);
		PieceMeshes.Add(AddMesh(Shapes.Cube, Shapes.Tint(this, i % 3 == 0 ? DebrisLight : DebrisDark)));
	}
	Light->SetAttenuationRadius(static_cast<float>(150000.0 * FMath::Max(S, 0.4)));
	Age = 0.0;
	NextPuffAt = 0.0;
	LastEmit = Source;
	bStarted = true;
}

void AA320Fx::MoveSource(const FVector& LocationCm)
{
	Source = LocationCm;
	SetActorLocation(LocationCm);
}

void AA320Fx::Extinguish()
{
	bExtinguished = true;
	for (UStaticMeshComponent* Flame : Flames)
	{
		Flame->SetVisibility(false);
	}
}

void AA320Fx::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!bStarted)
	{
		return;
	}
	const double Dt = FMath::Min(static_cast<double>(DeltaSeconds), 0.1);
	Age += Dt;
	UpdateExplosion(Age);
	UpdateFire(Age);
	UpdateSmoke(Dt);
	UpdateDebris(Dt);
}

void AA320Fx::UpdateExplosion(double T)
{
	const double S = Spec.SizeM / 30.0;
	// The flash, then the fire's flicker.
	const double Flash = Spec.bExplosion ? 3.0e6 * S * FMath::Exp(-T * 2.5) : 0.0;
	const double Flicker = Spec.bFire && !bExtinguished
		? 2.0e5 * S * SmoothStep(0.0, 1.0, T) * (0.75 + 0.25 * FMath::Sin(T * 13.0) * FMath::Sin(T * 7.3))
		: 0.0;
	Light->SetIntensity(static_cast<float>(Flash + Flicker));
	Light->SetWorldLocation(Source + FVector(0.0, 0.0, 800.0 * S));
	if (!FireballOuter)
	{
		return;
	}
	if (T > 2.4)
	{
		FireballOuter->SetVisibility(false);
		FireballInner->SetVisibility(false);
		return;
	}
	// Grows in a third of a second, keeps swelling and rising, then burns out.
	const double R = Spec.SizeM * (T < 0.35 ? T / 0.35 : 1.0 + 0.3 * (T - 0.35));
	const double Out = 1.0 - SmoothStep(1.0, 2.4, T);
	const FVector Centre = Source + FVector(0.0, 0.0, (0.2 * Spec.SizeM + 6.0 * S * T) * 100.0);
	FireballOuter->SetWorldLocationAndRotation(Centre, FRotator::ZeroRotator);
	FireballOuter->SetWorldScale3D(FVector(2.0 * R * FMath::Max(Out, 0.01)));
	FireballInner->SetWorldLocationAndRotation(Centre + FVector(0.0, 0.0, -0.1 * R * 100.0), FRotator::ZeroRotator);
	FireballInner->SetWorldScale3D(FVector(1.5 * R * FMath::Max(1.0 - SmoothStep(0.5, 1.8, T), 0.01)));
}

void AA320Fx::UpdateFire(double T)
{
	if (bExtinguished || Flames.Num() == 0)
	{
		return;
	}
	const double S = Spec.SizeM / 30.0;
	const double Grow = SmoothStep(0.2, 1.5, T);
	for (int32 i = 0; i < Flames.Num(); ++i)
	{
		// Each tongue flickers on its own rhythm.
		const double F = 0.7 + 0.2 * FMath::Sin(T * (6.0 + i) + i * 1.7) + 0.1 * Random.FRand();
		const double HeightM = (10.0 + 4.0 * (i % 3)) * S * F * Grow;
		const double WidthM = (5.0 + (i % 2) * 2.0) * S * Grow;
		Flames[i]->SetWorldLocationAndRotation(Source + FlameOffsets[i] + FVector(0.0, 0.0, HeightM * 50.0),
			FRotator(0.0, T * 40.0 * (i % 2 ? 1.0 : -1.0), 0.0));
		Flames[i]->SetWorldScale3D(FVector(FMath::Max(WidthM, 0.01), FMath::Max(WidthM, 0.01), FMath::Max(HeightM, 0.01)));
	}
}

void AA320Fx::EmitPuff(const FVector& At)
{
	const double S = Spec.SizeM / 30.0 * Spec.PuffScale;
	for (int32 i = 0; i < Puffs.Num(); ++i)
	{
		FPuff& P = Puffs[i];
		if (P.bAlive)
		{
			continue;
		}
		P.bAlive = true;
		P.Age = 0.0;
		P.Life = Spec.PuffLifeS * Random.FRandRange(0.7, 1.0);
		P.SizeM = Random.FRandRange(6.0, 10.0) * S;
		P.Pos = At + FVector(Random.FRandRange(-500.0, 500.0), Random.FRandRange(-500.0, 500.0), 300.0) * S;
		P.Vel = FVector(Random.FRandRange(150.0, 350.0), Random.FRandRange(50.0, 200.0), Random.FRandRange(500.0, 900.0) * S);
		PuffMeshes[i]->SetVisibility(true);
		return;
	}
}

void AA320Fx::UpdateSmoke(double Dt)
{
	if (PuffMeshes.Num() == 0)
	{
		return;
	}
	const double S = Spec.SizeM / 30.0 * Spec.PuffScale;
	// New puffs while burning: in time on the ground, and spaced along the path of a falling piece
	// so its trail stays continuous.
	if (!bExtinguished && Age > 0.3)
	{
		const double SpacingCm = 0.7 * 8.0 * S * 100.0;
		const double Moved = FVector::Dist(Source, LastEmit);
		int32 Count = 0;
		if (Moved >= SpacingCm)
		{
			Count = FMath::Min(FMath::FloorToInt32(Moved / SpacingCm), 6);
		}
		else if (Age >= NextPuffAt)
		{
			Count = 1;
		}
		for (int32 k = 1; k <= Count; ++k)
		{
			EmitPuff(FMath::Lerp(LastEmit, Source, static_cast<double>(k) / Count));
		}
		if (Count > 0)
		{
			LastEmit = Source;
			NextPuffAt = Age + 0.09;
		}
	}
	for (int32 i = 0; i < Puffs.Num(); ++i)
	{
		FPuff& P = Puffs[i];
		if (!P.bAlive)
		{
			continue;
		}
		P.Age += Dt;
		if (P.Age > P.Life)
		{
			P.bAlive = false;
			PuffMeshes[i]->SetVisibility(false);
			PuffMeshes[i]->SetWorldScale3D(Nothing);
			continue;
		}
		// Rising slows as the smoke cools; it spreads as it goes and thins out at the end.
		P.Vel.Z *= 1.0 - 0.08 * Dt;
		P.Pos += P.Vel * Dt;
		const double Size = (P.SizeM + 1.6 * S * P.Age) * (1.0 - SmoothStep(0.75 * P.Life, P.Life, P.Age));
		PuffMeshes[i]->SetWorldLocation(P.Pos);
		PuffMeshes[i]->SetWorldScale3D(FVector(FMath::Max(Size, 0.01)));
	}
}

void AA320Fx::UpdateDebris(double Dt)
{
	for (int32 i = 0; i < Pieces.Num(); ++i)
	{
		FPiece& P = Pieces[i];
		if (P.bResting)
		{
			continue;  // placed when it came to rest
		}
		P.Vel *= 1.0 - 0.15 * Dt;  // air drag
		P.Vel.Z -= GravityCm * Dt;
		P.Pos += P.Vel * Dt;
		P.Rot += P.Spin * Dt;
		const double Rest = Spec.GroundZ + P.SizeM.Z * 50.0;
		if (P.Pos.Z <= Rest)
		{
			P.Pos.Z = Rest;
			P.Rot.Pitch = Random.FRandRange(-10.0, 10.0);
			P.Rot.Roll = Random.FRandRange(-10.0, 10.0);
			P.bResting = true;
		}
		PieceMeshes[i]->SetWorldLocationAndRotation(P.Pos, P.Rot);
		PieceMeshes[i]->SetWorldScale3D(P.SizeM);
	}
}
