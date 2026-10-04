#include "A320World.h"

#include "A320Terrain.h"
#include "a320/WorldMap.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "Camera/PlayerCameraManager.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "ProceduralMeshComponent.h"
#include "Materials/MaterialInterface.h"
#include "Math/RandomStream.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	const FLinearColor Grass(0.10f, 0.20f, 0.06f);
	const FLinearColor Forest(0.03f, 0.09f, 0.03f);
	const FLinearColor Asphalt(0.045f, 0.045f, 0.05f);
	const FLinearColor Paint(0.9f, 0.9f, 0.9f);
	const FLinearColor Concrete(0.35f, 0.35f, 0.33f);
	const FLinearColor Water(0.02f, 0.06f, 0.12f);
	const FLinearColor LightWhite(1.0f, 1.0f, 0.9f);
	const FLinearColor LightRed(1.0f, 0.02f, 0.02f);
	const FLinearColor LightGreen(0.05f, 1.0f, 0.1f);

	// Runway-aligned helper: x along the landing course from a point, y to the right, up from the
	// runway's elevation in the flat world.
	struct FRunwayFrame
	{
		double OriginN, OriginE, DirN, DirE, BaseUp;

		FRunwayFrame(double N, double E, double CourseDeg, double BaseUpM = 0.0)
			: OriginN(N), OriginE(E), DirN(FMath::Cos(FMath::DegreesToRadians(CourseDeg))),
			  DirE(FMath::Sin(FMath::DegreesToRadians(CourseDeg))), BaseUp(BaseUpM)
		{
		}

		// Metres (north, east, up) for a runway-relative point.
		FVector At(double X, double Y, double Up = 0.0) const
		{
			return FVector(OriginN + X * DirN - Y * DirE, OriginE + X * DirE + Y * DirN, BaseUp + Up);
		}
	};

	FTransform BoxTransform(const FVector& CentreM, double YawDeg, const FVector& SizeM)
	{
		return FTransform(FRotator(0.0, YawDeg, 0.0), CentreM * 100.0, SizeM);
	}

	// Paint sits 5 cm above the asphalt, which sits 5 cm above the grass.
	constexpr double RunwayTopM = 0.05;
	constexpr double PaintTopM = 0.10;
}

AA320World::AA320World()
{
	// Ticks to stream the terrain around the aircraft and move the weather with the camera, also paused.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
	PrimaryActorTick.bTickEvenWhenPaused = true;
	Shapes.LoadInConstructor();
	// The 3D widget material: opaque, unlit, one texture. Terrain fallback where no material compiler exists.
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> UnlitFinder(
		TEXT("/Engine/EngineMaterials/Widget3DPassThrough_Opaque.Widget3DPassThrough_Opaque"));
	UnlitTextureMaterial = UnlitFinder.Object;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	// Static so the static scenery can attach; movable lights may attach to a static parent.
	Root->SetMobility(EComponentMobility::Static);
	SetRootComponent(Root);

	Sky = CreateDefaultSubobject<USkyAtmosphereComponent>(TEXT("Sky"));
	Sky->SetupAttachment(Root);

	Sun = CreateDefaultSubobject<UDirectionalLightComponent>(TEXT("Sun"));
	Sun->SetupAttachment(Root);
	Sun->SetMobility(EComponentMobility::Movable);
	Sun->SetRelativeRotation(FRotator(-35.0, 200.0, 0.0));
	Sun->SetIntensity(8.0f);
	Sun->SetAtmosphereSunLight(true);

	SkyLight = CreateDefaultSubobject<USkyLightComponent>(TEXT("SkyLight"));
	SkyLight->SetupAttachment(Root);
	SkyLight->SetMobility(EComponentMobility::Movable);
	SkyLight->bRealTimeCapture = true;

	Fog = CreateDefaultSubobject<UExponentialHeightFogComponent>(TEXT("Fog"));
	Fog->SetupAttachment(Root);
	Fog->SetMobility(EComponentMobility::Movable);
	Fog->SetFogDensity(0.004f);
	Fog->SetFogHeightFalloff(0.05f);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> PlaneFinder(TEXT("/Engine/BasicShapes/Plane.Plane"));
	PlaneMesh = PlaneFinder.Object;
}

void AA320World::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	// The aircraft is the pawn; before it is possessed, the aircraft that spawned this world.
	const APlayerController* Controller = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	const AActor* Viewer = Controller ? Controller->GetPawn() : nullptr;
	if (!Viewer)
	{
		Viewer = GetOwner();
	}
	if (TerrainStreamer && Viewer)
	{
		TerrainStreamer->Tick(Viewer->GetActorLocation());
	}
	// The weather follows the camera (the chase camera is 70 m behind the aircraft).
	if (Controller && Controller->PlayerCameraManager)
	{
		TickWeather(DeltaSeconds, Controller->PlayerCameraManager->GetCameraLocation());
	}
}

void AA320World::EndPlay(const EEndPlayReason::Type Reason)
{
	if (TerrainStreamer)
	{
		TerrainStreamer->Shutdown();
		TerrainStreamer.Reset();
	}
	Super::EndPlay(Reason);
}

UStaticMeshComponent* AA320World::AddMesh(UStaticMesh* Mesh, const FVector& CentreM, double YawDeg,
	const FVector& SizeM, const FLinearColor& Color)
{
	return AddMesh(Mesh, BoxTransform(CentreM, YawDeg, SizeM), Color);
}

UStaticMeshComponent* AA320World::AddMesh(UStaticMesh* Mesh, const FTransform& Transform, const FLinearColor& Color)
{
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
	C->SetMobility(EComponentMobility::Static);
	C->SetStaticMesh(Mesh);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetupAttachment(Root);
	C->SetRelativeTransform(Transform);
	const bool bLight = Color.Equals(LightWhite) || Color.Equals(LightRed) || Color.Equals(LightGreen);
	C->SetMaterial(0, bLight ? Glow(Color) : Shapes.Tint(this, Color));
	C->RegisterComponent();
	return C;
}

UInstancedStaticMeshComponent* AA320World::AddInstanced(UStaticMesh* Mesh, const FLinearColor& Color)
{
	UInstancedStaticMeshComponent* C = NewObject<UInstancedStaticMeshComponent>(this);
	C->SetMobility(EComponentMobility::Static);
	C->SetStaticMesh(Mesh);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetupAttachment(Root);
	C->SetMaterial(0, Shapes.Tint(this, Color));
	C->RegisterComponent();
	return C;
}

void AA320World::Build(const TArray<A320RunwayInfo>& Runways, const TArray<A320AirportInfo>& Airports)
{
	auto LevelOf = [&Airports](const A320RunwayInfo& R) { return Airports.IsValidIndex(R.airport) ? Airports[R.airport].elevationM : R.elevationM; };
	if (Runways.Num() == 0)
	{
		return;
	}

	BuildWeather();
	const FA320TerrainResult Terrain = A320Terrain::Build(this, Root, Shapes, UnlitTextureMaterial);
	TerrainStreamer = Terrain.Streamer;
	if (!Terrain.bLoaded)
	{
		// Flat stand-in: grass to 40 km, with the Baltic to the north (Tallinn Bay, ~6 km).
		AddMesh(Shapes.Cube, FVector(0.0, 0.0, -0.5), 0.0, FVector(80000.0, 80000.0, 1.0), Grass);
		AddMesh(Shapes.Cube, FVector(24000.0, 0.0, -0.45), 0.0, FVector(34000.0, 80000.0, 1.0), Water);
	}

	// The paved surface of each airport's runway, once, from its first direction's start to its end.
	// Courses are grid courses: true north turns by the meridian convergence away from EETN.
	for (int32 i = 0; i < Runways.Num(); ++i)
	{
		bool bFirstOfAirport = true;
		for (int32 j = 0; j < i; ++j)
		{
			bFirstOfAirport = bFirstOfAirport && Runways[j].airport != Runways[i].airport;
		}
		if (bFirstOfAirport)
		{
			BuildPavedRunway(Runways[i], LevelOf(Runways[i]));
			if (!Terrain.bLoaded && Runways[i].airport != Runways[0].airport)
			{
				// Without scenery, a grass field under the other airports at their elevation.
				const FVector Mid((Runways[i].startNorthM + Runways[i].endNorthM) / 2.0,
					(Runways[i].startEastM + Runways[i].endEastM) / 2.0, LevelOf(Runways[i]) - 0.5);
				AddMesh(Shapes.Cube, Mid, 0.0, FVector(30000.0, 30000.0, 1.0), Grass);
			}
		}
	}

	PapiLights.Reset();
	PapiShown.Init(-1, Runways.Num() * 4);
	for (int32 i = 0; i < Runways.Num(); ++i)
	{
		BuildRunwayDirection(Runways[i], i, LevelOf(Runways[i]));
	}
	BuildAirportLayouts();
	if (!Terrain.bLoaded)
	{
		BuildSurroundings(Runways[0]);  // the real terrain has the real lake, city, forests and buildings
	}
}

void AA320World::BuildPavedRunway(const A320RunwayInfo& R0, double LevelM)
{
	const double Course = R0.gridCourseDeg;
	const double LengthM = FVector2D(R0.endNorthM - R0.startNorthM, R0.endEastM - R0.startEastM).Size();
	const FRunwayFrame Paved(R0.startNorthM, R0.startEastM, Course, LevelM);
	AddMesh(Shapes.Cube, Paved.At(LengthM / 2.0, 0.0, RunwayTopM - 0.1), Course, FVector(LengthM, R0.widthM, 0.2), Asphalt);

	// Centreline (30 m dashes, 20 m gaps) and edge lines along the whole runway.
	const double PaintZ = (RunwayTopM + PaintTopM) / 2.0;
	const double PaintH = PaintTopM - RunwayTopM;
	for (double X = 80.0; X + 30.0 < LengthM - 80.0; X += 50.0)
	{
		AddMesh(Shapes.Cube, BoxTransform(Paved.At(X + 15.0, 0.0, PaintZ), Course, FVector(30.0, 0.9, PaintH)), Paint);
	}
	for (const double Side : {-1.0, 1.0})
	{
		AddMesh(Shapes.Cube, BoxTransform(Paved.At(LengthM / 2.0, Side * (R0.widthM / 2.0 - 1.0), PaintZ),
			Course, FVector(LengthM, 0.9, PaintH)), Paint);
		for (double X = 0.0; X <= LengthM; X += 60.0)
		{
			AddMesh(Shapes.Sphere, BoxTransform(Paved.At(X, Side * (R0.widthM / 2.0 + 1.5), 0.3),
				Course, FVector(0.45)), LightWhite);
		}
	}
}

void AA320World::BuildRunwayDirection(const A320RunwayInfo& Runway, int32 Index, double LevelM)
{
	const double Course = Runway.gridCourseDeg;
	const double HalfWidth = Runway.widthM / 2.0;
	const FRunwayFrame Thr(Runway.thresholdNorthM, Runway.thresholdEastM, Course, LevelM);
	const double PaintZ = (RunwayTopM + PaintTopM) / 2.0;
	const double PaintH = PaintTopM - RunwayTopM;
	auto AddPaint = [&](double X, double Y, double Length, double Width)
	{
		AddMesh(Shapes.Cube, BoxTransform(Thr.At(X + Length / 2.0, Y, PaintZ), Course, FVector(Length, Width, PaintH)), Paint);
	};

	// Threshold "piano keys": 6 stripes either side of the centreline.
	for (int32 k = 0; k < 6; ++k)
	{
		for (const double Side : {-1.0, 1.0})
		{
			AddPaint(6.0, Side * (2.7 + k * 3.4), 30.0, 1.8);
		}
	}
	// Aiming point at 400 m and touchdown zone stripes.
	for (const double Side : {-1.0, 1.0})
	{
		AddPaint(400.0, Side * 12.0, 45.0, 6.0);
		const double Zones[] = {150.0, 300.0, 600.0, 750.0, 900.0};
		const int32 Counts[] = {3, 3, 2, 2, 1};
		for (int32 z = 0; z < 5; ++z)
		{
			for (int32 k = 0; k < Counts[z]; ++k)
			{
				AddPaint(Zones[z], Side * (10.0 + k * 3.0), 22.5, 1.8);
			}
		}
	}

	// Threshold (green) and runway end (red) lights.
	for (double Y = -HalfWidth; Y <= HalfWidth + 0.1; Y += 4.5)
	{
		AddMesh(Shapes.Sphere, BoxTransform(Thr.At(-1.0, Y, 0.3), Course, FVector(0.5)), LightGreen);
		AddMesh(Shapes.Sphere, BoxTransform(Thr.At(Runway.landingDistanceM + 1.0, Y, 0.3), Course, FVector(0.5)), LightRed);
	}

	// Approach lights: 900 m of centreline barrettes with a crossbar at 300 m.
	for (double X = 30.0; X <= 900.0; X += 30.0)
	{
		for (int32 k = -2; k <= 2; ++k)
		{
			AddMesh(Shapes.Cube, BoxTransform(Thr.At(-X, k * 1.0, 1.0), Course, FVector(0.6)), LightWhite);
		}
	}
	for (double Y = -15.0; Y <= 15.0; Y += 1.5)
	{
		AddMesh(Shapes.Cube, BoxTransform(Thr.At(-300.0, Y, 1.0), Course, FVector(0.6)), LightWhite);
	}

	// PAPI left of the runway at the glideslope origin; the inner unit is 15 m from the
	// edge, units 9 m apart. Boxes are oversized so they read from a few miles out.
	const FRunwayFrame Gs(Runway.gsOriginNorthM, Runway.gsOriginEastM, Course, LevelM);
	for (int32 i = 0; i < 4; ++i)
	{
		const double Y = -(HalfWidth + 15.0 + (3 - i) * 9.0);
		PapiLights.Add(AddMesh(Shapes.Cube, Gs.At(0.0, Y, 1.0), Course, FVector(1.0, 3.0, 1.5), LightRed));
	}
	check(PapiLights.Num() == (Index + 1) * 4);
}

void AA320World::BuildSurroundings(const A320RunwayInfo& Runway)
{
	const double MidN = (Runway.startNorthM + Runway.endNorthM) / 2.0;
	const double MidE = (Runway.startEastM + Runway.endEastM) / 2.0;
	const FRunwayFrame Mid(MidN, MidE, Runway.trueCourseDeg);

	// Terminal, apron and tower north of the runway (EETN's terminal side).
	AddMesh(Shapes.Cube, FVector(MidN + 420.0, MidE + 300.0, 0.03), 0.0, FVector(260.0, 700.0, 0.1), Concrete);
	AddMesh(Shapes.Cube, FVector(MidN + 600.0, MidE + 300.0, 8.0), 0.0, FVector(70.0, 320.0, 16.0), FLinearColor(0.55f, 0.55f, 0.6f));
	AddMesh(Shapes.Cylinder, FVector(MidN + 640.0, MidE - 150.0, 22.0), 0.0, FVector(8.0, 8.0, 44.0), FLinearColor(0.7f, 0.7f, 0.7f));
	AddMesh(Shapes.Cube, FVector(MidN + 500.0, MidE + 1100.0, 10.0), 0.0, FVector(90.0, 120.0, 20.0), FLinearColor(0.4f, 0.42f, 0.45f));

	// Lake Ulemiste, just west of the airport.
	AddMesh(Shapes.Cylinder, FVector(-1300.0, -3300.0, 0.02), 0.0, FVector(2600.0, 2300.0, 0.1), Water);

	FRandomStream Random(320);
	auto ClearOfRunway = [&](double N, double E)
	{
		const double Dn = N - MidN, De = E - MidE;
		const double Along = Dn * Mid.DirN + De * Mid.DirE;
		const double Across = -Dn * Mid.DirE + De * Mid.DirN;
		const bool OnApproachPaths = FMath::Abs(Across) < 500.0 && FMath::Abs(Along) < 16000.0;
		const bool OnAirport = FMath::Abs(Across) < 1200.0 && FMath::Abs(Along) < 2600.0;
		const bool InLake = FVector2D(N + 1300.0, (E + 3300.0) * 0.9).Size() < 1400.0;
		return !OnApproachPaths && !OnAirport && !InLake && N < 5800.0;
	};

	// City and forest are instanced (thousands of boxes). If a packaged build shows them with
	// the default grey material, BasicShapeMaterial lacks the instanced-mesh usage flag.
	// Tallinn city to the north-west: a cluster of blocks gives a landmark on approach to 08.
	UInstancedStaticMeshComponent* City = AddInstanced(Shapes.Cube, FLinearColor(0.45f, 0.42f, 0.38f));
	for (int32 i = 0; i < 700; ++i)
	{
		const double N = 2600.0 + Random.FRandRange(-2400.0, 2400.0);
		const double E = -6000.0 + Random.FRandRange(-3500.0, 3500.0);
		if (!ClearOfRunway(N, E))
		{
			continue;
		}
		const double H = Random.FRandRange(8.0, i % 25 == 0 ? 110.0 : 35.0);
		City->AddInstance(BoxTransform(FVector(N, E, H / 2.0), Random.FRandRange(0.0, 90.0),
			FVector(Random.FRandRange(20.0, 70.0), Random.FRandRange(15.0, 50.0), H)));
	}

	// Forest patches everywhere else, for a sense of height and speed.
	UInstancedStaticMeshComponent* Trees = AddInstanced(Shapes.Cube, Forest);
	for (int32 i = 0; i < 3000; ++i)
	{
		const double N = Random.FRandRange(-25000.0, 5800.0);
		const double E = Random.FRandRange(-30000.0, 30000.0);
		if (!ClearOfRunway(N, E))
		{
			continue;
		}
		const double H = Random.FRandRange(12.0, 25.0);
		Trees->AddInstance(BoxTransform(FVector(N, E, H / 2.0), Random.FRandRange(0.0, 90.0),
			FVector(Random.FRandRange(40.0, 220.0), Random.FRandRange(40.0, 220.0), H)));
	}
}

void AA320World::UpdatePapi(int32 RunwayIndex, const int PapiWhite[4])
{
	for (int32 i = 0; i < 4; ++i)
	{
		const int32 Slot = RunwayIndex * 4 + i;
		if (!PapiLights.IsValidIndex(Slot) || PapiShown[Slot] == PapiWhite[i])
		{
			continue;
		}
		PapiShown[Slot] = PapiWhite[i];
		PapiLights[Slot]->SetMaterial(0, Glow(PapiWhite[i] ? LightWhite : LightRed));
	}
}

void AA320World::BuildAirportLayouts()
{
	const FString Dir = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Airports"));
	TArray<FString> Files;
	IFileManager::Get().FindFiles(Files, *FPaths::Combine(Dir, TEXT("*.txt")), true, false);
	if (Files.Num() == 0)
	{
		return;
	}
	// Taxiways just below the runways (so a runway crossing shows the runway), aprons below them.
	const FLinearColor TaxiAsphalt(0.06f, 0.06f, 0.065f);
	const FLinearColor TaxiYellow(0.85f, 0.65f, 0.05f);
	constexpr double TaxiTopM = 0.035, ApronTopM = 0.025, LineTopM = 0.045;
	UInstancedStaticMeshComponent* Segments = AddInstanced(Shapes.Cube, TaxiAsphalt);
	UInstancedStaticMeshComponent* Joints = AddInstanced(Shapes.Cylinder, TaxiAsphalt);
	UInstancedStaticMeshComponent* Lines = AddInstanced(Shapes.Cube, TaxiYellow);
	auto ParsePoints = [](const FString& Text)
	{
		TArray<FVector2D> Points;  // X north, Y east
		TArray<FString> Pairs;
		Text.ParseIntoArray(Pairs, TEXT(";"), true);
		for (const FString& Pair : Pairs)
		{
			FString N, E;
			if (Pair.Split(TEXT(","), &N, &E))
			{
				Points.Add(FVector2D(FCString::Atod(*N), FCString::Atod(*E)));
			}
		}
		return Points;
	};
	for (const FString& File : Files)
	{
		TArray<FString> LinesOfFile;
		if (!FFileHelper::LoadFileToStringArray(LinesOfFile, *FPaths::Combine(Dir, File)))
		{
			continue;
		}
		double Level = 0.0;
		for (const FString& Line : LinesOfFile)
		{
			FString Key, Value;
			if (Line.StartsWith(TEXT("#")) || !Line.Split(TEXT("="), &Key, &Value))
			{
				continue;
			}
			TArray<FString> Fields;
			Value.ParseIntoArray(Fields, TEXT("|"), false);
			if (Key == TEXT("level"))
			{
				Level = FCString::Atod(*Value);
			}
			else if (Key == TEXT("taxiway") && Fields.Num() >= 3)
			{
				const double WidthM = FMath::Max(FCString::Atod(*Fields[1]), 3.0);
				const TArray<FVector2D> Points = ParsePoints(Fields[2]);
				for (int32 i = 0; i < Points.Num(); ++i)
				{
					// Round joints so bends and junctions have no gaps.
					Joints->AddInstance(FTransform(FRotator::ZeroRotator,
						FVector(Points[i].X, Points[i].Y, Level + TaxiTopM - 0.1) * 100.0, FVector(WidthM, WidthM, 0.2)));
					if (i == 0)
					{
						continue;
					}
					const FVector2D A = Points[i - 1], B = Points[i];
					const double Len = FVector2D::Distance(A, B);
					if (Len < 0.1)
					{
						continue;
					}
					const double Yaw = FMath::RadiansToDegrees(FMath::Atan2(B.Y - A.Y, B.X - A.X));
					const FVector2D Mid = (A + B) / 2.0;
					Segments->AddInstance(FTransform(FRotator(0.0, Yaw, 0.0), FVector(Mid.X, Mid.Y, Level + TaxiTopM - 0.1) * 100.0,
						FVector(Len, WidthM, 0.2)));
					Lines->AddInstance(FTransform(FRotator(0.0, Yaw, 0.0), FVector(Mid.X, Mid.Y, Level + LineTopM - 0.01) * 100.0,
						FVector(Len, 0.3, 0.02)));
				}
			}
			else if (Key == TEXT("apron") && Fields.Num() >= 2)
			{
				const TArray<FVector2D> Points = ParsePoints(Fields[1]);
				std::vector<a320::geom::Vec2> Poly;  // x east, y north: the triangulation's plane
				for (const FVector2D& P : Points)
				{
					Poly.push_back({P.Y, P.X});
				}
				const std::vector<int> Tris = a320::worldmap::triangulate(Poly);
				if (Tris.empty())
				{
					continue;
				}
				TArray<FVector> Vertices;
				TArray<FVector> Normals;
				TArray<FVector2D> Uvs;
				for (const FVector2D& P : Points)
				{
					Vertices.Add(FVector(P.X, P.Y, Level + ApronTopM) * 100.0);
					Normals.Add(FVector::UpVector);
					Uvs.Add(FVector2D::ZeroVector);
				}
				TArray<int32> Triangles;
				for (size_t t = 0; t + 2 < Tris.size(); t += 3)
				{
					// Counter-clockwise seen from above (east right, north up): the face points up.
					const a320::geom::Vec2& A = Poly[static_cast<size_t>(Tris[t])];
					const a320::geom::Vec2& B = Poly[static_cast<size_t>(Tris[t + 1])];
					const a320::geom::Vec2& C = Poly[static_cast<size_t>(Tris[t + 2])];
					const bool bCcw = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x) > 0.0;
					Triangles.Append({Tris[t], bCcw ? Tris[t + 1] : Tris[t + 2], bCcw ? Tris[t + 2] : Tris[t + 1]});
				}
				UProceduralMeshComponent* Apron = NewObject<UProceduralMeshComponent>(this);
				Apron->SetupAttachment(Root);
				Apron->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				Apron->SetCastShadow(false);
				Apron->CreateMeshSection(0, Vertices, Triangles, Normals, Uvs, TArray<FColor>(), TArray<FProcMeshTangent>(), false);
				Apron->SetMaterial(0, Shapes.Tint(this, Concrete));
				Apron->RegisterComponent();
			}
		}
	}
}
