#include "A320Terrain.h"

#include "A320Shapes.h"
#include "A320Sim.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "GameFramework/Actor.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionConstant.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "ProceduralMeshComponent.h"
#include "TextureResource.h"
#if __has_include("Engine/Texture2DMipMap.h")
#include "Engine/Texture2DMipMap.h"
#endif
#include "UObject/Package.h"

namespace
{
	struct FTileSpec
	{
		FString Name;
		double SouthM = 0.0, WestM = 0.0, SizeM = 0.0, HoleM = 0.0;
		int32 Grid = 0;
	};

	// Skirts hang below every detailed tile's edges, hiding cracks where it meets a coarser one.
	constexpr double SkirtM = 30.0;
	// Buildings fade out beyond this; a 10 m roof is a pixel from further away anyway.
	constexpr int32 BuildingCullStartCm = 1200000;
	constexpr int32 BuildingCullEndCm = 1600000;

	TArray<uint8> Downsample(const TArray<uint8>& Src, int32 W, int32 H, int32& OutW, int32& OutH)
	{
		OutW = FMath::Max(W / 2, 1);
		OutH = FMath::Max(H / 2, 1);
		TArray<uint8> Dst;
		Dst.SetNumUninitialized(OutW * OutH * 4);
		for (int32 Y = 0; Y < OutH; ++Y)
		{
			const int32 Y0 = FMath::Min(Y * 2, H - 1), Y1 = FMath::Min(Y * 2 + 1, H - 1);
			for (int32 X = 0; X < OutW; ++X)
			{
				const int32 X0 = FMath::Min(X * 2, W - 1), X1 = FMath::Min(X * 2 + 1, W - 1);
				for (int32 C = 0; C < 4; ++C)
				{
					const int32 Sum = Src[(Y0 * W + X0) * 4 + C] + Src[(Y0 * W + X1) * 4 + C] +
						Src[(Y1 * W + X0) * 4 + C] + Src[(Y1 * W + X1) * 4 + C];
					Dst[(Y * OutW + X) * 4 + C] = static_cast<uint8>((Sum + 2) / 4);
				}
			}
		}
		return Dst;
	}

	// A JPEG as a texture with a full mip chain: without mips, distant ground shimmers.
	UTexture2D* LoadTexture(const FString& Path)
	{
		TArray<uint8> Compressed;
		if (!FFileHelper::LoadFileToArray(Compressed, *Path))
		{
			return nullptr;
		}
		IImageWrapperModule& Wrappers = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		const TSharedPtr<IImageWrapper> Wrapper = Wrappers.CreateImageWrapper(EImageFormat::JPEG);
		TArray64<uint8> Raw;
		if (!Wrapper.IsValid() || !Wrapper->SetCompressed(Compressed.GetData(), Compressed.Num()) ||
			!Wrapper->GetRaw(ERGBFormat::BGRA, 8, Raw) || Raw.Num() > MAX_int32)
		{
			return nullptr;
		}
		TArray<uint8> Level(Raw.GetData(), static_cast<int32>(Raw.Num()));
		Raw.Empty();
		int32 W = static_cast<int32>(Wrapper->GetWidth());
		int32 H = static_cast<int32>(Wrapper->GetHeight());
		UTexture2D* Texture = UTexture2D::CreateTransient(W, H, PF_B8G8R8A8);
		if (!Texture || Level.Num() != W * H * 4)
		{
			return nullptr;
		}
		Texture->SRGB = true;
		Texture->AddressX = TA_Clamp;
		Texture->AddressY = TA_Clamp;
		Texture->LODGroup = TEXTUREGROUP_World;
		Texture->NeverStream = true;  // the mips exist only in memory; nothing to stream them back from
		FTexturePlatformData* Platform = Texture->GetPlatformData();
		for (int32 MipIndex = 0;; ++MipIndex)
		{
			FTexture2DMipMap* Mip = MipIndex == 0 ? &Platform->Mips[0] : new FTexture2DMipMap(W, H, 1);
			if (MipIndex > 0)
			{
				Platform->Mips.Add(Mip);
			}
			Mip->BulkData.Lock(LOCK_READ_WRITE);
			void* Dest = Mip->BulkData.Realloc(Level.Num());
			FMemory::Memcpy(Dest, Level.GetData(), Level.Num());
			Mip->BulkData.Unlock();
			if (W == 1 && H == 1)
			{
				break;
			}
			int32 NextW = 0, NextH = 0;
			Level = Downsample(Level, W, H, NextW, NextH);
			W = NextW;
			H = NextH;
		}
		Texture->UpdateResource();
		return Texture;
	}

	// A lit, two-sided material with one texture parameter, built in memory. Only possible where
	// the material compiler exists (Play.bat runs the editor build); cooked builds use the fallback.
	UMaterialInterface* MakeLitMaterial(UTexture2D* DefaultTexture, FName& OutParameter)
	{
#if WITH_EDITOR
		UMaterial* Material = NewObject<UMaterial>(GetTransientPackage(),
			MakeUniqueObjectName(GetTransientPackage(), UMaterial::StaticClass(), TEXT("M_A320Terrain")), RF_Transient);
		UMaterialExpressionTextureSampleParameter2D* Sample = NewObject<UMaterialExpressionTextureSampleParameter2D>(Material);
		Sample->ParameterName = TEXT("Albedo");
		Sample->Texture = DefaultTexture;
		Sample->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(Material);
		Roughness->R = 0.92f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(Material);
		Specular->R = 0.2f;
		Material->GetExpressionCollection().AddExpression(Sample);
		Material->GetExpressionCollection().AddExpression(Roughness);
		Material->GetExpressionCollection().AddExpression(Specular);
		Sample->Material = Material;
		Roughness->Material = Material;
		Specular->Material = Material;
		UMaterialEditorOnlyData* Inputs = Material->GetEditorOnlyData();
		Inputs->BaseColor.Connect(0, Sample);
		Inputs->Roughness.Connect(0, Roughness);
		Inputs->Specular.Connect(0, Specular);
		Material->TwoSided = true;
		Material->PostEditChange();
		OutParameter = TEXT("Albedo");
		return Material;
#else
		(void)DefaultTexture;
		OutParameter = NAME_None;
		return nullptr;
#endif
	}

	bool LoadHeights(const FString& Path, int32 Grid, TArray<float>& Out)
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, *Path) || Bytes.Num() != Grid * Grid * 4)
		{
			return false;
		}
		Out.SetNumUninitialized(Grid * Grid);
		FMemory::Memcpy(Out.GetData(), Bytes.GetData(), Bytes.Num());  // little-endian float32, as on PC
		return true;
	}

	UProceduralMeshComponent* BuildTileMesh(AActor* Owner, USceneComponent* Parent, const FTileSpec& Tile,
		const TArray<float>& Heights)
	{
		const int32 G = Tile.Grid;
		const double Step = Tile.SizeM / (G - 1);
		const double CentreN = Tile.SouthM + Tile.SizeM / 2.0, CentreE = Tile.WestM + Tile.SizeM / 2.0;
		auto HeightAt = [&](int32 Row, int32 Col) { return Heights[FMath::Clamp(Row, 0, G - 1) * G + FMath::Clamp(Col, 0, G - 1)]; };

		TArray<FVector> Vertices;
		TArray<FVector> Normals;
		TArray<FVector2D> Uvs;
		TArray<FProcMeshTangent> Tangents;
		TArray<int32> Triangles;
		Vertices.Reserve(G * G + 4 * G);
		Normals.Reserve(G * G + 4 * G);
		Uvs.Reserve(G * G + 4 * G);
		Tangents.Reserve(G * G + 4 * G);
		for (int32 Row = 0; Row < G; ++Row)
		{
			for (int32 Col = 0; Col < G; ++Col)
			{
				const double N = Row * Step - Tile.SizeM / 2.0, E = Col * Step - Tile.SizeM / 2.0;
				Vertices.Add(FVector(N, E, HeightAt(Row, Col)) * 100.0);
				const double DhDn = (HeightAt(Row + 1, Col) - HeightAt(Row - 1, Col)) / (2.0 * Step);
				const double DhDe = (HeightAt(Row, Col + 1) - HeightAt(Row, Col - 1)) / (2.0 * Step);
				Normals.Add(FVector(-DhDn, -DhDe, 1.0).GetSafeNormal());
				// Image rows run north to south, columns west to east.
				Uvs.Add(FVector2D(static_cast<double>(Col) / (G - 1), 1.0 - static_cast<double>(Row) / (G - 1)));
				Tangents.Add(FProcMeshTangent(FVector(0.0, 1.0, DhDe).GetSafeNormal(), false));
			}
		}
		const bool bHasHole = Tile.HoleM > 0.0;
		for (int32 Row = 0; Row + 1 < G; ++Row)
		{
			for (int32 Col = 0; Col + 1 < G; ++Col)
			{
				const double CellN = Tile.SouthM + (Row + 0.5) * Step, CellE = Tile.WestM + (Col + 0.5) * Step;
				if (bHasHole && FMath::Abs(CellN) < Tile.HoleM && FMath::Abs(CellE) < Tile.HoleM)
				{
					continue;
				}
				const int32 A = Row * G + Col, B = A + 1, C = A + G, D = C + 1;
				// Counter-clockwise seen from above (north up, east right): Unreal's front face points up.
				Triangles.Append({A, D, C, A, B, D});
			}
		}
		if (!bHasHole)
		{
			// Edge loop from the south-west corner (east, north, west, south), each vertex copied below.
			TArray<int32> Edge;
			for (int32 Col = 0; Col < G; ++Col) Edge.Add(Col);
			for (int32 Row = 1; Row < G; ++Row) Edge.Add(Row * G + G - 1);
			for (int32 Col = G - 2; Col >= 0; --Col) Edge.Add((G - 1) * G + Col);
			for (int32 Row = G - 2; Row >= 0; --Row) Edge.Add(Row * G);
			const int32 First = Vertices.Num();
			for (const int32 Top : Edge)
			{
				// Copies first: TArray::Add rejects a reference into the array being grown.
				const FVector Below = Vertices[Top] - FVector(0.0, 0.0, SkirtM * 100.0);
				const FVector Normal = Normals[Top];
				const FVector2D Uv = Uvs[Top];
				const FProcMeshTangent Tangent = Tangents[Top];
				Vertices.Add(Below);
				Normals.Add(Normal);
				Uvs.Add(Uv);
				Tangents.Add(Tangent);
			}
			for (int32 i = 0; i + 1 < Edge.Num(); ++i)
			{
				Triangles.Append({Edge[i], First + i + 1, Edge[i + 1], Edge[i], First + i, First + i + 1});  // facing out
			}
		}

		UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(Owner);
		Mesh->SetupAttachment(Parent);
		Mesh->SetRelativeLocation(FVector(CentreN, CentreE, 0.0) * 100.0);
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Mesh->SetCastShadow(false);  // gentle relief: not worth the shadow cost over 300 km
		Mesh->CreateMeshSection(0, Vertices, Triangles, Normals, Uvs, TArray<FColor>(), Tangents, false);
		Mesh->RegisterComponent();
		return Mesh;
	}

	bool ParseTile(const FString& Value, FTileSpec& Out)
	{
		TArray<FString> F;
		Value.ParseIntoArray(F, TEXT("|"), false);
		if (F.Num() < 6)
		{
			return false;
		}
		Out.Name = F[0];
		Out.SouthM = FCString::Atod(*F[1]);
		Out.WestM = FCString::Atod(*F[2]);
		Out.SizeM = FCString::Atod(*F[3]);
		Out.Grid = FCString::Atoi(*F[4]);
		Out.HoleM = FCString::Atod(*F[5]);
		return Out.Grid >= 2 && Out.SizeM > 0.0 && !Out.Name.Contains(TEXT("/")) && !Out.Name.Contains(TEXT("\\"));
	}

	int32 BuildBuildings(AActor* Owner, USceneComponent* Parent, FA320Shapes& Shapes, const FString& Path)
	{
		TArray<uint8> Bytes;
		constexpr int32 Stride = 7 * sizeof(float);
		if (!FFileHelper::LoadFileToArray(Bytes, *Path) || Bytes.Num() % Stride != 0)
		{
			return 0;
		}
		// A few roof/wall tones; the satellite image already shows the real roofs from above.
		const FLinearColor Tones[] = {FLinearColor(0.55f, 0.53f, 0.50f), FLinearColor(0.42f, 0.41f, 0.40f),
			FLinearColor(0.62f, 0.57f, 0.49f), FLinearColor(0.35f, 0.33f, 0.33f)};
		constexpr int32 ToneCount = UE_ARRAY_COUNT(Tones);
		TArray<FTransform> Boxes[ToneCount];
		const int32 Count = Bytes.Num() / Stride;
		for (int32 i = 0; i < Count; ++i)
		{
			float V[7];
			FMemory::Memcpy(V, Bytes.GetData() + i * Stride, Stride);
			const float North = V[0], East = V[1], Ground = V[2], Length = V[3], Width = V[4], Yaw = V[5], Height = V[6];
			// Sunk 1 m so slopes don't show a gap under the box.
			const FVector Centre(North, East, Ground - 1.0f + (Height + 1.0f) / 2.0f);
			Boxes[i % ToneCount].Add(FTransform(FRotator(0.0, Yaw, 0.0), Centre * 100.0,
				FVector(Length, Width, Height + 1.0f)));
		}
		for (int32 t = 0; t < ToneCount; ++t)
		{
			UInstancedStaticMeshComponent* Ism = NewObject<UInstancedStaticMeshComponent>(Owner);
			Ism->SetMobility(EComponentMobility::Static);
			Ism->SetStaticMesh(Shapes.Cube);
			Ism->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Ism->SetupAttachment(Parent);
			Ism->SetMaterial(0, Shapes.Tint(Owner, Tones[t]));
			Ism->SetCullDistances(BuildingCullStartCm, BuildingCullEndCm);
			Ism->RegisterComponent();
			Ism->AddInstances(Boxes[t], false);
		}
		return Count;
	}
}

FA320TerrainResult A320Terrain::Build(AActor* Owner, USceneComponent* Parent, FA320Shapes& Shapes,
	UMaterialInterface* UnlitTextureMaterial)
{
	FA320TerrainResult Result;
	const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Terrain")));
	TArray<FString> Lines;
	if (!FFileHelper::LoadFileToStringArray(Lines, *FPaths::Combine(Dir, TEXT("terrain.txt"))))
	{
		UE_LOG(LogA320, Log, TEXT("No terrain data in %s (run tools/make_terrain.py): flat scenery"), *Dir);
		return Result;
	}

	TArray<FTileSpec> Tiles;
	FString BuildingsFile;
	for (const FString& Line : Lines)
	{
		FString Key, Value;
		if (Line.StartsWith(TEXT("#")) || !Line.Split(TEXT("="), &Key, &Value))
		{
			continue;
		}
		FTileSpec Tile;
		if (Key == TEXT("tile") && ParseTile(Value, Tile))
		{
			Tiles.Add(Tile);
		}
		else if (Key == TEXT("buildings") && !Value.Contains(TEXT("/")) && !Value.Contains(TEXT("\\")))
		{
			BuildingsFile = Value;
		}
	}

	UMaterialInterface* Material = nullptr;
	FName Parameter = NAME_None;
	for (const FTileSpec& Tile : Tiles)
	{
		TArray<float> Heights;
		if (!LoadHeights(FPaths::Combine(Dir, Tile.Name + TEXT(".f32")), Tile.Grid, Heights))
		{
			UE_LOG(LogA320, Warning, TEXT("Terrain tile %s: missing or wrong-sized heights"), *Tile.Name);
			continue;
		}
		UTexture2D* Texture = LoadTexture(FPaths::Combine(Dir, Tile.Name + TEXT(".jpg")));
		if (!Texture)
		{
			UE_LOG(LogA320, Warning, TEXT("Terrain tile %s: image could not be read"), *Tile.Name);
			continue;
		}
		if (!Material)
		{
			Material = MakeLitMaterial(Texture, Parameter);
			if (!Material)
			{
				Material = UnlitTextureMaterial;
				Parameter = TEXT("SlateUI");
			}
			if (!Material)
			{
				return Result;
			}
		}
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Material, Owner);
		Mid->SetTextureParameterValue(Parameter, Texture);
		UProceduralMeshComponent* Mesh = BuildTileMesh(Owner, Parent, Tile, Heights);
		Mesh->SetMaterial(0, Mid);
		++Result.Tiles;
	}
	Result.bLoaded = Result.Tiles > 0;
	if (Result.bLoaded && !BuildingsFile.IsEmpty())
	{
		Result.Buildings = BuildBuildings(Owner, Parent, Shapes, FPaths::Combine(Dir, BuildingsFile));
	}
	UE_LOG(LogA320, Log, TEXT("Terrain: %d tiles, %d buildings from %s"), Result.Tiles, Result.Buildings, *Dir);
	return Result;
}
