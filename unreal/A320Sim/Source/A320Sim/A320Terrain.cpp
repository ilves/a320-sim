#include "A320Terrain.h"

#include "A320Shapes.h"
#include "A320Sim.h"

#include "Async/Async.h"
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
#include "UObject/StrongObjectPtr.h"

namespace
{
	struct FTileSpec
	{
		FString Name;
		FString Path;  // without the extension
		double SouthM = 0.0, WestM = 0.0, SizeM = 0.0, HoleM = 0.0;
		int32 Grid = 0;
	};

	// Skirts hang below every tile's edges, hiding cracks where it meets a coarser one.
	constexpr double SkirtM = 30.0;
	// Buildings fade out beyond this; a 10 m roof is a pixel from further away anyway.
	constexpr int32 BuildingCullStartCm = 1200000;
	constexpr int32 BuildingCullEndCm = 1600000;

	// Streaming radii, horizontal from the aircraft (m): tiles by their centre, building chunks by
	// their nearest edge. The gap between loading and unloading stops a piece on the edge flickering.
	constexpr double DetailLoadM = 20000.0, DetailUnloadM = 28000.0;
	constexpr double RegionLoadM = 45000.0, RegionUnloadM = 60000.0;
	constexpr double BuildingsLoadM = 15000.0, BuildingsUnloadM = 20000.0;
	constexpr double BuildingChunkM = 10000.0;
	// Game-thread work per frame: each creation uploads a texture and builds a mesh.
	constexpr int32 MaxCreatesPerTick = 2;
	constexpr int32 MaxUnloadsPerTick = 8;
	constexpr int32 MaxJobsInFlight = 4;
	// A jump this far (start-up, a scenario reset) loads the tiles under the aircraft at once,
	// rather than showing the coarse base layer there for the first frames.
	constexpr double TeleportM = 5000.0;
	constexpr double PrimeReachM = 2000.0;

	// A few roof/wall tones; the satellite image already shows the real roofs from above.
	const FLinearColor BuildingTones[] = {FLinearColor(0.55f, 0.53f, 0.50f), FLinearColor(0.42f, 0.41f, 0.40f),
		FLinearColor(0.62f, 0.57f, 0.49f), FLinearColor(0.35f, 0.33f, 0.33f)};
	constexpr int32 ToneCount = UE_ARRAY_COUNT(BuildingTones);
	constexpr int32 BuildingFloats = 7;

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

	struct FDecodedImage
	{
		int32 Width = 0, Height = 0;
		TArray<TArray<uint8>> Mips;  // BGRA, down to 1x1: without mips, distant ground shimmers
	};

	// Any thread: decoding and the mip chain are most of a tile's cost.
	bool DecodeImage(IImageWrapperModule& Wrappers, const FString& Path, FDecodedImage& Out)
	{
		TArray<uint8> Compressed;
		if (!FFileHelper::LoadFileToArray(Compressed, *Path))
		{
			return false;
		}
		const TSharedPtr<IImageWrapper> Wrapper = Wrappers.CreateImageWrapper(EImageFormat::JPEG);
		TArray64<uint8> Raw;
		if (!Wrapper.IsValid() || !Wrapper->SetCompressed(Compressed.GetData(), Compressed.Num()) ||
			!Wrapper->GetRaw(ERGBFormat::BGRA, 8, Raw) || Raw.Num() > MAX_int32)
		{
			return false;
		}
		int32 W = static_cast<int32>(Wrapper->GetWidth());
		int32 H = static_cast<int32>(Wrapper->GetHeight());
		if (W <= 0 || H <= 0 || Raw.Num() != static_cast<int64>(W) * H * 4)
		{
			return false;
		}
		Out.Width = W;
		Out.Height = H;
		Out.Mips.Reset();
		Out.Mips.Emplace(Raw.GetData(), static_cast<int32>(Raw.Num()));
		Raw.Empty();
		while (W > 1 || H > 1)
		{
			int32 NextW = 0, NextH = 0;
			TArray<uint8> Next = Downsample(Out.Mips.Last(), W, H, NextW, NextH);
			Out.Mips.Add(MoveTemp(Next));
			W = NextW;
			H = NextH;
		}
		return true;
	}

	// Game thread.
	UTexture2D* CreateTexture(const FDecodedImage& Image)
	{
		if (Image.Mips.Num() == 0)
		{
			return nullptr;
		}
		UTexture2D* Texture = UTexture2D::CreateTransient(Image.Width, Image.Height, PF_B8G8R8A8);
		if (!Texture || Image.Mips[0].Num() != Image.Width * Image.Height * 4)
		{
			return nullptr;
		}
		Texture->SRGB = true;
		Texture->AddressX = TA_Clamp;
		Texture->AddressY = TA_Clamp;
		Texture->LODGroup = TEXTUREGROUP_World;
		Texture->NeverStream = true;  // the mips exist only in memory; nothing to stream them back from
		FTexturePlatformData* Platform = Texture->GetPlatformData();
		int32 W = Image.Width, H = Image.Height;
		for (int32 MipIndex = 0; MipIndex < Image.Mips.Num(); ++MipIndex)
		{
			FTexture2DMipMap* Mip = MipIndex == 0 ? &Platform->Mips[0] : new FTexture2DMipMap(W, H, 1);
			if (MipIndex > 0)
			{
				Platform->Mips.Add(Mip);
			}
			const TArray<uint8>& Level = Image.Mips[MipIndex];
			Mip->BulkData.Lock(LOCK_READ_WRITE);
			void* Dest = Mip->BulkData.Realloc(Level.Num());
			FMemory::Memcpy(Dest, Level.GetData(), Level.Num());
			Mip->BulkData.Unlock();
			W = FMath::Max(W / 2, 1);
			H = FMath::Max(H / 2, 1);
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

	struct FTileGeometry
	{
		FVector CentreCm = FVector::ZeroVector;
		TArray<FVector> Vertices;
		TArray<FVector> Normals;
		TArray<FVector2D> Uvs;
		TArray<FProcMeshTangent> Tangents;
		TArray<int32> Triangles;
	};

	// Any thread.
	void BuildTileGeometry(const FTileSpec& Tile, const TArray<float>& Heights, FTileGeometry& Out)
	{
		const int32 G = Tile.Grid;
		const double Step = Tile.SizeM / (G - 1);
		Out.CentreCm = FVector(Tile.SouthM + Tile.SizeM / 2.0, Tile.WestM + Tile.SizeM / 2.0, 0.0) * 100.0;
		auto HeightAt = [&](int32 Row, int32 Col) { return Heights[FMath::Clamp(Row, 0, G - 1) * G + FMath::Clamp(Col, 0, G - 1)]; };

		TArray<FVector>& Vertices = Out.Vertices;
		TArray<FVector>& Normals = Out.Normals;
		TArray<FVector2D>& Uvs = Out.Uvs;
		TArray<FProcMeshTangent>& Tangents = Out.Tangents;
		TArray<int32>& Triangles = Out.Triangles;
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
	}

	struct FTileData
	{
		FDecodedImage Image;
		FTileGeometry Geometry;
	};

	// Any thread.
	bool LoadTileData(IImageWrapperModule& Wrappers, const FTileSpec& Tile, FTileData& Out, FString& OutError)
	{
		TArray<float> Heights;
		if (!LoadHeights(Tile.Path + TEXT(".f32"), Tile.Grid, Heights))
		{
			OutError = TEXT("missing or wrong-sized heights");
			return false;
		}
		if (!DecodeImage(Wrappers, Tile.Path + TEXT(".jpg"), Out.Image))
		{
			OutError = TEXT("image could not be read");
			return false;
		}
		BuildTileGeometry(Tile, Heights, Out.Geometry);
		return true;
	}

	bool ParseTile(const FString& Value, const FString& Dir, FTileSpec& Out)
	{
		TArray<FString> F;
		Value.ParseIntoArray(F, TEXT("|"), false);
		if (F.Num() < 6)
		{
			return false;
		}
		Out.Name = F[0];
		Out.Path = FPaths::Combine(Dir, Out.Name);
		Out.SouthM = FCString::Atod(*F[1]);
		Out.WestM = FCString::Atod(*F[2]);
		Out.SizeM = FCString::Atod(*F[3]);
		Out.Grid = FCString::Atoi(*F[4]);
		Out.HoleM = FCString::Atod(*F[5]);
		return Out.Grid >= 2 && Out.SizeM > 0.0 && !Out.Name.Contains(TEXT("/")) && !Out.Name.Contains(TEXT("\\"));
	}

	// Horizontal distance from a point (north, east; m) to the nearest point of a tile's square.
	double DistanceToSquareM(const FTileSpec& Tile, const FVector2D& Point)
	{
		const double DN = FMath::Max3(Tile.SouthM - Point.X, 0.0, Point.X - (Tile.SouthM + Tile.SizeM));
		const double DE = FMath::Max3(Tile.WestM - Point.Y, 0.0, Point.Y - (Tile.WestM + Tile.SizeM));
		return FMath::Sqrt(DN * DN + DE * DE);
	}

	enum class ELayer : uint8
	{
		Detail,
		Region,
		Buildings
	};

	// OpenStreetMap boxes in one square of the building grid, as in buildings.bin.
	struct FBuildingChunk
	{
		TArray<float> Boxes;  // BuildingFloats per box: north, east, ground, length, width, yaw, height
		TArray<uint8> Tones;
	};

	// Any thread.
	void BuildingTransforms(const FBuildingChunk& Chunk, TArray<FTransform> (&Out)[ToneCount])
	{
		for (int32 i = 0; i < Chunk.Tones.Num(); ++i)
		{
			const float* V = Chunk.Boxes.GetData() + i * BuildingFloats;
			const float North = V[0], East = V[1], Ground = V[2], Length = V[3], Width = V[4], Yaw = V[5], Height = V[6];
			// Sunk 1 m so slopes don't show a gap under the box.
			const FVector Centre(North, East, Ground - 1.0f + (Height + 1.0f) / 2.0f);
			Out[Chunk.Tones[i]].Add(FTransform(FRotator(0.0, Yaw, 0.0), Centre * 100.0, FVector(Length, Width, Height + 1.0f)));
		}
	}

	// What a worker thread hands back: only plain data, never UObjects.
	struct FJob
	{
		FTileData Tile;
		TArray<FTransform> Boxes[ToneCount];
		bool bOk = false;
		FString Error;
	};

	struct FItem
	{
		ELayer Layer = ELayer::Region;
		FTileSpec Spec;  // for buildings, the chunk's square
		TSharedPtr<FBuildingChunk, ESPMode::ThreadSafe> Chunk;
		int32 Twin = INDEX_NONE;  // the region tile under a detailed one, and the other way round
		TWeakObjectPtr<UProceduralMeshComponent> Mesh;
		TArray<TWeakObjectPtr<UInstancedStaticMeshComponent>> Instances;
		TSharedPtr<FJob, ESPMode::ThreadSafe> Job;
		TFuture<void> Pending;
		bool bResident = false;
		bool bFailed = false;  // not retried every frame
	};

	// Components are created on the owner, which keeps them (and through them the dynamic materials
	// and textures) reachable for the garbage collector; unloading destroys the component.
	class FStreamer final : public FA320TerrainStreamer
	{
	public:
		FStreamer(AActor* InOwner, USceneComponent* InParent, FA320Shapes& InShapes, UMaterialInterface* InUnlit)
			: Owner(InOwner), Parent(InParent), Shapes(InShapes), Unlit(InUnlit),
			  Wrappers(&FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper")))
		{
		}

		virtual ~FStreamer() override
		{
			Shutdown();
		}

		bool LoadFixed(const FTileSpec& Tile)
		{
			FTileData Data;
			FString Error;
			if (!LoadTileData(*Wrappers, Tile, Data, Error) || !CreateTile(Data, true))
			{
				UE_LOG(LogA320, Warning, TEXT("Terrain tile %s: %s"), *Tile.Name, Error.IsEmpty() ? TEXT("no material") : *Error);
				return false;
			}
			return true;
		}

		void AddTile(ELayer Layer, const FTileSpec& Tile)
		{
			FItem Item;
			Item.Layer = Layer;
			Item.Spec = Tile;
			Items.Add(MoveTemp(Item));
		}

		// Pairs each detailed tile with the region tile of the same square, so one can stand in for the other.
		void LinkTwins()
		{
			auto SquareKey = [](const FTileSpec& Tile)
			{
				return FIntVector(FMath::RoundToInt(Tile.SouthM), FMath::RoundToInt(Tile.WestM), FMath::RoundToInt(Tile.SizeM));
			};
			TMap<FIntVector, int32> Regions;
			for (int32 i = 0; i < Items.Num(); ++i)
			{
				if (Items[i].Layer == ELayer::Region)
				{
					Regions.Add(SquareKey(Items[i].Spec), i);
				}
			}
			for (int32 i = 0; i < Items.Num(); ++i)
			{
				const int32* Found = Items[i].Layer == ELayer::Detail ? Regions.Find(SquareKey(Items[i].Spec)) : nullptr;
				if (Found)
				{
					Items[i].Twin = *Found;
					Items[*Found].Twin = i;
				}
			}
		}

		// Buckets the boxes into chunks of the building grid; returns how many there are.
		int32 AddBuildings(const FString& Path)
		{
			TArray<uint8> Bytes;
			constexpr int32 Stride = BuildingFloats * sizeof(float);
			if (!FFileHelper::LoadFileToArray(Bytes, *Path) || Bytes.Num() % Stride != 0)
			{
				return 0;
			}
			TMap<FIntPoint, int32> Chunks;
			const int32 Count = Bytes.Num() / Stride;
			for (int32 i = 0; i < Count; ++i)
			{
				float V[BuildingFloats];
				FMemory::Memcpy(V, Bytes.GetData() + i * Stride, Stride);
				const FIntPoint Key(FMath::FloorToInt(V[0] / BuildingChunkM), FMath::FloorToInt(V[1] / BuildingChunkM));
				const int32* Found = Chunks.Find(Key);
				int32 Index = Found ? *Found : INDEX_NONE;
				if (Index == INDEX_NONE)
				{
					FItem Item;
					Item.Layer = ELayer::Buildings;
					Item.Spec.Name = FString::Printf(TEXT("buildings %d,%d"), Key.X, Key.Y);
					Item.Spec.SouthM = Key.X * BuildingChunkM;
					Item.Spec.WestM = Key.Y * BuildingChunkM;
					Item.Spec.SizeM = BuildingChunkM;
					Item.Chunk = MakeShared<FBuildingChunk, ESPMode::ThreadSafe>();
					Index = Items.Add(MoveTemp(Item));
					Chunks.Add(Key, Index);
				}
				FBuildingChunk& Chunk = *Items[Index].Chunk;
				Chunk.Boxes.Append(V, BuildingFloats);
				Chunk.Tones.Add(static_cast<uint8>(i % ToneCount));  // as before streaming: same tone per building
			}
			return Count;
		}

		int32 Num(ELayer Layer) const
		{
			int32 N = 0;
			for (const FItem& Item : Items)
			{
				N += Item.Layer == Layer ? 1 : 0;
			}
			return N;
		}

		virtual void Tick(const FVector& ViewerCm) override
		{
			if (!Owner.IsValid() || !Parent.IsValid())
			{
				return;
			}
			const FVector2D Viewer(ViewerCm.X / 100.0, ViewerCm.Y / 100.0);  // north, east (m)
			if (!bHasViewer || FVector2D::Distance(Viewer, LastViewer) > TeleportM)
			{
				Prime(Viewer);
			}
			bHasViewer = true;
			LastViewer = Viewer;

			// Finished background work: create it, or drop it if the aircraft has moved away meanwhile.
			int32 Created = 0;
			for (FItem& Item : Items)
			{
				if (!Item.Pending.IsValid() || !Item.Pending.IsReady())
				{
					continue;
				}
				const bool bStillWanted = DistanceM(Item, Viewer) <= UnloadRadiusM(Item.Layer);
				if (bStillWanted && Created >= MaxCreatesPerTick)
				{
					continue;  // next frame
				}
				Item.Pending = TFuture<void>();
				--InFlight;
				const TSharedPtr<FJob, ESPMode::ThreadSafe> Job = MoveTemp(Item.Job);
				Item.Job.Reset();
				if (!bStillWanted)
				{
					continue;
				}
				if (!Job->bOk)
				{
					UE_LOG(LogA320, Warning, TEXT("Terrain %s: %s"), *Item.Spec.Name, *Job->Error);
					Item.bFailed = true;
					continue;
				}
				Create(Item, *Job);
				// A building chunk (thousands of instances) costs about as much as two tiles.
				Created += Item.Layer == ELayer::Buildings ? 2 : 1;
			}

			// Far pieces go; a detailed tile only once the region tile replacing it is in, so there's no hole.
			int32 Unloaded = 0;
			for (FItem& Item : Items)
			{
				if (Unloaded >= MaxUnloadsPerTick)
				{
					break;
				}
				if (!Item.bResident || DistanceM(Item, Viewer) <= UnloadRadiusM(Item.Layer))
				{
					continue;
				}
				if (Item.Layer == ELayer::Detail && Item.Twin != INDEX_NONE)
				{
					const FItem& Region = Items[Item.Twin];
					if (!Region.bResident && !Region.bFailed && DistanceM(Region, Viewer) < LoadRadiusM(Region.Layer))
					{
						continue;
					}
				}
				Unload(Item);
				++Unloaded;
			}

			// The nearest missing pieces start next.
			if (InFlight >= MaxJobsInFlight)
			{
				return;
			}
			Wanted.Reset();
			for (int32 i = 0; i < Items.Num(); ++i)
			{
				const FItem& Item = Items[i];
				if (Item.bResident || Item.bFailed || Item.Pending.IsValid())
				{
					continue;
				}
				const double Distance = DistanceM(Item, Viewer);
				if (Distance < LoadRadiusM(Item.Layer) && !CoveredByDetail(Item, Distance))
				{
					Wanted.Emplace(Distance, i);
				}
			}
			// Equal distances: a detailed tile before the region tile of the same square, which it makes unneeded.
			Wanted.Sort([this](const TPair<double, int32>& L, const TPair<double, int32>& R)
			{
				return L.Key < R.Key || (L.Key == R.Key && Items[L.Value].Layer < Items[R.Value].Layer);
			});
			for (const TPair<double, int32>& Next : Wanted)
			{
				if (InFlight >= MaxJobsInFlight)
				{
					break;
				}
				FItem& Item = Items[Next.Value];
				if (!CoveredByDetail(Item, Next.Key))
				{
					Start(Item);
				}
			}
		}

		virtual void Shutdown() override
		{
			for (FItem& Item : Items)
			{
				if (Item.Pending.IsValid())
				{
					Item.Pending.Wait();
					Item.Pending = TFuture<void>();
				}
				Item.Job.Reset();
			}
			InFlight = 0;
		}

	private:
		static double LoadRadiusM(ELayer Layer)
		{
			return Layer == ELayer::Detail ? DetailLoadM : Layer == ELayer::Region ? RegionLoadM : BuildingsLoadM;
		}

		static double UnloadRadiusM(ELayer Layer)
		{
			return Layer == ELayer::Detail ? DetailUnloadM : Layer == ELayer::Region ? RegionUnloadM : BuildingsUnloadM;
		}

		// A region tile isn't loaded while the detailed tile of its square is in or on its way, and
		// still wanted; it is needed only once that one leaves (which waits for it, so there's no hole).
		bool CoveredByDetail(const FItem& Item, double Distance) const
		{
			if (Item.Layer != ELayer::Region || Item.Twin == INDEX_NONE || Distance >= DetailLoadM)
			{
				return false;
			}
			const FItem& Detail = Items[Item.Twin];
			return Detail.bResident || Detail.Pending.IsValid();
		}

		static double DistanceM(const FItem& Item, const FVector2D& Viewer)
		{
			const FTileSpec& Tile = Item.Spec;
			if (Item.Layer == ELayer::Buildings)
			{
				return DistanceToSquareM(Tile, Viewer);
			}
			return FVector2D::Distance(Viewer, FVector2D(Tile.SouthM + Tile.SizeM / 2.0, Tile.WestM + Tile.SizeM / 2.0));
		}

		UMaterialInterface* MaterialFor(UTexture2D* Texture)
		{
			if (!Material.IsValid())
			{
				UMaterialInterface* Base = MakeLitMaterial(Texture, Parameter);
				if (!Base)
				{
					Base = Unlit;
					Parameter = TEXT("SlateUI");
				}
				if (Base)
				{
					Material.Reset(Base);
				}
			}
			return Material.Get();
		}

		UProceduralMeshComponent* CreateTile(const FTileData& Data, bool bVisible)
		{
			UTexture2D* Texture = CreateTexture(Data.Image);
			UMaterialInterface* Base = Texture ? MaterialFor(Texture) : nullptr;
			if (!Base)
			{
				return nullptr;
			}
			AActor* OwnerActor = Owner.Get();
			UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, OwnerActor);
			Mid->SetTextureParameterValue(Parameter, Texture);
			const FTileGeometry& Geometry = Data.Geometry;
			UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(OwnerActor);
			Mesh->SetupAttachment(Parent.Get());
			Mesh->SetRelativeLocation(Geometry.CentreCm);
			Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Mesh->SetCastShadow(false);  // gentle relief: not worth the shadow cost over hundreds of km
			// The section's bounds come from its vertices, skirts included, so frustum culling is per tile.
			Mesh->CreateMeshSection(0, Geometry.Vertices, Geometry.Triangles, Geometry.Normals, Geometry.Uvs,
				TArray<FColor>(), Geometry.Tangents, false);
			Mesh->SetMaterial(0, Mid);
			Mesh->SetVisibility(bVisible);
			Mesh->RegisterComponent();
			return Mesh;
		}

		void Start(FItem& Item)
		{
			const TSharedPtr<FJob, ESPMode::ThreadSafe> Job = MakeShared<FJob, ESPMode::ThreadSafe>();
			Item.Job = Job;
			// Copies for the worker: it must not touch Items, which only the game thread changes.
			IImageWrapperModule* ImageWrappers = Wrappers;
			const ELayer Layer = Item.Layer;
			const FTileSpec Tile = Item.Spec;
			const TSharedPtr<FBuildingChunk, ESPMode::ThreadSafe> Chunk = Item.Chunk;
			Item.Pending = Async(EAsyncExecution::ThreadPool, [Job, ImageWrappers, Layer, Tile, Chunk]()
			{
				if (Layer == ELayer::Buildings)
				{
					BuildingTransforms(*Chunk, Job->Boxes);
					Job->bOk = true;
				}
				else
				{
					Job->bOk = LoadTileData(*ImageWrappers, Tile, Job->Tile, Job->Error);
				}
			});
			++InFlight;
		}

		// Game thread.
		void Create(FItem& Item, const FJob& Job)
		{
			if (Item.Layer == ELayer::Buildings)
			{
				AActor* OwnerActor = Owner.Get();
				for (int32 t = 0; t < ToneCount; ++t)
				{
					if (Job.Boxes[t].Num() == 0)
					{
						continue;
					}
					UInstancedStaticMeshComponent* Ism = NewObject<UInstancedStaticMeshComponent>(OwnerActor);
					Ism->SetMobility(EComponentMobility::Static);
					Ism->SetStaticMesh(Shapes.Cube);
					Ism->SetCollisionEnabled(ECollisionEnabled::NoCollision);
					Ism->SetupAttachment(Parent.Get());
					Ism->SetMaterial(0, Shapes.Tint(OwnerActor, BuildingTones[t]));
					Ism->SetCullDistances(BuildingCullStartCm, BuildingCullEndCm);
					// Instances before registering: the render state is built once, not rebuilt per batch.
					Ism->AddInstances(Job.Boxes[t], false);
					Ism->RegisterComponent();
					Item.Instances.Emplace(Ism);
				}
				Item.bResident = true;
				return;
			}
			// A region tile under a resident detailed one is kept hidden, ready for when that one goes.
			const bool bCovered = Item.Layer == ELayer::Region && Item.Twin != INDEX_NONE && Items[Item.Twin].bResident;
			UProceduralMeshComponent* Mesh = CreateTile(Job.Tile, !bCovered);
			if (!Mesh)
			{
				UE_LOG(LogA320, Warning, TEXT("Terrain %s: no material"), *Item.Spec.Name);
				Item.bFailed = true;
				return;
			}
			Item.Mesh = Mesh;
			Item.bResident = true;
			if (Item.Layer == ELayer::Detail && Item.Twin != INDEX_NONE)
			{
				SetShown(Items[Item.Twin], false);
			}
		}

		void Unload(FItem& Item)
		{
			if (UProceduralMeshComponent* Mesh = Item.Mesh.Get())
			{
				Mesh->DestroyComponent();
			}
			Item.Mesh.Reset();
			for (const TWeakObjectPtr<UInstancedStaticMeshComponent>& Ism : Item.Instances)
			{
				if (Ism.IsValid())
				{
					Ism->DestroyComponent();
				}
			}
			Item.Instances.Reset();
			Item.bResident = false;
			if (Item.Layer == ELayer::Detail && Item.Twin != INDEX_NONE)
			{
				SetShown(Items[Item.Twin], true);
			}
		}

		static void SetShown(const FItem& Item, bool bShown)
		{
			if (UProceduralMeshComponent* Mesh = Item.Mesh.Get())
			{
				Mesh->SetVisibility(bShown);
			}
		}

		// The tiles right under the aircraft, loaded now on this thread.
		void Prime(const FVector2D& Viewer)
		{
			int32 Loaded = 0;
			for (FItem& Item : Items)
			{
				// Whichever of a square's two tiles the streaming would settle on: the detailed one when near.
				const double Distance = DistanceM(Item, Viewer);
				const bool bTwinWanted = Item.Twin != INDEX_NONE &&
					(Item.Layer == ELayer::Region ? Distance < DetailLoadM : Distance >= DetailLoadM);
				if (Item.Layer == ELayer::Buildings || bTwinWanted || Item.bResident || Item.bFailed ||
					Item.Pending.IsValid() || DistanceToSquareM(Item.Spec, Viewer) > PrimeReachM)
				{
					continue;
				}
				FJob Job;
				Job.bOk = LoadTileData(*Wrappers, Item.Spec, Job.Tile, Job.Error);
				if (!Job.bOk)
				{
					UE_LOG(LogA320, Warning, TEXT("Terrain %s: %s"), *Item.Spec.Name, *Job.Error);
					Item.bFailed = true;
					continue;
				}
				Create(Item, Job);
				Loaded += Item.bResident ? 1 : 0;
			}
			UE_LOG(LogA320, Log, TEXT("Terrain: %d tiles loaded under the aircraft at %.1f km N, %.1f km E"), Loaded,
				Viewer.X / 1000.0, Viewer.Y / 1000.0);
		}

		TWeakObjectPtr<AActor> Owner;
		TWeakObjectPtr<USceneComponent> Parent;
		FA320Shapes& Shapes;
		UMaterialInterface* Unlit = nullptr;  // a UPROPERTY of the owner
		IImageWrapperModule* Wrappers = nullptr;
		TStrongObjectPtr<UMaterialInterface> Material;  // kept even while no tile uses it
		FName Parameter = NAME_None;
		TArray<FItem> Items;
		TArray<TPair<double, int32>> Wanted;
		int32 InFlight = 0;
		FVector2D LastViewer = FVector2D::ZeroVector;
		bool bHasViewer = false;
	};
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

	// tile= is always loaded; detail= and region= (files under Region/) stream around the aircraft.
	const TSharedRef<FStreamer> Streamer = MakeShared<FStreamer>(Owner, Parent, Shapes, UnlitTextureMaterial);
	FString BuildingsFile;
	for (const FString& Line : Lines)
	{
		FString Key, Value;
		if (Line.StartsWith(TEXT("#")) || !Line.Split(TEXT("="), &Key, &Value))
		{
			continue;
		}
		FTileSpec Tile;
		if (Key == TEXT("tile") && ParseTile(Value, Dir, Tile))
		{
			Result.Tiles += Streamer->LoadFixed(Tile) ? 1 : 0;
		}
		else if (Key == TEXT("detail") && ParseTile(Value, Dir, Tile))
		{
			Streamer->AddTile(ELayer::Detail, Tile);
		}
		else if (Key == TEXT("region") && ParseTile(Value, FPaths::Combine(Dir, TEXT("Region")), Tile))
		{
			Streamer->AddTile(ELayer::Region, Tile);
		}
		else if (Key == TEXT("buildings") && !Value.Contains(TEXT("/")) && !Value.Contains(TEXT("\\")))
		{
			BuildingsFile = Value;
		}
	}
	Streamer->LinkTwins();
	Result.Streamed = Streamer->Num(ELayer::Detail) + Streamer->Num(ELayer::Region);
	Result.bLoaded = Result.Tiles + Result.Streamed > 0;
	if (Result.bLoaded && !BuildingsFile.IsEmpty())
	{
		Result.Buildings = Streamer->AddBuildings(FPaths::Combine(Dir, BuildingsFile));
	}
	if (Result.Streamed > 0 || Result.Buildings > 0)
	{
		Result.Streamer = TSharedPtr<FA320TerrainStreamer>(Streamer);
	}
	UE_LOG(LogA320, Log, TEXT("Terrain: %d tiles loaded, %d streamed, %d buildings in %d chunks from %s"), Result.Tiles,
		Result.Streamed, Result.Buildings, Streamer->Num(ELayer::Buildings), *Dir);
	return Result;
}
