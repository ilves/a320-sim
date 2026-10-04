#pragma once

#include "CoreMinimal.h"

class AActor;
class UMaterialInterface;
class USceneComponent;
struct FA320Shapes;

// Keeps the streamed scenery (detailed and region tiles, building chunks) loaded around the
// aircraft: near pieces are decoded on worker threads and created a few per frame, far ones dropped.
class FA320TerrainStreamer
{
public:
	virtual ~FA320TerrainStreamer() = default;
	// Game thread, every frame. ViewerCm: the aircraft in the world frame.
	virtual void Tick(const FVector& ViewerCm) = 0;
	// Waits for work still running on worker threads; call before the owner goes away.
	virtual void Shutdown() = 0;
};

// Real-world scenery from Content/Terrain, written by tools/make_terrain.py: height-field tiles
// draped with satellite imagery, and OpenStreetMap buildings as boxes. Placed in the world's
// frame (X north, Y east, Z up from field elevation; cm).
struct FA320TerrainResult
{
	bool bLoaded = false;
	int32 Tiles = 0;     // always loaded
	int32 Streamed = 0;  // tiles that stream in and out
	int32 Buildings = 0;
	// Null when the data has nothing to stream (an older manifest loads everything up front).
	TSharedPtr<FA320TerrainStreamer> Streamer;
};

namespace A320Terrain
{
	// UnlitTextureMaterial: an engine material with a "SlateUI" texture parameter, used when a
	// lit terrain material can't be built at runtime (cooked builds have no material compiler).
	// Owner and Shapes must outlive the returned streamer (the owning actor holds all three).
	FA320TerrainResult Build(AActor* Owner, USceneComponent* Parent, FA320Shapes& Shapes,
		UMaterialInterface* UnlitTextureMaterial);
}
