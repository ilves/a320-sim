#pragma once

#include "CoreMinimal.h"

class AActor;
class UMaterialInterface;
class USceneComponent;
struct FA320Shapes;

// Real-world scenery from Content/Terrain, written by tools/make_terrain.py: height-field tiles
// draped with satellite imagery, and OpenStreetMap buildings as boxes. Placed in the world's
// frame (X north, Y east, Z up from field elevation; cm).
struct FA320TerrainResult
{
	bool bLoaded = false;
	int32 Tiles = 0;
	int32 Buildings = 0;
};

namespace A320Terrain
{
	// UnlitTextureMaterial: an engine material with a "SlateUI" texture parameter, used when a
	// lit terrain material can't be built at runtime (cooked builds have no material compiler).
	FA320TerrainResult Build(AActor* Owner, USceneComponent* Parent, FA320Shapes& Shapes,
		UMaterialInterface* UnlitTextureMaterial);
}
