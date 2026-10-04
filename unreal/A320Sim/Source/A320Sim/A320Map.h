#pragma once

#include "CoreMinimal.h"

#include "a320/WorldMap.h"
#include "a320/a320_api.h"

class UTexture2D;

// The world map's data: the tile pyramid and the places (Content/Map, see tools/make_map.py),
// with the sim's airports added as places so they can be searched and labelled too.
class FA320MapData
{
public:
	// Without Content/Map the map still shows the airports and the aircraft, on a plain background.
	void Load(const FString& Dir, const TArray<A320AirportInfo>& Airports);
	bool IsLoaded() const { return bLoaded; }
	bool HasTiles() const { return !Manifest.tiles.empty(); }
	const a320::worldmap::Manifest& GetManifest() const { return Manifest; }
	const std::vector<a320::worldmap::Place>& GetPlaces() const { return Places; }
	FString TilePath(int32 Level, int32 Row, int32 Col) const;
	static FString NameOf(const a320::worldmap::Place& Place);

	// Game thread: a tile's JPEG as a texture, or null.
	static UTexture2D* LoadTileTexture(const FString& Path);

private:
	FString Dir;
	a320::worldmap::Manifest Manifest;
	std::vector<a320::worldmap::Place> Places;
	bool bLoaded = false;
};
