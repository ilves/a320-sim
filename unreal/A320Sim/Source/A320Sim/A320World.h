#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "A320Shapes.h"
#include "A320Terrain.h"
#include "a320/a320_api.h"

#include "A320World.generated.h"

class APostProcessVolume;
class UDirectionalLightComponent;
class UMaterialInstanceDynamic;
class UTexture2D;
class UExponentialHeightFogComponent;
class UInstancedStaticMeshComponent;
class UMaterialInterface;
class USkyAtmosphereComponent;
class USkyLightComponent;
class UStaticMeshComponent;

// Sky, sun, terrain and the airport, built at runtime from the core's runway data. The terrain is
// the real one when Content/Terrain exists (see A320Terrain.h), otherwise flat grass and water.
// The world is flat: X north, Y east, Z up from the first airport's field elevation (cm); other
// airports' runways sit at their own elevation.
UCLASS()
class AA320World : public AActor
{
	GENERATED_BODY()

public:
	AA320World();

	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

	// Runways sit at their airport's level (A320AirportInfo.elevationM), where the terrain is flattened.
	void Build(const TArray<A320RunwayInfo>& Runways, const TArray<A320AirportInfo>& Airports);
	// papiWhite: left to right as seen on approach.
	void UpdatePapi(int32 RunwayIndex, const int PapiWhite[4]);
	// The weather (A320Weather, a320_weather_info) and day or night: the sun or the moon, the sky
	// light, the fog, a cloud deck, rain around the camera, and the exposure (A320WorldWeather.cpp).
	void SetWeather(int32 InWeather, bool bInNight);

private:
	UStaticMeshComponent* AddMesh(UStaticMesh* Mesh, const FVector& CentreM, double YawDeg, const FVector& SizeM,
		const FLinearColor& Color);
	UStaticMeshComponent* AddMesh(UStaticMesh* Mesh, const FTransform& Transform, const FLinearColor& Color);
	UInstancedStaticMeshComponent* AddInstanced(UStaticMesh* Mesh, const FLinearColor& Color);
	void BuildPavedRunway(const A320RunwayInfo& Runway, double LevelM);
	void BuildRunwayDirection(const A320RunwayInfo& Runway, int32 Index, double LevelM);
	// Taxiways and aprons from Content/Airports/<ICAO>.txt (tools/make_terrain.py, OpenStreetMap).
	void BuildAirportLayouts();
	void BuildSurroundings(const A320RunwayInfo& Runway);
	// Runway, approach and PAPI lights: unlit, so they shine at night and in fog.
	UMaterialInterface* Glow(const FLinearColor& Color);
	void BuildWeather();
	void TickWeather(float DeltaSeconds, const FVector& CameraCm);
	void MakeCloudTexture(double Cover);

	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<USkyAtmosphereComponent> Sky;
	UPROPERTY() TObjectPtr<UDirectionalLightComponent> Sun;
	UPROPERTY() TObjectPtr<USkyLightComponent> SkyLight;
	UPROPERTY() TObjectPtr<UExponentialHeightFogComponent> Fog;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> PapiLights;  // 4 per runway direction
	UPROPERTY() FA320Shapes Shapes;
	UPROPERTY() TObjectPtr<UMaterialInterface> UnlitTextureMaterial;
	TArray<int32> PapiShown;
	UPROPERTY() TMap<uint32, TObjectPtr<UMaterialInstanceDynamic>> GlowCache;
	UPROPERTY() TArray<TObjectPtr<UTexture2D>> GlowTextures;
	UPROPERTY() TObjectPtr<UStaticMesh> PlaneMesh;
	UPROPERTY() TObjectPtr<APostProcessVolume> Exposure;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> CloudSheets;  // the deck's underside and top
	UPROPERTY() TObjectPtr<UMaterialInstanceDynamic> CloudMaterial;
	UPROPERTY() TObjectPtr<UTexture2D> CloudTexture;
	UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> Rain;
	int32 Weather = 0;
	bool bNight = false;
	A320WeatherInfo WeatherInfo{};
	double ClearFogDensity = 0.002;  // the fog outside the clouds; in them it closes in
	double RainFallCm = 0.0;
	// Streams terrain tiles and buildings around the aircraft each tick; null without streamed data.
	TSharedPtr<FA320TerrainStreamer> TerrainStreamer;
};
