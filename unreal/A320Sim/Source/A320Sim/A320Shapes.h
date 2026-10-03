#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

#include "A320Shapes.generated.h"

class UMaterialInstanceDynamic;
class UMaterialInterface;
class UStaticMesh;

// Engine basic shapes and a tintable material: the MVP draws everything from primitives,
// so the repository needs no binary assets.
USTRUCT()
struct FA320Shapes
{
	GENERATED_BODY()

	UPROPERTY() TObjectPtr<UStaticMesh> Cube = nullptr;      // 1 m, centred
	UPROPERTY() TObjectPtr<UStaticMesh> Cylinder = nullptr;  // 1 m diameter, 1 m along Z, centred
	UPROPERTY() TObjectPtr<UStaticMesh> Sphere = nullptr;    // 1 m diameter, centred
	UPROPERTY() TObjectPtr<UStaticMesh> Cone = nullptr;      // 1 m, apex up
	UPROPERTY() TObjectPtr<UMaterialInterface> Material = nullptr;

	// Must be called from a UObject constructor (uses ConstructorHelpers).
	void LoadInConstructor();

	// One dynamic material per colour, cached on the owner.
	UMaterialInstanceDynamic* Tint(UObject* Owner, const FLinearColor& Color);

private:
	UPROPERTY() TMap<uint32, TObjectPtr<UMaterialInstanceDynamic>> TintCache;
};
