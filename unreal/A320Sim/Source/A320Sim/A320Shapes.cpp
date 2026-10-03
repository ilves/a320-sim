#include "A320Shapes.h"

#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

void FA320Shapes::LoadInConstructor()
{
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereFinder(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> ConeFinder(TEXT("/Engine/BasicShapes/Cone.Cone"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> MaterialFinder(
		TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	Cube = CubeFinder.Object;
	Cylinder = CylinderFinder.Object;
	Sphere = SphereFinder.Object;
	Cone = ConeFinder.Object;
	Material = MaterialFinder.Object;
}

UMaterialInstanceDynamic* FA320Shapes::Tint(UObject* Owner, const FLinearColor& Color)
{
	const uint32 Key = Color.ToFColor(true).ToPackedARGB();
	if (TObjectPtr<UMaterialInstanceDynamic>* Found = TintCache.Find(Key))
	{
		return *Found;
	}
	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Material, Owner);
	Mid->SetVectorParameterValue(TEXT("Color"), Color);
	TintCache.Add(Key, Mid);
	return Mid;
}
