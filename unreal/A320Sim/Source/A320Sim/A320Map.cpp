#include "A320Map.h"

#include "Engine/Texture2D.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

#include <string>

namespace
{
	// The data files are UTF-8 without a byte order mark: read the bytes as they are.
	std::string ReadUtf8(const FString& Path)
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, *Path))
		{
			return std::string();
		}
		return std::string(reinterpret_cast<const char*>(Bytes.GetData()), static_cast<size_t>(Bytes.Num()));
	}
}

void FA320MapData::Load(const FString& InDir, const TArray<A320AirportInfo>& Airports)
{
	Dir = InDir;
	Manifest = a320::worldmap::Manifest{};
	Manifest.parse(ReadUtf8(FPaths::Combine(Dir, TEXT("map.txt"))));
	Places = a320::worldmap::parsePlaces(ReadUtf8(FPaths::Combine(Dir, TEXT("places.txt"))));
	for (int32 i = 0; i < Airports.Num(); ++i)
	{
		const A320AirportInfo& A = Airports[i];
		a320::worldmap::Place P;
		P.type = "airport";
		P.name = std::string(A.name) + " (" + A.icao + ")";
		P.key = a320::worldmap::fold(P.name);
		P.northM = A.northM;
		P.eastM = A.eastM;
		P.rank = 1000000000LL;  // above every town for labels
		P.airport = i;
		Places.insert(Places.begin() + i, P);
	}
	bLoaded = true;
}

FString FA320MapData::TilePath(int32 Level, int32 Row, int32 Col) const
{
	return FPaths::Combine(Dir, FString::Printf(TEXT("L%d"), Level), FString::Printf(TEXT("%d_%d.jpg"), Row, Col));
}

FString FA320MapData::NameOf(const a320::worldmap::Place& Place)
{
	return FString(UTF8_TO_TCHAR(Place.name.c_str()));
}

UTexture2D* FA320MapData::LoadTileTexture(const FString& Path)
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
		!Wrapper->GetRaw(ERGBFormat::BGRA, 8, Raw))
	{
		return nullptr;
	}
	const int32 W = static_cast<int32>(Wrapper->GetWidth()), H = static_cast<int32>(Wrapper->GetHeight());
	if (W <= 0 || H <= 0 || Raw.Num() != static_cast<int64>(W) * H * 4)
	{
		return nullptr;
	}
	UTexture2D* Texture = UTexture2D::CreateTransient(W, H, PF_B8G8R8A8);
	if (!Texture)
	{
		return nullptr;
	}
	Texture->SRGB = true;
	Texture->NeverStream = true;
	Texture->AddressX = TA_Clamp;
	Texture->AddressY = TA_Clamp;
	FTexture2DMipMap& Mip = Texture->GetPlatformData()->Mips[0];
	Mip.BulkData.Lock(LOCK_READ_WRITE);
	FMemory::Memcpy(Mip.BulkData.Realloc(Raw.Num()), Raw.GetData(), Raw.Num());
	Mip.BulkData.Unlock();
	Texture->UpdateResource();
	return Texture;
}
